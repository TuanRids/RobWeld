#include "pch.h"
#include "zmpdata.h"
#include "ui/FrameManage.h"
zmpdata::zmpdata() : image_texture(0) {}

zmpdata::~zmpdata() {
    if (image_texture != 0) {
        glDeleteTextures(1, &image_texture);
    }
}


void zmpdata::render_send() {
    std::lock_guard<std::mutex> lock(mtx);  // Lock mutex

    const char* ipc_send_mapping = Config::IPC_SEND_MAPPING;

    HANDLE hMapFile = CreateFileMapping(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, 0, 1024, TEXT(ipc_send_mapping));
    if (hMapFile == NULL) {
        return;
    }

    LPCTSTR pBuf = (LPCTSTR)MapViewOfFile(hMapFile, FILE_MAP_ALL_ACCESS, 0, 0, 1024);
    if (pBuf == NULL) {
        CloseHandle(hMapFile);
        return;
    }

    // Serialize TriggerToPy map to a string format
    std::string data_to_send;
    for (const auto& kv : TriggerToPy) {
        data_to_send += kv.first + ":" + std::to_string(kv.second) + ";";
    }

    // Write data to shared memory
    CopyMemory((PVOID)pBuf, data_to_send.c_str(), data_to_send.size() + 1);

    // Clean up
    UnmapViewOfFile(pBuf);
    CloseHandle(hMapFile);
}

void zmpdata::render() {
    auto start = std::chrono::high_resolution_clock::now();

    cv::Mat img;
    int frame_count;
    float rotation_speed;

    if (receive_data(img, frame_count, rotation_speed)) {
        if (!img.empty()) {
            // Call Imshow to reformat data for ImGui
            cv::imshow("Hidden", img);
            cv::waitKey(1);

            // Hide the main window
            HWND hwnd = FindWindow(NULL, TEXT("Hidden"));
            if (hwnd != NULL) { ShowWindow(hwnd, SW_HIDE); }
            if (image_texture != 0) { glDeleteTextures(1, &image_texture); }

            // Convert image to texture for ImGui
            image_texture = matToTexture(img, GL_LINEAR, GL_LINEAR, GL_CLAMP_TO_EDGE);
            // std::cout << "Received frame count: " << frame_count << ", rotation speed: " << rotation_speed << std::endl;
        }
    }

    static float pos_x, pos_y, size_x, size_y;
    nui::FrameManage::getViewportSize(pos_x, pos_y);
    nui::FrameManage::get3DSize(size_x, size_y);

    ImGui::SetNextWindowPos(ImVec2(pos_x + 15+ size_x*0.7, pos_y + 35)); // Set the position of the frame
    ImGui::SetNextWindowSize(ImVec2(size_x*0.295, size_y * 0.8)); // Set the size of the frame
    ImGui::Begin("Camera Vision", nullptr,
        ImGuiWindowFlags_NoBackground); // Do not display background
    
    if (ImGui::Button("Send 0")) { TriggerToPy["Send1"] = 1; } ImGui::SameLine();
    if (ImGui::Button("Send 1")) { TriggerToPy["Send2"] = 1; }ImGui::SameLine();
    if (ImGui::Button("Send 2")) { TriggerToPy["Send3"] = 1; }ImGui::SameLine();
    if (ImGui::Button("Send 3")) { TriggerToPy["Send4"] = 1; }
    if (image_texture != 0) {
        ImVec2 winsize = ImGui::GetWindowSize();  // get the size of the window
        float scale_ratio = (float)img.cols / (float)img.rows;

        ImVec2 image_size;
        if (winsize.x / scale_ratio > winsize.y) {
            image_size.x = winsize.y * scale_ratio;
            image_size.y = winsize.y;
        }
		else {
			image_size.x = winsize.x;
			image_size.y = winsize.x / scale_ratio;
		}
        ImGui::Image((void*)(intptr_t)image_texture, image_size);
    }
    ImGui::End();

    render_send();
    reset_TriggerToPy();
}

void zmpdata::reset_TriggerToPy() {
    static auto start = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() > 500) {  // Changed to 500ms
        TriggerToPy["Send1"] = 0;
        TriggerToPy["Send2"] = 0;
        TriggerToPy["Send3"] = 0;
        TriggerToPy["Send4"] = 0;
        start = std::chrono::high_resolution_clock::now();
    }
}

bool zmpdata::receive_data(cv::Mat& img, int& frame_count, float& rotation_speed) {
    std::lock_guard<std::mutex> lock(mtx);  // Lock mutex
    const char* img_shm_name = Config::IPC_GET_IMG;
    const char* data_shm_name = Config::IPC_GET_DATA;

    // Open the shared memory objects
    HANDLE hMapFileImg = OpenFileMapping(FILE_MAP_ALL_ACCESS, FALSE, TEXT(img_shm_name));
    HANDLE hMapFileData = OpenFileMapping(FILE_MAP_ALL_ACCESS, FALSE, TEXT(data_shm_name));

    if (hMapFileImg == NULL || hMapFileData == NULL) {
        return false;
    }
    // Map the shared memory objects
    LPCTSTR pBufImg = (LPCTSTR)MapViewOfFile(hMapFileImg, FILE_MAP_ALL_ACCESS, 0, 0, 0);
    LPCTSTR pBufData = (LPCTSTR)MapViewOfFile(hMapFileData, FILE_MAP_ALL_ACCESS, 0, 0, 0);

    if (pBufImg == NULL || pBufData == NULL) {
        if (pBufImg != NULL) UnmapViewOfFile(pBufImg);
        if (pBufData != NULL) UnmapViewOfFile(pBufData);
        CloseHandle(hMapFileImg);
        CloseHandle(hMapFileData);
        return false;
    }

    // Read image data from shared memory
    std::vector<uchar> img_vec(reinterpret_cast<const uchar*>(pBufImg), reinterpret_cast<const uchar*>(pBufImg) + 640 * 480 * 3);
    img = cv::imdecode(img_vec, cv::IMREAD_COLOR);

    // Read frame count and rotation speed from shared memory
    void* data_ptr = (void*)pBufData;
    std::memcpy(&frame_count, data_ptr, sizeof(int));
    std::memcpy(&rotation_speed, static_cast<char*>(data_ptr) + sizeof(int), sizeof(float));

    // Clean up
    UnmapViewOfFile(pBufImg);
    UnmapViewOfFile(pBufData);
    CloseHandle(hMapFileImg);
    CloseHandle(hMapFileData);

    return true;
}

GLuint zmpdata::matToTexture(const cv::Mat& mat, GLenum minFilter, GLenum magFilter, GLenum wrapFilter) {
    // Generate a number for our textureID's unique handle
    GLuint textureID;
    glGenTextures(1, &textureID);

    // Bind to our texture handle
    glBindTexture(GL_TEXTURE_2D, textureID);

    // Catch silly-mistake texture interpolation method for magnification
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, minFilter);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, magFilter);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrapFilter);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrapFilter);

    // Copy image data into the texture
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, mat.cols, mat.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, mat.ptr());

    return textureID;
}
