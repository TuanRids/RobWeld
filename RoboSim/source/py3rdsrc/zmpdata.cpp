#include "pch.h"
#include "zmpdata.h"

zmpdata::zmpdata() : image_texture(0) {    sttlogs = &nui::StatusLogs::getInstance();}

std::vector<std::vector<float>> zmpdata::shared_get6pos = std::vector<std::vector<float>>(50, std::vector<float>(6, -99999.0f));

zmpdata::~zmpdata() {
    if (image_texture != 0) {
        glDeleteTextures(1, &image_texture);
    }
}

void zmpdata::send_datatoIPC() {

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

void zmpdata::Display_info()
{
    static float pos_x, pos_y, size_x, size_y;
    nui::FrameManage::getViewportSize(pos_x, pos_y);
    nui::FrameManage::get3DSize(size_x, size_y);

    ImGui::SetNextWindowPos(ImVec2(pos_x + 15, pos_y + 35)); // Set the position of the frame
    ImGui::SetNextWindowSize(ImVec2(size_x * 0.2, size_y * 0.97 - 35)); // Set the size of the frame
    ImGui::Begin("Camera Vision (R)", nullptr,
        ImGuiWindowFlags_NoDocking |    // Cannot be docked
        ImGuiWindowFlags_NoBackground | // Do not display background
        ImGuiWindowFlags_NoNavFocus);

    if (ImGui::BeginPopupContextItem("Vision Popup", ImGuiPopupFlags_MouseButtonRight)) {
        if (ImGui::MenuItem("Run Inspection Plan")) { TriggerToPy["Send1"] = 1; }
        if (ImGui::MenuItem("Start Robot Left")) { TriggerToPy["Send2"] = 1; }
        if (ImGui::MenuItem("Start Robot Right")) { TriggerToPy["Send3"] = 1; }
        if (ImGui::MenuItem("Stop RB")) { TriggerToPy["Send4"] = 1; }
        if (ImGui::MenuItem("Clean")) { clean_image(); }
        if (ImGui::MenuItem("Home")) { TriggerToPy["Send5"] = 1; }
        if (ImGui::MenuItem("Stop All")) { TriggerToPy["Send6"] = 1; }
        ImGui::EndPopup();
    }
    
    if (ImGui::Button("R Insp")) { TriggerToPy["Send1"] = 1; }   ImGui::SameLine();
    if (ImGui::Button("R Left")) { TriggerToPy["Send2"] = 1; }  ImGui::SameLine();
    if (ImGui::Button("R Right")) { TriggerToPy["Send3"] = 1; } ImGui::SameLine();
    if (ImGui::Button("Stop")) { TriggerToPy["Send4"] = 1; }

    if (image_texture != 0) {
        ImVec2 winsize = ImGui::GetWindowSize();  // get the size of the window
        float scale_ratio = (float)img.cols / (float)img.rows;

        ImVec2 image_size;
        float half_winsize_y = winsize.y / 2.0f;
        if (winsize.x / scale_ratio > half_winsize_y) {
            image_size.x = half_winsize_y * scale_ratio;
            image_size.y = half_winsize_y;
        }
        else {
            image_size.x = winsize.x;
            image_size.y = winsize.x / scale_ratio;
        }
        ImGui::Image((void*)(intptr_t)image_texture, image_size);
    }
    
    if (image_texture_below != 0) {
        ImVec2 winsize = ImGui::GetWindowSize();  // get the size of the window
        float scale_ratio = (float)img_below.cols / (float)img_below.rows;

        ImVec2 image_size;
        float half_winsize_y = winsize.y / 2.0f;
        if (winsize.x / scale_ratio > half_winsize_y) {
            image_size.x = half_winsize_y * scale_ratio;
            image_size.y = half_winsize_y;
        }
        else {
            image_size.x = winsize.x;
            image_size.y = winsize.x / scale_ratio;
        }
        ImGui::Image((void*)(intptr_t)image_texture_below, image_size);
    }
    ImGui::End();
}

void zmpdata::getter_6pos(std::vector<std::vector<float>>& get6pos) {
    std::vector<std::vector<float>> temptemp;
    const float epsilon = 1.1f; 

    for (const auto& pos : shared_get6pos) {
        float cc = std::fabs(pos[0] + 99999.0f);
        if (cc > epsilon) {
            temptemp.push_back(pos);
        }
        else {
            break;
        }
    }

    if (!temptemp.empty()) {
        get6pos = temptemp;
    }
}


void zmpdata::render() {

    int frame_count;
    float rotation_speed;

    if (receive_data(img, img_below, frame_count, rotation_speed)) {
        if (!img.empty()) {
            // Delete old texture
            if (image_texture != 0) {
                glDeleteTextures(1, &image_texture);
                image_texture = 0; 
            }
            if (image_texture != 0) { glDeleteTextures(1, &image_texture); }
            // Convert image to texture for ImGui
            image_texture = matToTexture(img, GL_LINEAR, GL_LINEAR, GL_CLAMP_TO_EDGE);
        }
        if (!img_below.empty()) {
            // delete old texture
            if (image_texture_below != 0) {
                glDeleteTextures(1, &image_texture_below);
                image_texture_below = 0;
            }
            if (image_texture_below != 0) { glDeleteTextures(1, &image_texture_below); }

            // Convert image to texture for ImGui
            image_texture_below = matToTexture(img_below, GL_LINEAR, GL_LINEAR, GL_CLAMP_TO_EDGE);
        }
    }
    
    Display_info();
    send_datatoIPC();
    reset_TriggerToPy();
}

void zmpdata::clean_image()
{
    glDeleteTextures(1, &image_texture);
    image_texture = 0;
    glDeleteTextures(1, &image_texture_below);
    image_texture_below = 0;
}

void zmpdata::reset_TriggerToPy() {
    static auto start = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() > 500) {  // Changed to 500ms
        for (int i = 1; i <= 9; ++i) {
            TriggerToPy["Send" + std::to_string(i)] = 0;
        }
        start = std::chrono::high_resolution_clock::now();
    }
}


bool zmpdata::receive_data(cv::Mat& img, cv::Mat& img_below, int& frame_count, float& rotation_speed) {
    const char* img_shm_name = Config::IPC_GET_IMG;             // Get Image
    const char* img_shm_name_below = Config::IPC_GET_IMG_BELOW; // Get Image Below
    const char* data_shm_name = Config::IPC_GET_DATA;           // Get Data
    const char* data_shm_rospos = Config::IPC_GET_POS;          // Get ROS Position
    const char* dat_shm_status = Config::IPC_GET_STATUS;        // Get Status  

    // Open the shared memory objects
    HANDLE hMapFileImg = OpenFileMapping(FILE_MAP_ALL_ACCESS, FALSE, TEXT(img_shm_name));
    HANDLE hMapFileImgBelow = OpenFileMapping(FILE_MAP_ALL_ACCESS, FALSE, TEXT(img_shm_name_below));
    HANDLE hMapFileData = OpenFileMapping(FILE_MAP_ALL_ACCESS, FALSE, TEXT(data_shm_name));
    HANDLE hMapFileRosPos = OpenFileMapping(FILE_MAP_ALL_ACCESS, FALSE, TEXT(data_shm_rospos));
    HANDLE hMapFileStatus = OpenFileMapping(FILE_MAP_ALL_ACCESS, FALSE, TEXT(dat_shm_status));

    if (hMapFileImg == NULL && hMapFileImgBelow == NULL && hMapFileData == NULL && hMapFileRosPos == NULL || hMapFileStatus == NULL) {
        return false;
    }

    // Map the shared memory objects
    LPCTSTR pBufImg = (hMapFileImg != NULL) ? (LPCTSTR)MapViewOfFile(hMapFileImg, FILE_MAP_ALL_ACCESS, 0, 0, 0) : NULL;
    LPCTSTR pBufImgBelow = (hMapFileImgBelow != NULL) ? (LPCTSTR)MapViewOfFile(hMapFileImgBelow, FILE_MAP_ALL_ACCESS, 0, 0, 0) : NULL;
    LPCTSTR pBufData = (hMapFileData != NULL) ? (LPCTSTR)MapViewOfFile(hMapFileData, FILE_MAP_ALL_ACCESS, 0, 0, 0) : NULL;
    LPCTSTR pBufPos = (hMapFileRosPos != NULL) ? (LPCTSTR)MapViewOfFile(hMapFileRosPos, FILE_MAP_ALL_ACCESS, 0, 0, 0) : NULL;
    LPCTSTR pBufStatus = (hMapFileStatus != NULL) ? (LPCTSTR)MapViewOfFile(hMapFileStatus, FILE_MAP_ALL_ACCESS, 0, 0, 0) : NULL;
    // Check if all mapped buffers are NULL
    if (pBufImg == NULL && pBufImgBelow == NULL && pBufData == NULL) {
        if (pBufImg != NULL) UnmapViewOfFile(pBufImg);
        if (pBufImgBelow != NULL) UnmapViewOfFile(pBufImgBelow);
        if (pBufData != NULL) UnmapViewOfFile(pBufData);
        if (hMapFileImg != NULL) CloseHandle(hMapFileImg);
        if (hMapFileImgBelow != NULL) CloseHandle(hMapFileImgBelow);
        if (hMapFileData != NULL) CloseHandle(hMapFileData);
        if (hMapFileRosPos != NULL) CloseHandle(hMapFileRosPos);
        if (hMapFileStatus != NULL) CloseHandle(hMapFileStatus);
        return false;
    }

    // Read image data from shared memory (IMG)
    if (pBufImg != NULL) {
        std::vector<uchar> img_vec(reinterpret_cast<const uchar*>(pBufImg), reinterpret_cast<const uchar*>(pBufImg) + 640 * 480 * 3);
        img = cv::imdecode(img_vec, cv::IMREAD_COLOR);
        UnmapViewOfFile(pBufImg);
        CloseHandle(hMapFileImg);
    }
    else { img.release(); }

    // Read image data from shared memory (IMG_BELOW)
    if (pBufImgBelow != NULL) {
        std::vector<uchar> img_below_vec(reinterpret_cast<const uchar*>(pBufImgBelow), reinterpret_cast<const uchar*>(pBufImgBelow) + 640 * 480 * 3);
        img_below = cv::imdecode(img_below_vec, cv::IMREAD_COLOR);
        UnmapViewOfFile(pBufImgBelow);
        CloseHandle(hMapFileImgBelow);
    }
    else { img_below.release(); }

    // Read frame count and rotation speed from shared memory
    if (pBufData != NULL) {
        void* data_ptr = const_cast<void*>(reinterpret_cast<const void*>(pBufData));
        std::memcpy(&frame_count, data_ptr, sizeof(int));
        std::memcpy(&rotation_speed, static_cast<char*>(data_ptr) + sizeof(int), sizeof(float));
        UnmapViewOfFile(pBufData);
        CloseHandle(hMapFileData);
    }
    else {
        frame_count = 0;
        rotation_speed = 0.0f;
    }

    // Read robot position from shared memory, if not return -99999
    if (pBufPos != NULL) {
        float* pos_ptr = const_cast<float*>(reinterpret_cast<const float*>(pBufPos));
        for (size_t i = 0; i < 50; ++i) {
            if (pos_ptr[i * 6] == -99999.0f) { break; } // end of shared memory

            for (size_t j = 0; j < 6; ++j) { shared_get6pos[i][j] = pos_ptr[i * 6 + j]; }
        }
        UnmapViewOfFile(pBufPos);
        CloseHandle(hMapFileRosPos);
    }
    else {
        shared_get6pos[0][0] = -99999.0f;
    }

    // Read status from shared memory
    if (pBufStatus != NULL) {
        unsigned int key;
        char value[252]; // assuming value length is 252 bytes

        // Copy key
        if (memcpy_s(&key, sizeof(unsigned int), pBufStatus, sizeof(unsigned int)) != 0) {
            std::cerr << "Error copying key from shared memory." << std::endl;
        }

        // Copy value
        if (memcpy_s(value, sizeof(value), const_cast<char*>(reinterpret_cast<const char*>(pBufStatus)) + sizeof(unsigned int), 252) != 0) {
            std::cerr << "Error copying value from shared memory." << std::endl;
        }
        value[251] = '\0'; // Ensure null-termination

        if (key != stt_id)
        {
			stt_id = key;
            *sttlogs << value;
        }
        UnmapViewOfFile(pBufStatus);
        CloseHandle(hMapFileStatus);
    }

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
