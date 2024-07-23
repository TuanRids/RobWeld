#include "pch.h"
#include "zmpdata.h"
#include <stdlib.h>
zmpdata::zmpdata() : image_texture(0), image_texture_below{0} 
{ 
    sttlogs = &nui::StatusLogs::getInstance(); }

std::vector<std::vector<float>> zmpdata::shared_get6pos (std::vector<std::vector<float>>(50, std::vector<float>(6, -999.0f)));
std::vector<std::vector<float>> zmpdata::shared_3Ddata (std::vector<std::vector<float>>(5, std::vector<float>(1024, -999.0f)));

zmpdata::~zmpdata() {
    if (image_texture != 0) {
        glDeleteTextures(1, &image_texture);
    }
}

void zmpdata::send_datatoIPC() {

    const char* ipc_send_mapping = Config::IPC_SEND_TRIGGER;

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


void applyGammaCorrection(const cv::Mat& src, cv::Mat& dst, float gamma) {
    CV_Assert(gamma >= 0);
    cv::Mat lut(1, 256, CV_8UC1);
    uchar* p = lut.ptr();
    for (int i = 0; i < 256; i++) {
        p[i] = cv::saturate_cast<uchar>(pow(i / 255.0, gamma) * 255.0);
    }
    cv::LUT(src, lut, dst);
}
void applySaturation(const cv::Mat& src, cv::Mat& dst, float saturationScale) {
    cv::Mat hsv;
    cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> channels;
    cv::split(hsv, channels);
    channels[1] *= saturationScale; // Adjust saturation channel
    cv::merge(channels, hsv);
    cv::cvtColor(hsv, dst, cv::COLOR_HSV2BGR);
}
void applySharpness(const cv::Mat& src, cv::Mat& dst, float alpha) {
    cv::Mat blurred;
    cv::GaussianBlur(src, blurred, cv::Size(0, 0), 5);
    cv::addWeighted(src, 1.0 + alpha, blurred, -alpha, 0, dst);
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
        if (ImGui::MenuItem("Run Inspection Plan")) {  TriggerToPy["Send1"] = 1; }
        if (ImGui::MenuItem("Start Robot Left")) { TriggerToPy["Send2"] = 1; }
        if (ImGui::MenuItem("Start Robot Right")) { TriggerToPy["Send3"] = 1; }
        if (ImGui::MenuItem("Stop RB")) { TriggerToPy["Send4"] = 1; }
        if (ImGui::MenuItem("Clean")) { clean_image(); }
        //if (ImGui::MenuItem("Home")) { TriggerToPy["Send5"] = 1; }
        if (ImGui::MenuItem("Stop All")) { TriggerToPy["Send6"] = 1; }
        ImGui::EndPopup();
    }

    if (ImGui::Button("R Insp")) { TriggerToPy["Send1"] = 1; }   ImGui::SameLine();
    if (ImGui::Button("R Left")) { TriggerToPy["Send2"] = 1; }  ImGui::SameLine();
    if (ImGui::Button("R Right")) { TriggerToPy["Send3"] = 1; } ImGui::SameLine();
    if (ImGui::Button("Stop")) { TriggerToPy["Send4"] = 1; }

    // Process and display main image
    if (!img.empty()) {
        if (image_texture != 0) {
            glDeleteTextures(1, &image_texture);
        }
        image_texture = matToTexture(img);
    }
    if (image_texture != 0) {
        ImVec2 winsize = ImGui::GetWindowSize();  // get the size of the window
        float scale_ratio = (float)img.cols / (float)img.rows;

        ImVec2 image_size;
        if (winsize.x / scale_ratio > winsize.y) {
            image_size.x = winsize.y * scale_ratio;
            image_size.y = winsize.y*0.95;
        }
        else {
            image_size.x = winsize.x;
            image_size.y = winsize.x / scale_ratio * 0.95;
        }
        ImGui::Image((void*)(intptr_t)image_texture, image_size);
    }

    // Process and display secondary image with adjustments
    if (!img_below.empty()) {
        if (image_texture_below != 0) {
            glDeleteTextures(1, &image_texture_below);
        }
        image_texture_below = matToTexture(img_below);
    }
    if (image_texture_below != 0) {
        ImVec2 winsize = ImGui::GetWindowSize();  // get the size of the window
        float scale_ratio = (float)img_below.cols / (float)img_below.rows;

        ImVec2 image_size;
        if (winsize.x / scale_ratio > winsize.y) {
            image_size.x = winsize.y * scale_ratio;
            image_size.y = winsize.y * 0.95;
        }
        else {
            image_size.x = winsize.x;
            image_size.y = winsize.x / scale_ratio * 0.95;
        }
        ImGui::Image((void*)(intptr_t)image_texture_below, image_size);

        if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0)) {
            UnImgFrameTrigger = !UnImgFrameTrigger;
        }
        if (UnImgFrameTrigger)
        {
            ImGui::SetWindowPos(ImVec2(20, 20));
            ImGui::Begin("Laser View", &UnImgFrameTrigger, ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysHorizontalScrollbar | ImGuiWindowFlags_AlwaysVerticalScrollbar);
            ImGui::BeginGroup();

            // Image adjustment sliders
            static int Brightness = 0, Contrast = 0, Sharpness = 0;
            static float Gamma = 0.0f, Saturation = 0.0f;
            static float zoomFactor = 1.0f; // Initialize zoom factor

            ImGui::SetNextItemWidth(80);
            ImGui::SliderInt("Brightness", &Brightness, -100, 100);ImGui::SetNextItemWidth(80);
            ImGui::SliderInt("Contrast", &Contrast, -100, 100);ImGui::SetNextItemWidth(80);
            ImGui::SliderInt("Sharpness", &Sharpness, 0, 100);ImGui::SetNextItemWidth(80);
            ImGui::SliderFloat("Gamma", &Gamma, 0.0f, 3.0f);ImGui::SetNextItemWidth(80);
            ImGui::SliderFloat("Saturation", &Saturation, 0.0f, 3.0f); ImGui::SetNextItemWidth(80);
            ImGui::SliderFloat("ZoomFactor", &zoomFactor, 0.1f, 10.0f);
            ImGui::EndGroup();
            ImGui::SameLine();
            ImGui::BeginGroup();

            // Calculate the size of the image to fit the window
            ImVec2 win_size = ImGui::GetWindowSize();
            float scale_ratio = (float)img_below.cols / (float)img_below.rows;
            ImVec2 img_size;
            if (win_size.x / scale_ratio > win_size.y) {
                img_size.x = win_size.y * scale_ratio;
                img_size.y = win_size.y * 0.95;
            }
            else {
                img_size.x = win_size.x;
                img_size.y = 0.95f * win_size.x / scale_ratio;
            }

            // Apply zoom factor
            img_size.x *= zoomFactor;
            img_size.y *= zoomFactor;

            // Apply image adjustments using OpenCV
            cv::Mat adjusted_img;
            img_below.convertTo(adjusted_img, -1, 1 + Contrast / 100.0, Brightness);
            if (Gamma > 0) { applyGammaCorrection(adjusted_img, adjusted_img, Gamma); }
            if (Saturation > 0) { applySaturation(adjusted_img, adjusted_img, Saturation); }
            if (Sharpness) { applySharpness(adjusted_img, adjusted_img, Sharpness / 100.0); }

            static GLuint adjusted_texture = 0;
            if (adjusted_texture != 0) {
                glDeleteTextures(1, &adjusted_texture);
            }
            adjusted_texture = matToTexture(adjusted_img);

            ImGui::Image((void*)(intptr_t)adjusted_texture, img_size);

            // Handle zooming
            ImGuiIO& io = ImGui::GetIO();
            if (io.MouseWheel != 0) {
                zoomFactor += io.MouseWheel * 0.1f; // Adjust zoom factor with mouse wheel
                if (zoomFactor < 0.1f) zoomFactor = 0.1f; // Prevent zooming out too much
            }
            ImVec2 mouse_pos = io.MousePos;
            ImVec2 window_pos = ImGui::GetWindowPos(); 
            ImVec2 window_size = ImGui::GetWindowSize(); 
            if (ImGui::IsMouseClicked(0)) {
                if (mouse_pos.x < window_pos.x || mouse_pos.x > window_pos.x + window_size.x ||
                    mouse_pos.y < window_pos.y || mouse_pos.y > window_pos.y + window_size.y) {
                    UnImgFrameTrigger = false;
                }
            }
            ImGui::EndGroup();
            ImGui::End();
        }


    }
    ImGui::End();
}


void zmpdata::trigger_3DCreator()
{
    if (shared_3Ddata.size() < 10) { return; }

    // Initialize start to 10 seconds before now
    static auto start = std::chrono::high_resolution_clock::now() - std::chrono::seconds(10);

    if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count() > 10000) {
        std::unique_ptr<PclToMesh> pcl2m = std::make_unique<PclToMesh>();
        pcl2m->setter_data(shared_3Ddata);
        pcl2m->processPointCloud();
        *sttlogs << "Creating 3D Object" ;
        // Reset start to now after processing
        start = std::chrono::high_resolution_clock::now();
        shared_3Ddata = std::vector<std::vector<float>>(5, std::vector<float>(1024, -999.0f));
    }
}

void zmpdata::getter_6pos(std::vector<std::vector<float>>& get6pos) {
    std::vector<std::vector<float>> temptemp;

    for (const auto& pos : shared_get6pos) {
        float cc = std::fabs(pos[0] + 999.0f);
        if (cc > 5.0f) {
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
    receive_data();
    trigger_3DCreator();
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

bool zmpdata::receive_data() {
    const char* img_shm_name = Config::IPC_GET_IMG;             // Get Image
    const char* img_shm_name_below = Config::IPC_GET_IMG_BELOW; // Get Image Below
    const char* data_shm_rospos = Config::IPC_GET_POS;          // Get ROS Position
    const char* dat_shm_status = Config::IPC_GET_STATUS;        // Get Status  
    const char* dat_shm_coord = Config::IPC_GET_COOR3DMESH;     // Get Coordinators

    // Open the shared memory objects
    HANDLE hMapFileImg = OpenFileMapping(FILE_MAP_ALL_ACCESS, FALSE, TEXT(img_shm_name));
    HANDLE hMapFileImgBelow = OpenFileMapping(FILE_MAP_ALL_ACCESS, FALSE, TEXT(img_shm_name_below));
    HANDLE hMapFileRosPos = OpenFileMapping(FILE_MAP_ALL_ACCESS, FALSE, TEXT(data_shm_rospos));
    HANDLE hMapFileStatus = OpenFileMapping(FILE_MAP_ALL_ACCESS, FALSE, TEXT(dat_shm_status));
    HANDLE hMapFileCoord = OpenFileMapping(FILE_MAP_ALL_ACCESS, FALSE, TEXT(dat_shm_coord));


    // Map the shared memory objects
    LPCTSTR pBufImg = (hMapFileImg != NULL) ? (LPCTSTR)MapViewOfFile(hMapFileImg, FILE_MAP_ALL_ACCESS, 0, 0, 0) : NULL;
    LPCTSTR pBufImgBelow = (hMapFileImgBelow != NULL) ? (LPCTSTR)MapViewOfFile(hMapFileImgBelow, FILE_MAP_ALL_ACCESS, 0, 0, 0) : NULL;
    LPCTSTR pBufPos = (hMapFileRosPos != NULL) ? (LPCTSTR)MapViewOfFile(hMapFileRosPos, FILE_MAP_ALL_ACCESS, 0, 0, 0) : NULL;
    LPCTSTR pBufStatus = (hMapFileStatus != NULL) ? (LPCTSTR)MapViewOfFile(hMapFileStatus, FILE_MAP_ALL_ACCESS, 0, 0, 0) : NULL;
    LPCTSTR pBufCoord = (hMapFileCoord != NULL) ? (LPCTSTR)MapViewOfFile(hMapFileCoord, FILE_MAP_ALL_ACCESS, 0, 0, 0) : NULL;


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


    // Read robot position from shared memory, if not return -999, then zero out
    if (pBufPos != NULL) {
        float* pos_ptr = const_cast<float*>(reinterpret_cast<const float*>(pBufPos));
        for (size_t i = 0; i < 50; ++i) {
            for (size_t j = 0; j < 6; ++j) { shared_get6pos[i][j] = pos_ptr[i * 6 + j]; }
        }
        UnmapViewOfFile(pBufPos);
        CloseHandle(hMapFileRosPos);
    }
    else {}

    // Read status from shared memory
    if (pBufStatus != NULL) {
        unsigned int key;
        char value[252]; // assuming value length is 252 bytes

        // Copy key
        if (memcpy_s(&key, sizeof(unsigned int), pBufStatus, sizeof(unsigned int)) != 0) { *sttlogs << "Error copying Status key from shared memory."; }

        // Copy value
        if (memcpy_s(value, sizeof(value), const_cast<char*>(reinterpret_cast<const char*>(pBufStatus)) + sizeof(unsigned int), 252) != 0) { *sttlogs << "Error copying value from shared memory."; }
        value[251] = '\0'; // Ensure null-termination

        if (key != stt_id)
        {
            stt_id = key;
            *sttlogs << value;
        }
        UnmapViewOfFile(pBufStatus);
        CloseHandle(hMapFileStatus);
    }
        
    // Read 3D coordinates from python, then close
    if (pBufCoord != NULL) {
        unsigned int key;
        unsigned int row_number;
        float* data_ptr;

        if (memcpy_s(&key, sizeof(unsigned int), pBufCoord, sizeof(unsigned int)) != 0){ *sttlogs << "Error copying 3D Coord  key from shared memory."; }

        if (memcpy_s(&row_number, sizeof(unsigned int), const_cast<char*>(reinterpret_cast<const char*>(pBufCoord)) + sizeof(unsigned int), sizeof(unsigned int)) != 0) {
            *sttlogs << "Error copying row_number from shared memory.";}

        if (key != coord_id) {
            coord_id = key;
            data_ptr = const_cast<float*>(reinterpret_cast<const float*>(pBufCoord) + 2); // +2 to skip key and row_number
            // Adjust the size of shared_3Ddata based on row_number
            shared_3Ddata = std::vector<std::vector<float>>(row_number, std::vector<float>(1024, -999.0f));
            for (unsigned int i = 0; i < row_number; ++i) {
                for (unsigned int j = 0; j < 1024; ++j) {
                    shared_3Ddata[i][j] = data_ptr[i * 1024 + j];
                }
            }
            *sttlogs << "[3D] Read 3D coordinates from Shared memory, size: 1024 x " + std::to_string(row_number);
        }
        UnmapViewOfFile(pBufCoord);
        CloseHandle(hMapFileCoord);

    }
    else {shared_3Ddata = std::vector<std::vector<float>>(1, std::vector<float>(1, 0.0f));}

    return true;
}

GLuint zmpdata::matToTexture(const cv::Mat& mat) {
    // Generate a number for our textureID's unique handle
    GLuint textureID;
    glGenTextures(1, &textureID);

    // Bind to our texture handle 
    glBindTexture(GL_TEXTURE_2D, textureID); 

    // Catch silly-mistake texture interpolation method for magnification
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    // Copy image data into the texture
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, mat.cols, mat.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, mat.ptr());

    return textureID;
}
