#include "pch.h"
#include "IPCtransfer.h"
#include <stdlib.h>
IPCtransfer::IPCtransfer() : image_texture(0), image_texture_below{ 0 }
{
    sttlogs = &nui::StatusLogs::getInstance();
    history_img = std::make_unique<std::deque<cv::Mat>>();
    robinit = &RobInitFile::getinstance();
    promesh = &nelems::mMesh::getInstance();
    pcl2m = std::make_unique<PclToMesh>();

}

std::vector<std::vector<float>> IPCtransfer::shared_get6pos(std::vector<std::vector<float>>(50, std::vector<float>(6, -999.0f)));
// std::vector<std::vector<float>> IPCtransfer::shared_3Ddata(std::vector<std::vector<float>>(5, std::vector<float>(1024, -999.0f)));
bool IPCtransfer::robot_cnstt = false;
IPCtransfer::~IPCtransfer() {
    if (image_texture != 0) {
        glDeleteTextures(1, &image_texture);
    }
}

void IPCtransfer::send_datatoIPC() {

    const char* ipc_send_mapping = Cfigreader("IPC_SEND_TRIGGER", "Local\\TriggerVision").c_str(); //"Local\\TriggerVision"

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


//void IPCtransfer::trigger_3DCreator()
//{
//    // pcl2m->mainlooping();
//    // check to delete fast LidarObj_medium  LidarObj_fast
//    /*if (promesh->check_selected("LidarObj_medium")) {
//        if (promesh->check_selected("LidarObj_fast")) {
//            promesh->delete_byname("LidarObj_fast");
//        }
//    }*/
//    pcl2m->setter_data(shared_3Ddata);
//    pcl2m->processPointCloud(coord_id);
//    shared_3Ddata = std::vector<std::vector<float>>(5, std::vector<float>(1024, -999.0f));
//
//}

void IPCtransfer::getter_6pos(std::vector<std::vector<float>>& get6pos) {

    std::vector<std::vector<float>> valid_positions;

    for (const auto& pos : shared_get6pos) {
        float difference_from_invalid_value = std::fabs(pos[0] + 999.0f);
        if (difference_from_invalid_value > 5.0f) {
            valid_positions.push_back(pos);
        }
        else {
            break;  // Stop processing if an invalid position is encountered
        }
    }
    shared_get6pos = std::vector<std::vector<float>>(10, std::vector<float>(6, -999.0f));
    if (!valid_positions.empty()) {
        get6pos = valid_positions;
    }
}

void IPCtransfer::trigger_run()
{
    auto checkconnect = [this]()->bool {if (!robot_cnstt) { *sttlogs << "Robot is not Connected."; return false; } else { return true; } };

    sceneview->reset_camera();
    if (checkconnect()) { TriggerToPy["Send1"] = 1; promesh->delete_byname("ScanMesh");    }
}

void IPCtransfer::IPCTransferRender() {
    pcl2m->show3d_data(); // For testing
    receive_data();
    // trigger_3DCreator();
    Display_info();
    send_datatoIPC();
    reset_TriggerToPy();
}

void IPCtransfer::clean_image()
{
    glDeleteTextures(1, &image_texture);
    image_texture = 0;
    glDeleteTextures(1, &image_texture_below);
    image_texture_below = 0;
}

void IPCtransfer::reset_TriggerToPy() {
    static auto start = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() > 500) {  // Changed to 500ms
        for (int i = 1; i <= 9; ++i) {
            TriggerToPy["Send" + std::to_string(i)] = 0;
        }
        start = std::chrono::high_resolution_clock::now();
    }
}

bool IPCtransfer::receive_data() {
    static std::string tempt1 = std::string(Cfigreader("IPC_GET_IMG",               "Local\\ImgAbove"));        const char* dat_Img = tempt1.c_str(); 
    static std::string tempt2 = std::string(Cfigreader("IPC_GET_IMG_BELOW",         "Local\\ImgBelow"));        const char* dat_ImgBelow = tempt2.c_str();
    static std::string tempt3 = std::string(Cfigreader("IPC_GET_POS",               "Local\\Robpos"));          const char* dat_rospos = tempt3.c_str();
    static std::string tempt4 = std::string(Cfigreader("IPC_GET_STATUS",            "Local\\VisionStatus"));    const char* dat_status = tempt4.c_str();
    static std::string tempt5 = std::string(Cfigreader("IPC_GET_COOR3DMESH",        "Local\\Coor3DMesh"));      const char* dat_coord = tempt5.c_str();
    static std::string tempt6 = std::string(Cfigreader("IPC_GET_CHART",             "Local\\ChartInspection")); const char* dat_chart = tempt6.c_str();


    // Open the shared memory objects
    HANDLE hMapFileImg = OpenFileMapping(FILE_MAP_ALL_ACCESS, FALSE, TEXT(dat_Img));
    HANDLE hMapFileImgBelow = OpenFileMapping(FILE_MAP_ALL_ACCESS, FALSE, TEXT(dat_ImgBelow));
    HANDLE hMapFileRosPos = OpenFileMapping(FILE_MAP_ALL_ACCESS, FALSE, TEXT(dat_rospos));
    HANDLE hMapFileStatus = OpenFileMapping(FILE_MAP_ALL_ACCESS, FALSE, TEXT(dat_status));
    HANDLE hMapFileCoord = OpenFileMapping(FILE_MAP_ALL_ACCESS, FALSE, TEXT(dat_coord));
    HANDLE hMapFileChart = OpenFileMapping(FILE_MAP_ALL_ACCESS, FALSE, TEXT(dat_chart));



    // Map the shared memory objects
    LPCTSTR pBufImg = (hMapFileImg != NULL) ? (LPCTSTR)MapViewOfFile(hMapFileImg, FILE_MAP_ALL_ACCESS, 0, 0, 0) : NULL;
    LPCTSTR pBufImgBelow = (hMapFileImgBelow != NULL) ? (LPCTSTR)MapViewOfFile(hMapFileImgBelow, FILE_MAP_ALL_ACCESS, 0, 0, 0) : NULL;
    LPCTSTR pBufPos = (hMapFileRosPos != NULL) ? (LPCTSTR)MapViewOfFile(hMapFileRosPos, FILE_MAP_ALL_ACCESS, 0, 0, 0) : NULL;
    LPCTSTR pBufStatus = (hMapFileStatus != NULL) ? (LPCTSTR)MapViewOfFile(hMapFileStatus, FILE_MAP_ALL_ACCESS, 0, 0, 0) : NULL;
    LPCTSTR pBufCoord = (hMapFileCoord != NULL) ? (LPCTSTR)MapViewOfFile(hMapFileCoord, FILE_MAP_ALL_ACCESS, 0, 0, 0) : NULL;
    LPCTSTR pBufChart = (hMapFileChart != NULL) ? (LPCTSTR)MapViewOfFile(hMapFileChart, FILE_MAP_ALL_ACCESS, 0, 0, 0) : NULL;


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
        unsigned int key;
        if (memcpy_s(&key, sizeof(unsigned int), pBufImgBelow, sizeof(unsigned int)) != 0) { *sttlogs << "Error copying IMG Below key from shared memory."; }
        std::vector<uchar> img_below_vec(reinterpret_cast<const uchar*>(pBufImgBelow) + sizeof(unsigned int), reinterpret_cast<const uchar*>(pBufImgBelow) + 640 * 480 * 3);
        if (key != img_below_id)
        {
            img_below_id = key;
            img_below = cv::imdecode(img_below_vec, cv::IMREAD_COLOR);
        }
        UnmapViewOfFile(pBufImgBelow);
        CloseHandle(hMapFileImgBelow);
    }
    else { img_below.release(); }


    // Read robot position from shared memory, if not return -999, then zero out
    if (pBufPos != NULL) {
        unsigned int ros_gkey, ros_gnumber;
        float* ros_gdata;
        static unsigned int ros_check_key{ 0 };
        if (memcpy_s(&ros_gkey, sizeof(unsigned int), pBufPos, sizeof(unsigned int)) != 0) { *sttlogs << "Error copying 3D RobPos key from shared memory.";  }

        if (memcpy_s(&ros_gnumber, sizeof(unsigned int), const_cast<char*>(reinterpret_cast<const char*>(pBufPos)) + sizeof(unsigned int), sizeof(unsigned int)) != 0) {
            *sttlogs << "Error copying rospos Number from shared memory."; 
        }

        if (ros_gkey != ros_check_key) {
            ros_check_key = ros_gkey;
            ros_gdata = const_cast<float*>(reinterpret_cast<const float*>(pBufPos) + 2); // +2 to skip key and row_number
            // Adjust the size of shared_3Ddata based on row_number
            shared_get6pos = std::vector<std::vector<float>>(ros_gnumber, std::vector<float>(6, -999.0f));
            for (unsigned int i = 0; i < ros_gnumber; ++i) {
                for (unsigned int j = 0; j < 6; ++j) {
                    shared_get6pos[i][j] = ros_gdata[i * 6 + j];
                }
            }
            *sttlogs << "Read Robot Position from Shared memory: " + std::to_string(ros_gnumber);
        }

        UnmapViewOfFile(pBufPos);
        CloseHandle(hMapFileRosPos);
    }
    else {}

    // Read status from shared memory
    if (pBufStatus != NULL) {
        unsigned int key;
        char value[252];
        if (memcpy_s(&key, sizeof(unsigned int), pBufStatus, sizeof(unsigned int)) != 0) { *sttlogs << "Error copying Status key from shared memory."; }

        if (memcpy_s(value, sizeof(value), const_cast<char*>(reinterpret_cast<const char*>(pBufStatus)) + sizeof(unsigned int), 252) != 0) { *sttlogs << "Error copying value from shared memory."; }
        value[251] = '\0';

        if (key != stt_id)
        {
            stt_id = key;
            *sttlogs << value;
        }
        UnmapViewOfFile(pBufStatus);
        CloseHandle(hMapFileStatus);
    }

    //// Read 3D coordinates from python, then close
    //if (pBufCoord != NULL) {
    //    unsigned int key;
    //    unsigned int row_number;
    //    float* data_ptr;

    //    if (memcpy_s(&key, sizeof(unsigned int), pBufCoord, sizeof(unsigned int)) != 0) { *sttlogs << "Error copying 3D Coord  key from shared memory."; }

    //    if (memcpy_s(&row_number, sizeof(unsigned int), const_cast<char*>(reinterpret_cast<const char*>(pBufCoord)) + sizeof(unsigned int), sizeof(unsigned int)) != 0) {
    //        *sttlogs << "Error copying row_number from shared memory.";
    //    }

    //    if (key != coord_id) {
    //        coord_id = key;
    //        data_ptr = const_cast<float*>(reinterpret_cast<const float*>(pBufCoord) + 2); // +2 to skip key and row_number
    //        // Adjust the size of shared_3Ddata based on row_number
    //        shared_3Ddata = std::vector<std::vector<float>>(row_number+20, std::vector<float>(1024, -999.0f));
    //        for (unsigned int i = 0; i < row_number; ++i) {
    //            for (unsigned int j = 0; j < 1024; ++j) {
    //                shared_3Ddata[i][j] = data_ptr[i * 1024 + j];
    //            }
    //        }
    //        *sttlogs << "[3D] Read 3D coordinates from Shared memory, size: 1024 x " + std::to_string(row_number);
    //    }
    //    UnmapViewOfFile(pBufCoord);
    //    CloseHandle(hMapFileCoord);

    //}
    //else { shared_3Ddata = std::vector<std::vector<float>>(1, std::vector<float>(1, 0.0f)); }

    // Read chart from python, then close
    if (pBufChart != NULL) {
        unsigned int key;
        float* data_ptr;

        if (memcpy_s(&key, sizeof(unsigned int), pBufChart, sizeof(unsigned int)) != 0) { *sttlogs << "Error copying 3D Chart key from shared memory."; }

        if (key != chart_id) {
            static std::map<std::string, float> IPCNewData{};
            chart_id = key; std::string ipcChartContent; float ipcChartValue;
            data_ptr = const_cast<float*>(reinterpret_cast<const float*>(pBufChart) + 1); // +1 to skip key
            // Adjust the size of shared_3Ddata based on row_number
            for (unsigned int i = 0; i < 8; ++i) {
                ipcChartValue = data_ptr[i * 65];
                ipcChartContent = std::string(reinterpret_cast<const char*>(data_ptr + (i * 65) + 1), 256); // Adjusted from i * 256 + 1
                // Check if ipcChartContent is empty or null
                if (ipcChartValue == 0.0f && ipcChartContent[0] == '\0') {
                    break; // Exit loop if no more valid data
                }
                ipcChartContent.erase(std::find(ipcChartContent.begin(), ipcChartContent.end(), '\0'), ipcChartContent.end()); // Remove trailing null characters
                IPCNewData[ipcChartContent] = ipcChartValue;
            }
            nui::FrameManage::update_chart_dat(IPCNewData);
        }
        UnmapViewOfFile(pBufChart);
        CloseHandle(hMapFileChart);

    }

    return true;
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

void IPCtransfer::Display_info()
{
    ImGui::Begin("Camera Vision (R)", nullptr,
       // ImGuiWindowFlags_NoDocking |    // Cannot be docked
        ImGuiWindowFlags_NoBackground | // Do not display background
        ImGuiWindowFlags_NoNavFocus);

    displayControlButtons();
    ImGui::BeginGroup();
    displayMainImage();
    displaySecondaryImage();
    ImGui::EndGroup(); ImGui::SameLine();
    if (UnImgFrameTrigger) { displayLaserView(); }
    ImGui::End();
}

GLuint IPCtransfer::matToTexture(const cv::Mat& mat) {
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

void IPCtransfer::displayControlButtons() {
    if (!sceneview) { sceneview = &nui::SceneView::getInstance(); }    
    auto checkconnect = [this]()->bool {if (!robot_cnstt) { *sttlogs << "Robot is not Connected."; return false; } else { return true; } };

    ImVec4 runcolor = ImVec4(0.5f, 0.0f, 0.2f, 1.0f);
    ImGui::PushStyleColor(ImGuiCol_Button, runcolor);
    if (ImGui::Button("F1 Run")) 
    { 
        promesh->delete_byname("ScanMesh"); sceneview->reset_camera();
        if (checkconnect()) { TriggerToPy["Send1"] = 1; } 
    }
    ImGui::SameLine();
    ImGui::PopStyleColor(1);

    if (ImGui::Button("F2 Re. 3D") || tgre3d == 1) { pcl2m->reset_unqueIDpath(); tgre3d = 0; }ImGui::SameLine();
    if (ImGui::Button("F3 Details")|| tgdetails == 1) {UnImgFrameTrigger = true; tgdetails = 0;}ImGui::SameLine();
    if (ImGui::Button("F4 Insp.") || tgsample == 1) { if (checkconnect()) { TriggerToPy["Send2"] = 1; }; tgsample = 0;    }
    // if (ImGui::Button("R Right")) { if (checkconnect()) { TriggerToPy["Send3"] = 1; } } ImGui::SameLine();
    // if (ImGui::Button("R Insp")) { if (checkconnect()) { TriggerToPy["Send4"] = 1; } }ImGui::SameLine();
}

void IPCtransfer::displayMainImage() {
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
            image_size.x = winsize.y * scale_ratio * 0.95;
            image_size.y = winsize.y * 0.95;
        }
        else {
            image_size.x = winsize.x * 0.95;
            image_size.y = winsize.x / scale_ratio * 0.95;
        }
        ImGui::Image((void*)(intptr_t)image_texture, image_size);
    }
}

void IPCtransfer::displaySecondaryImage() {
    if (!img_below.empty()) {
        if (image_texture_below != 0) {
            glDeleteTextures(1, &image_texture_below);
        }
        image_texture_below = matToTexture(img_below);
        static unsigned int tempt_needtodel = 0;
        if (tempt_needtodel != img_below_id)
        {
            tempt_needtodel = img_below_id;
            history_img->push_back(img_below);
        }
        if (history_img->size() > 10) {
            history_img->pop_front();
        }
    }
    if (image_texture_below != 0) {
        ImVec2 winsize = ImGui::GetWindowSize();  // get the size of the window
        float scale_ratio = (float)img_below.cols / (float)img_below.rows;

        ImVec2 image_size;
        if (winsize.x / scale_ratio > winsize.y) {
            image_size.x = winsize.y * scale_ratio * 0.95;
            image_size.y = winsize.y * 0.95;
        }
        else {
            image_size.x = winsize.x * 0.95;
            image_size.y = winsize.x / scale_ratio * 0.95;
        }
        ImGui::Image((void*)(intptr_t)image_texture_below, image_size);

    }
}

void IPCtransfer::displayLaserView() {
    ImGui::SetWindowPos(ImVec2(20, 20));

    static std::unique_ptr<bool> lock_frame = std::make_unique<bool>(); ;
    if (lock_frame == nullptr) { robinit->get_settings("lock_frame", *lock_frame); }

    if (!lock_frame)
    {
        ImGui::Begin("Details Vision", &UnImgFrameTrigger, ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoMove | 
            ImGuiWindowFlags_AlwaysHorizontalScrollbar | ImGuiWindowFlags_AlwaysVerticalScrollbar);
    }
    else {
        ImGui::Begin("Details Vision", &UnImgFrameTrigger, ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_AlwaysHorizontalScrollbar | ImGuiWindowFlags_AlwaysVerticalScrollbar);
    }
    //adjustment
    ImGui::BeginGroup();
    static int Brightness = 0, Contrast = 0, Sharpness = 0;
    static float Gamma = 0.0f, Saturation = 0.0f;
    static float zoomFactor = 1.0f; // Initialize zoom factor

    ImGui::SetNextItemWidth(80);
    ImGui::SliderInt("Brightness", &Brightness, -100, 100); ImGui::SetNextItemWidth(80);
    ImGui::SliderInt("Contrast", &Contrast, -100, 100); ImGui::SetNextItemWidth(80);
    ImGui::SliderInt("Sharpness", &Sharpness, 0, 100); ImGui::SetNextItemWidth(80);
    ImGui::SliderFloat("Gamma", &Gamma, 0.0f, 3.0f); ImGui::SetNextItemWidth(80);
    ImGui::SliderFloat("Saturation", &Saturation, 0.0f, 3.0f); ImGui::SetNextItemWidth(80);
    ImGui::SliderFloat("ZoomFactor", &zoomFactor, 0.1f, 10.0f);
    for (auto it = history_img->begin(); it != history_img->end(); ++it)
    {
        std::string button_label = "" + std::to_string(it - history_img->begin());
        GLuint thumbnail_texture = matToTexture(*it);
        ImGui::Image((void*)(intptr_t)thumbnail_texture, ImVec2(48, 48)); ImGui::SameLine(); //ImGui::SetNextItemWidth(10);
        if (ImGui::Button(button_label.c_str(), ImVec2(13, 48))) { img_selected = *it; }
        if ((it - history_img->begin()) % 2 == 0) { ImGui::SameLine(); }
    }
    ImGui::EndGroup();
    ImGui::SameLine();

    // ImGui::BeginGroup();
    ImGui::BeginChild("Image", ImVec2(0, 0), true, ImGuiWindowFlags_AlwaysVerticalScrollbar);
    ImVec2 win_size = ImGui::GetWindowSize();
    float scale_ratio = (float)img_selected.cols / (float)img_selected.rows;
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
    img_selected.convertTo(adjusted_img, -1, 1 + Contrast / 100.0, Brightness);

   
    if (Gamma > 0 && history_img->size() > 0) { applyGammaCorrection(adjusted_img, adjusted_img, Gamma); }
    if (Saturation > 0 && history_img->size() > 0) { applySaturation(adjusted_img, adjusted_img, Saturation); }
    if (Sharpness && history_img->size() > 0) { applySharpness(adjusted_img, adjusted_img, Sharpness / 100.0); }

    static GLuint adjusted_texture = 0;
    if (adjusted_texture != 0) {
        glDeleteTextures(1, &adjusted_texture);
    }
    adjusted_texture = matToTexture(adjusted_img);

    ImGui::Image((void*)(intptr_t)adjusted_texture, img_size);

    ImGuiIO& io = ImGui::GetIO();
    // Handle zooming
    if (io.MouseWheel != 0) {
        zoomFactor += io.MouseWheel * 0.1f; // Adjust zoom factor with mouse wheel
        if (zoomFactor < 0.1f) zoomFactor = 0.1f; // Prevent zooming out too much
    }

    ImGui::EndChild();

    // close if clicked outside
    
    ImVec2 window_pos = ImGui::GetWindowPos();
    ImVec2 window_size = ImGui::GetWindowSize();
    ImVec2 mouse_pos = io.MousePos;
    if (ImGui::IsMouseClicked(0) && !lock_frame) {
        if (mouse_pos.x < window_pos.x || mouse_pos.x > window_pos.x + window_size.x ||
            mouse_pos.y < window_pos.y || mouse_pos.y > window_pos.y + window_size.y) {
            UnImgFrameTrigger = false;
        }
    }
    ImGui::End();
}
