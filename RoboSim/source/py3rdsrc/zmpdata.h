#pragma once
#include "pch.h"  // config in pch

#include <Windows.h>
#include <opencv2/opencv.hpp>

#include <stdexcept>
#include <chrono>
#include <map>
#include "ui/statuslogs.h"
#include "ui/FrameManage.h"
#include "mesh_import/pcltomesh.h"
class zmpdata {
public:
    zmpdata();
    ~zmpdata();
    void render();
    void send_datatoIPC();
    void Display_info();
    void trigger_3DCreator();

    void getter_6pos(std::vector<std::vector<float>>& get6pos);
private:
    bool UnImgFrameTrigger = false;
    
    GLuint image_texture;
    GLuint image_texture_below;
    cv::Mat img;
    cv::Mat img_below;
    std::map<std::string, bool> TriggerToPy;
    
    bool SharedMemoryTrigger = false;
    static std::vector<std::vector<float>> shared_get6pos;
    static std::vector<std::vector<float>> shared_3Ddata;

    unsigned int stt_id = 999, coord_id = 999;
    nui::StatusLogs* sttlogs;

    void clean_image();
    void reset_TriggerToPy();
    bool receive_data();
    GLuint matToTexture(const cv::Mat& mat);
};