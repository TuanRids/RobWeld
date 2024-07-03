#pragma once
#include "pch.h"  // config in pch

#include <Windows.h>
#include <opencv2/opencv.hpp>

#include <stdexcept>
#include <chrono>
#include <map>
#include "ui/statuslogs.h"
#include "ui/FrameManage.h"

class zmpdata {
public:
    zmpdata();
    ~zmpdata();
    void render();
    void send_datatoIPC();
    void Display_info();

    void getter_6pos(std::vector<std::vector<float>>& get6pos);
private:
    GLuint image_texture;
    GLuint image_texture_below;
    cv::Mat img;
    cv::Mat img_below;
    std::map<std::string, bool> TriggerToPy;
    
    bool SharedMemoryTrigger = false;
    static std::vector<std::vector<float>> shared_get6pos;

    unsigned int stt_id = 999;
    std::unique_ptr<nui::StatusLogs> sttlogs;

    void clean_image();
    void reset_TriggerToPy();
    bool receive_data(cv::Mat& img, cv::Mat& img_below, int& frame_count, float& rotation_speed);
    GLuint matToTexture(const cv::Mat& mat, GLenum minFilter, GLenum magFilter, GLenum wrapFilter);
};