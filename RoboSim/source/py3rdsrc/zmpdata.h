#pragma once
#include "pch.h"  // config in pch

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <stdexcept>
#include <chrono>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <map>
#include <Windows.h>
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

    void clean_image();
    void reset_TriggerToPy();
    bool receive_data(cv::Mat& img, cv::Mat& img_below, int& frame_count, float& rotation_speed);
    GLuint matToTexture(const cv::Mat& mat, GLenum minFilter, GLenum magFilter, GLenum wrapFilter);
};