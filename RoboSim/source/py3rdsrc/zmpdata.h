#pragma once
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
#include "pch.h"  // config in pch
#include <mutex>


class zmpdata {
public:
    zmpdata();
    ~zmpdata();
    // Show Vision Data
    void render();
	// Send data to Shared_Memory
    void render_send();

private:
    GLuint image_texture;
    std::map<std::string, bool> TriggerToPy;
    std::mutex mtx;  // Add mutex for synchronization


    // Reset TriggerToPy value to 0 after 300ms
    void reset_TriggerToPy();
    // Receive data from Shared_Memory
    bool receive_data(cv::Mat& img, int& frame_count, float& rotation_speed);
    // Send data to Shared_Memory
    GLuint matToTexture(const cv::Mat& mat, GLenum minFilter, GLenum magFilter, GLenum wrapFilter);
};
