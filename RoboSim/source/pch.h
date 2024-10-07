#pragma once

// Std includes
#include <string>
#include <algorithm>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include <memory>
// #include "config.hpp"
#include <stdexcept>
#include <chrono>
#include "set"
#include "map"
#include <Windows.h>
#include <stdexcept>


#include "imgui.h"
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <imgui_impl_win32.h>
#include <imgui_impl_dx11.h>
#include <ImFileBrowser.h>

#pragma warning( push )
#pragma warning( disable : 26819) //3rd party library
#include <opencv2/opencv.hpp>
#pragma warning( pop )

#include <gl/glew.h>
#include <GLFW/glfw3.h>m

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/transform.hpp>

#define BIND_FN(fn) [this](auto&&... args) -> decltype(auto) { return this->fn(std::forward<decltype(args)>(args)...); }
