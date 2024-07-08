#pragma once

// Std includes
#include <string>
#include <algorithm>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <memory>
#include "config.hpp"
#include <stdexcept>
#include <chrono>
#include "set"
#include "map"

#include "imgui.h"
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <ImFileBrowser.h>


/// UPDATE UNDO REDO COMMANDS. CURRENTLY ITS TOO COMPLICATED
/// 
/// IMPROVE RENDER QUALITY!!!!
/// 
/// 1. Get original position & direction 


#include <gl/glew.h>
#include <GLFW/glfw3.h>m

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/transform.hpp>

#define BIND_FN(fn) [this](auto&&... args) -> decltype(auto) { return this->fn(std::forward<decltype(args)>(args)...); }
