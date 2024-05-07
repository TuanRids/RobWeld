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

#include "imgui.h"
#include <ImFileBrowser.h>

/// SETUP RELEASE
/// TEST RELEASE
/// Shader Problems
/// BIG PROBLEMS: COOR REWITE 2 cases *****************=> Fixed
/// Check: Rotate original, rotate degree update
/// Resize Problems			*****************=> Fixed - forget to bind framebuffer
/// TEST ROBOT
/// UPDATE DRAW LINES, POINTS
/// ADD DRAW SPHERE, CYLINDER, BOX, CONE, ETC.
/// ADD DRAW TEXT


// GL includes
#include <gl/glew.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/transform.hpp>

#define BIND_FN(fn) [this](auto&&... args) -> decltype(auto) { return this->fn(std::forward<decltype(args)>(args)...); }
