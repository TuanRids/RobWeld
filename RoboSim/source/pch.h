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

/// Shader problems: right view - behind objects cover the front objects
/// TEST ROBOT										=> 
/// UPDATE DRAW LINES, POINTS						=> 
/// ADD DRAW SPHERE, CYLINDER, BOX, CONE, ETC.		=> 
/// ADD DRAW TEXT									=> 
/// 
/// robot table position: x,y,z,rx,ry,rz + 6 5 4 (3 2 1 expand)
/// Keyframe: with a,v between keys
/// 
// GL includes

#include <gl/glew.h>
#include <GLFW/glfw3.h>m

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/transform.hpp>

#define BIND_FN(fn) [this](auto&&... args) -> decltype(auto) { return this->fn(std::forward<decltype(args)>(args)...); }
