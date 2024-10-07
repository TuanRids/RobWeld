#pragma once
#include "pch.h"

#include <string>
#include <fstream>
#include <sstream>
float Cfigreader(const std::string& key, float defaultValue);
void Cfigreader(const std::string& key, std::string& value);
std::string Cfigreader(const std::string& key, std::string defaultvalue);
ImVec4 Cfigreader(const std::string& gtheme, const std::string& key, ImVec4 defaultValue);