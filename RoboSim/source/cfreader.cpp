#include "pch.h"
#include "cfreader.h"
std::filesystem::path currentDir = std::filesystem::current_path();

ImVec4 Cfigreader(const std::string& gtheme, const std::string& key, ImVec4 defaultValue)
{
	std::ifstream configFile(currentDir / "config.ini");
	std::string line;
	bool inThemeSection = false;

	while (std::getline(configFile, line))
	{
		if (line.empty() || line[0] == ';') {continue;}

		if (line[0] == '[') {
			inThemeSection = (line.find("[theme][" + gtheme + "]") != std::string::npos);
			continue;
		}
		if (inThemeSection) {
			std::istringstream iss(line);
			std::string k, temp;
			float r, g, b, a;
			if (iss >> k >> temp >> r >> g >> b >> a)
			{
				if (k == key)
				{
					return ImVec4(r, g, b, a);
				}
			}
		}
	}
	return defaultValue;
}



float Cfigreader(const std::string& key, float defaultValue)
{
	std::filesystem::path path = currentDir / "config.ini";
	std::ifstream configFile(path);
	std::string line;
	while (std::getline(configFile, line))
	{
		if (line.empty() || line[0] == '[' || line[0] == ';') { continue; }
		std::istringstream iss(line);
		std::string k,temp;
		std::string value;
		iss >> k >> temp >> value;
		if (k == key)
		{
			return std::stof(value);
		}
	}
	return defaultValue;
} void Cfigreader(const std::string& key, std::string& value)
{
	std::ifstream configFile(currentDir / "config.ini");
	std::string line;
	while (std::getline(configFile, line))
	{
		if (line.empty() || line[0] == '[' || line[0] == ';') { continue; }
		std::istringstream iss(line);
		std::string k , temp;
		std::string defaultValue; iss >> k >> temp >> defaultValue;
		if (k == key)
		{
			value = defaultValue;
			return;
		}
	}
}
std::string Cfigreader(const std::string& key, std::string defaultvalue)
{
	std::ifstream configFile(currentDir / "config.ini");
	std::string line;
	while (std::getline(configFile, line))
	{
		if (line.empty() || line[0] == '[' || line[0] == ';') { continue; }
		std::istringstream iss(line);
		std::string k, temp;
		std::string value; iss >> k >> temp >> value;
		if ( k == key)
		{
			return value;
		}
	}
	return defaultvalue;
}