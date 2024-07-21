
#ifndef ROBINITFILE_H
#define ROBINITFILE_H

#include "pch.h"
#pragma warning( push )
#pragma warning( disable : 26819) //3rd party library
#include "nlohmann/json.hpp"
#pragma warning( pop )

#include "ui/statuslogs.h"
#include <locale>

using json = nlohmann::json;
class RobInitFile {
public:
    static RobInitFile& getinstance() {
        static RobInitFile instance; return instance;
    }    
    bool get_settings(const std::string& key, std::string& value) {
        auto it = Init_settings.find(key);
        if (it != Init_settings.end()) {
            value = it->second;
            return true;
        }
        *sttlogs << "Key not found: " << key;
        return false;
    }
    std::string get_settings(const std::string& key) {
        std::string value;
        auto it = Init_settings.find(key);
        if (it != Init_settings.end()) {
            value = it->second;
            return value;
        }
        *sttlogs << "Key not found: " << key;
    }
    void update_settings(const std::string& key, const std::string& value) { Init_settings[key] = value;}
    void SaveInit_encode() {
        std::ofstream outputFile("robosim_settings.txt");
        if (!outputFile.is_open()) {
            *sttlogs << "Failed to open file for writing: robosim_settings.txt";
            return; // Exit if file cannot be opened
        }

        bool writeError = false;
        for (const auto& kv : Init_settings) {
            *sttlogs << kv.first + " : " + kv.second; //

            // Check if the value contains newlines or other problematic characters
            if (kv.second.find('\n') != std::string::npos) {
                *sttlogs << "Error: Newline character in value for key " << kv.first;
                writeError = true;
                continue; // Skip this key-value pair or handle it differently
            }

            outputFile << kv.first << " : " << kv.second << "\n";
            if (outputFile.fail()) {
                *sttlogs << "Failed to write data for key " << kv.first;
                writeError = true;
            }
        }

        if (!writeError) {
            *sttlogs << "All configuration settings saved successfully.";
        }

        outputFile.close(); // Always close the file
    }


    
private:
    nui::StatusLogs* sttlogs;
    std::map<std::string, std::string> Init_settings;

    
    void ReadIniFile_decode() {
        std::ifstream inputFile("robosim_settings.txt");
        if (!inputFile.is_open()) {
            *sttlogs << "Failed to open file for reading: robosim_settings.txt";
            return;
        }

        std::string line;
        while (getline(inputFile, line)) {
            size_t delimiterPos = line.find(' : ');
            if (delimiterPos != std::string::npos) {
                std::string key = line.substr(0, delimiterPos);
                std::string value = line.substr(delimiterPos + 3);
                Init_settings[key] = value;
            }
        }

        inputFile.close();
        *sttlogs << "Configuration loaded from text format.";
    }


    // Singleton
    RobInitFile() {
        sttlogs = &nui::StatusLogs::getInstance();
        ReadIniFile_decode(); // Only read once here, no need to check again after this call
    }

};
#endif