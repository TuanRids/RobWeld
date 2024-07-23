#pragma once
#include "pch.h"
#include <set>
#include <cstdio>
#include <windows.h>
#include <tlhelp32.h>
#include "statuslogs.h"
#include "Filemgr/RobInitFile.h"


namespace nui
{
    class CMDReader {
    public:
        CMDReader() : sttlogs(nullptr), processHandle(nullptr), readHandle(nullptr), writeHandle(nullptr),robinit(nullptr) {
            sttlogs = &nui::StatusLogs::getInstance();
            robinit = &RobInitFile::getinstance();
            CMDClear();
        }
        ~CMDReader() {}
        void CMDClear() {
            if (processHandle) {
                TerminateProcess(processHandle, 0); // Terminate the process properly
                CloseHandle(processHandle);
            }
            if (readHandle) {
                CloseHandle(readHandle);
            }
        }

        void readCMD() {
            auto currentTime = std::chrono::system_clock::now();
            auto processOutput = getProcessOutput();

            for (const auto& [time, line] : processOutput) {
                if (std::chrono::duration_cast<std::chrono::minutes>(currentTime - time).count() > 5) {
                    continue;
                }

                if (processedTimes.find(time) != processedTimes.end()) {
                    continue;
                }

                processedTimes.insert(time);
                *sttlogs << "\n****************** cmd Logs: \n" + line + "\n******************";
            }
        }
        void restart() {
			initializeProcess();
        }
    private:
        RobInitFile* robinit;
        nui::StatusLogs* sttlogs;
        HANDLE processHandle{};
        HANDLE readHandle{};
        HANDLE writeHandle{};
        std::set<std::chrono::system_clock::time_point> processedTimes;
        std::string pythonPath; 
        std::string scriptPath; 
        std::string workDir;  

        void initializeProcess() {
            robinit->get_settings("pythonPath", pythonPath); // = "C:\\Users\\FSAM\\AppData\\Local\\Programs\\Python\\Python312\\python.exe";
            robinit->get_settings("scriptPath", scriptPath); //= "E:\\Quan\\AutoRoboticInspection-V1\\VIKO_UltraRobot\\src\\Infer_software.py";
            robinit->get_settings("workDir", workDir); // = "E:\\Quan\\AutoRoboticInspection-V1\\VIKO_UltraRobot";
            
            SECURITY_ATTRIBUTES sa;
            sa.nLength = sizeof(SECURITY_ATTRIBUTES);
            sa.bInheritHandle = TRUE;
            sa.lpSecurityDescriptor = NULL;

            if (!CreatePipe(&readHandle, &writeHandle, &sa, 0)) {
                *sttlogs << "Failed to create pipe" ;
                return;
            }

            STARTUPINFO si;
            ZeroMemory(&si, sizeof(STARTUPINFO));
            si.cb = sizeof(STARTUPINFO);
            si.hStdError = writeHandle;
            si.hStdOutput = writeHandle;
            si.dwFlags |= STARTF_USESTDHANDLES | STARTF_USESHOWWINDOW;
            si.wShowWindow = SW_HIDE;

            PROCESS_INFORMATION pi;
            ZeroMemory(&pi, sizeof(PROCESS_INFORMATION));

            
            char orgiDir[MAX_PATH];
            GetCurrentDirectory(MAX_PATH, orgiDir);

            // switch
            SetCurrentDirectory(workDir.c_str());
            std::string command = pythonPath + " " + scriptPath;

            if (!CreateProcess(NULL, const_cast<LPSTR>(command.c_str()), NULL, NULL, TRUE, CREATE_NO_WINDOW, NULL, NULL, &si, &pi)) {
                *sttlogs << "Failed to create process" ;
                CloseHandle(readHandle);
                CloseHandle(writeHandle);
                return;
            }

            processHandle = pi.hProcess;
            CloseHandle(pi.hThread);
            CloseHandle(writeHandle);  // Close the write end of the pipe in the parent process
            SetCurrentDirectory(orgiDir);
        }

        std::vector<std::pair<std::chrono::system_clock::time_point, std::string>> getProcessOutput() {
            std::vector<std::pair<std::chrono::system_clock::time_point, std::string>> result;
            char buffer[1024];
            DWORD bytesRead;
            auto currentTime = std::chrono::system_clock::now();

            while (PeekNamedPipe(readHandle, NULL, 0, NULL, &bytesRead, NULL) && bytesRead > 0) {
                if (ReadFile(readHandle, buffer, sizeof(buffer) - 1, &bytesRead, NULL) && bytesRead > 0) {
                    buffer[bytesRead] = '\0';
                    std::string line(buffer);
                    line.erase(line.find_last_not_of(" \n\r\t") + 1);
                    result.emplace_back(currentTime, line);
                }
            }

            return result;
        }
    };
}