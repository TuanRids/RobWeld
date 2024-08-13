#pragma once
#include "pch.h"
#include <cstdio>
#include <windows.h>
#include <tlhelp32.h>
#include "statuslogs.h"
#include "Filemgr/RobInitFile.h"
#include "exception"

namespace nui
{
    class CMDReader {
    public:
        CMDReader() : sttlogs(nullptr), processHandle(nullptr), readHandle(nullptr), writeHandle(nullptr),robinit(nullptr) {
            sttlogs = &nui::StatusLogs::getInstance();
            robinit = &RobInitFile::getinstance();
            CMDClear();
        }
        ~CMDReader() {
            if (processHandle) {
                CloseHandle(processHandle);
            }
            if (readHandle) {
                CloseHandle(readHandle);
            }
            if (writeHandle) {
                CloseHandle(writeHandle);
            }
        }
        void CMDClear() {
            PROCESSENTRY32 processEntry;
            processEntry.dwSize = sizeof(PROCESSENTRY32);
            HANDLE snapshot = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);

            if (snapshot == INVALID_HANDLE_VALUE) {
                std::cerr << "Failed to create snapshot" << std::endl;
                return;
            }

            if (Process32First(snapshot, &processEntry)) {
                do {
                    if (_stricmp(processEntry.szExeFile, "python.exe") == 0) {
                        HANDLE processHandle = OpenProcess(PROCESS_TERMINATE, FALSE, processEntry.th32ProcessID);
                        if (processHandle != nullptr) {
                            TerminateProcess(processHandle, 0); // Terminate the process properly
                            CloseHandle(processHandle);
                        }
                    }
                } while (Process32Next(snapshot, &processEntry));
            }

            CloseHandle(snapshot);
        }


        void readCMD() {
            auto currentTime = std::chrono::system_clock::now();
            auto processOutput = getProcessOutput();

            for (const auto& [time, line] : processOutput) {
                if (bufferLines.find(line) != bufferLines.end()) {
                    continue;
                }

                processedTimes.insert(time);
                *sttlogs << "cmd Logs: \n" + line + "\n";
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
        std::set<std::string> bufferLines;
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
            std::string command;
            SetCurrentDirectory(workDir.c_str());
            try
            {
                command = pythonPath + " " + scriptPath;
            }
            catch (std::exception& e) 
            { *sttlogs << "ERROR: " + *(e.what()); return; }

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
            char buffer[1024*2];
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