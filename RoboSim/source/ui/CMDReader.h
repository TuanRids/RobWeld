#pragma once
#include "pch.h"
#include <set>
#include <cstdio>
#include <windows.h>
#include <tlhelp32.h>
#include "statuslogs.h"



namespace nui
{
    class CMDReader {
    public:
        CMDReader() : sttlogs(nullptr), processHandle(nullptr), readHandle(nullptr), writeHandle(nullptr) {
            sttlogs = &nui::StatusLogs::getInstance();
            initializeProcess();
        }
        ~CMDReader() {
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
                *sttlogs << "#CMD LOGS for Dev: " + line;
            }
        }
        void restart() {
            if (processHandle) {
                TerminateProcess(processHandle, 0); // Terminate the process properly
                CloseHandle(processHandle);
            }
            if (readHandle) {
                CloseHandle(readHandle);
            }

			initializeProcess();
        }
    private:
        nui::StatusLogs* sttlogs;
        HANDLE processHandle;
        HANDLE readHandle;
        HANDLE writeHandle;
        std::set<std::chrono::system_clock::time_point> processedTimes;

        void initializeProcess() {
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

            std::string pythonPath = "C:\\Users\\ngdtu\\AppData\\Local\\Programs\\Python\\Python312\\python.exe";
            std::string scriptPath = "C:\\Users\\ngdtu\\source\\python\\zeromp\\test_cmdreader.py";
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
        }

        std::vector<std::pair<std::chrono::system_clock::time_point, std::string>> getProcessOutput() {
            std::vector<std::pair<std::chrono::system_clock::time_point, std::string>> result;
            char buffer[512];
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