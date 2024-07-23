#ifndef NUI_STATUSLOGS_H
#define NUI_STATUSLOGS_H

#include <deque>
#include <vector>
#include <string>
#include <sstream>
#include <ctime>
#include <imgui.h>
#include <mutex>

namespace nui {
    class StatusLogs {
    public:
        static StatusLogs& getInstance() {
            static StatusLogs instance;
            return instance;
        }

        StatusLogs(const StatusLogs&) = delete;
        StatusLogs& operator=(const StatusLogs&) = delete;

        void setStatus(const std::string& status) {
            std::lock_guard<std::mutex> lock(sttmutex);
            // Get current time
            std::time_t now = std::time(nullptr);
            std::tm localTime;
            localtime_s(&localTime, &now);

            // Format time as [hour-min-sec]
            char timeBuffer[10];
            std::strftime(timeBuffer, sizeof(timeBuffer), "%H-%M-%S", &localTime);

            // Format the status message with the timestamp
            std::ostringstream formattedStatus;
            formattedStatus << "[" << timeBuffer << "]: " << status;

            // Add formatted status to the deque
            statusQueue.push_front(formattedStatus.str());
            while (statusQueue.size() > maxSize) {
                statusQueue.pop_back();
            }
            updateCurrentStatus();
        }

        const std::vector<std::string>& getStatus() const {
            std::lock_guard<std::mutex> lock(sttmutex);
            return currentStatus;
        }

        StatusLogs& operator<<(const std::string& status) {
            setStatus(status);
            return *this;
        }

    private:
        StatusLogs() {}
        ~StatusLogs() = default;
        mutable std::mutex sttmutex;

        void updateCurrentStatus() {
            currentStatus.clear();
            for (const auto& s : statusQueue) {
                currentStatus.push_back(s);
            }
        }

        std::deque<std::string> statusQueue;
        std::vector<std::string> currentStatus;
        const size_t maxSize = 50; // Maximum number of status messages to keep
    };

}

#endif // NUI_STATUSLOGS_H
