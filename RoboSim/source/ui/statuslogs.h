#ifndef NUI_STATUSLOGS_H
#define NUI_STATUSLOGS_H

#include <deque>
#include <string>
#include <sstream>
#include <ctime>
#include <imgui.h>

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
            statusQueue.push_back(formattedStatus.str());
            while (statusQueue.size() > maxSize) {
                statusQueue.pop_front();
            }
            updateCurrentStatus();
        }

        const std::string& getStatus() const {
            return currentStatus;
        }

        StatusLogs& operator<<(const std::string& status) {
            setStatus(status);
            return *this;
        }

    private:
        StatusLogs() {}
        ~StatusLogs() = default; 

        void updateCurrentStatus() {
            std::ostringstream oss;
            for (const auto& s : statusQueue) {
                oss << s << "\n";
            }
            currentStatus = oss.str();
        }

        std::deque<std::string> statusQueue;
        std::string currentStatus;
        unsigned int idex{0};
        const size_t maxSize = 50; // Maximum number of status messages to keep
    };

}

#endif // NUI_STATUSLOGS_H
