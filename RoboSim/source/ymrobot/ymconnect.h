#pragma once
#include "pch.h"
#include "YMConnect/YMConnect.h"
#include "imgui.h"
#include "elems/mesh.h"
#include <iomanip> 
#include "ui/FrameManage.h"
#include <vector>
#include <sstream>
#include "ui/statuslogs.h"
#include "IPCTransfer/IPCtransfer.h"
#include "Filemgr/RobInitFile.h"
#include "mutex"


struct UIState {
    unsigned int coumove{ 3 };
    bool START_Flag{ false };
    bool lineshpath{ true };
    MotionAccelDecel accdec{ 20,20 };
    // 1: Linear, 2: Circular, 3: Joint 4: Mid-Cir
    std::vector<int> movTypes = std::vector<int>(50, 0);
    float spdlinear{};
    float spdjoint{};
    std::vector<std::vector<float>> rbpos{ std::vector<std::vector<float>>(3, std::vector<float>(6, 0.0f)) };
    std::unique_ptr<StatusInfo> tpstatus = std::make_unique<StatusInfo>();
    std::unique_ptr<BaseAxisPositionVariableData> b1PositionData = std::make_unique<BaseAxisPositionVariableData>();
    std::unique_ptr<PositionData> b1crpos = std::make_unique<PositionData>();
    std::unique_ptr<PositionData> b1workpos = std::make_unique<PositionData>();

    // other trigger
    bool SharedMemoryFlag = true;
};

namespace nymrobot {

    class ymconnect {
    private:
        char ip_address[64] = "192.168.10.102";
        bool autocn_flag = true;

        mutable std::mutex ymmutex;
        static UIState ui_state;
        StatusInfo status{};
        MotomanController* controller;
        UINT32 restime = 10;
        bool VisualizeFlag = false;
        std::vector<std::vector<float>> limitangle{ 6, {-360,360} };

        static std::vector<std::vector<float>> get6pos; 
        std::vector<std::shared_ptr<nelems::oMesh>> base;

        std::array<float,6> angle;
        nelems::mMesh* proMeshRb = nullptr;
        std::stringstream resultmsg;
        nui::StatusLogs* sttlogs;
        IPCtransfer* shmdata;
        RobInitFile* robinit;
        static std::string connect_content;
        ymconnect() : controller(nullptr), angle{}, sttlogs(nullptr), shmdata(nullptr) {
            robinit = &RobInitFile::getinstance(); 
            sttlogs = &nui::StatusLogs::getInstance();
            shmdata = &IPCtransfer::getInstance();
            for (int i{ 0 }; i < 7; i++) { base.push_back(nullptr); }
            
        }
        ~ymconnect();
        // ROBOT CONTROL & RENDER 
        void Robot_Controls_table();
        void rotateJoint(size_t jointIndex, float& ang, float& pre, const float tolerance,
            std::vector<std::shared_ptr<nelems::oMesh>>& base,
            float diffX, float diffY, float diffZ);
        // Limangle for Robot
        // Move Robot
        void move_robot();
		// Setup UI
        void setup_MOVE_ui(UIState& ui_state);
        // Read Robot
        void read_robot();
        // Connect to robot
        void connect_robot();
		// Disconnect from robot
        void disconnect_robot(bool showmsg);
        // Render the Robot & UI
        bool check_connect() { if (!controller) { return false; } else { return true; } }



        void start_reading();
        void stop_reading();
        std::thread read_thread;
        std::atomic<bool> running{ false };
        std::mutex data_mutex;

    public:
        static ymconnect& getInstance() { static ymconnect instance; return instance; }
        // setter for get6pos from shared memory. Only take when != -1.
        void setter_get6pos(const std::vector<std::vector<float>>& get6pos_) { get6pos = get6pos_; }
        void render();

    };

}
