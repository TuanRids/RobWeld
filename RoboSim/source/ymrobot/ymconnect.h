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
#include "py3rdsrc/zmpdata.h"

struct UIState {
    unsigned int coumove{ 3 };
    bool START_Flag{ false };
    bool lineshpath{ true };
    // 1: Linear, 2: Circular, 3: Joint 4: Mid-Cir
    std::vector<int> movTypes = std::vector<int>(50, 0);
    float spdlinear{};
    float spdjoint{};
    std::vector<std::vector<float>> rbpos{ std::vector<std::vector<float>>(3, std::vector<float>(6, 0.0f)) };
    std::unique_ptr<StatusInfo> tpstatus = std::make_unique<StatusInfo>();
    std::unique_ptr<BaseAxisPositionVariableData> b1PositionData = std::make_unique<BaseAxisPositionVariableData>();
    std::unique_ptr<PositionData> b1origi = std::make_unique<PositionData>();
    std::unique_ptr<PositionData> b1crpos = std::make_unique<PositionData>();

    // other trigger
    bool SharedMemoryFlag = true;
};

namespace nymrobot {

    class ymconnect {
    private:
        static UIState ui_state;
        StatusInfo status;
        MotomanController* controller;
        UINT32 restime = 10;
        bool switchVisualizeMode = false;
        std::vector<std::vector<float>> limitangle;

        static std::vector<std::vector<float>> get6pos; // Get 6 positions from shared memory

        std::array<float,6> angle;
        nelems::mMesh* proMeshRb = nullptr;
        std::stringstream resultmsg;
        nui::StatusLogs* sttlogs;
        std::unique_ptr<zmpdata> shmdata;

    public:
        ymconnect() : controller(nullptr), angle{}, sttlogs(nullptr), shmdata(nullptr) {
            YMConnect::OpenConnection("192.168.0.0", status, restime);
            sttlogs = &nui::StatusLogs::getInstance();
            shmdata = std::make_unique<zmpdata>();
        }
        ~ymconnect();
        // setter for get6pos from shared memory. Only take when != -1. zmpdata
        void setter_get6pos(const std::vector<std::vector<float>>& get6pos_) { get6pos = get6pos_; }

        // Connect to robot
        void connect_robot();
		// Disconnect from robot
        void disconnect_robot(bool showmsg);
        // Render the Robot & UI
        void render();
        // Limangle for Robot
        void set_limitangle(const std::vector<std::vector<float>>& getlim) { limitangle = getlim; }
        // Move Robot
        void move_robot();
		// Setup UI
        void setup_MOVE_ui(UIState& ui_state);
        // Read Robot
        void read_robot();
        // Get SwitchVisualize
        bool getSwitchVisualize() const { return switchVisualizeMode; }
        // Set SwitchVisualize
        void setSwitchVisualize() { switchVisualizeMode = false; }
        // Get Angle
        void get_angle(float& g1, float& g2, float& g3, float& g4, float& g5, float& g6);
    };

}
