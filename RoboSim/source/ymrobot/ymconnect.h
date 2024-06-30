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
/// Struct used for interfacing between the UI and the Move Robot functionality. 
/// The Move Robot is a critical function of this software, facilitating various types of robot movements.
///
/// Members:
/// ========
/// - coumove: (int) The count of moves. Default is 3.
/// - joinflag: (bool) Indicates if the move is a joint move. Default is false.
/// - circuflag: (bool) Indicates if the move is a circular move. Default is false.
/// - linMFlag: (bool) Indicates if the move is a linear move. Default is false.
/// - lineshpath: (bool) Flag to show the move path. Default is false.
/// - rbpos: (std::vector<std::vector<float>>) The robot positions, represented as a 2D vector of floats.
///          Each inner vector represents a position with six float values. Default is a 3x6 vector of zeros.
/// - spdlinear: (float) The linear speed for movements.
/// - spdjoint: (float) The joint speed for movements.
/// - limitangle: (float) The angle limit for movements.
/// - tpstatus: (std::unique_ptr<StatusInfo>) A unique pointer to temporary status information. 
///             Initialized with a new StatusInfo object.
/// - b1PositionData: (std::unique_ptr<BaseAxisPositionVariableData>) A unique pointer to temporary position data from the robot. 
///                   Initialized with a new BaseAxisPositionVariableData object.
/// - b1origi: (std::unique_ptr<PositionData>) A unique pointer to the original position data.
///            Initialized with a new PositionData object.
/// - b1crpos: (std::unique_ptr<PositionData>) A unique pointer to the current position data.
///            Initialized with a new PositionData object.
struct UIState {
    int coumove{ 3 };
    bool joinflag{ false };
    bool circuflag{ false };
    bool linMFlag{ false };
    bool lineshpath{ false };
    float spdlinear{};
    float spdjoint{};
    std::vector<std::vector<float>> rbpos{ std::vector<std::vector<float>>(3, std::vector<float>(6, 0.0f)) };
    std::unique_ptr<StatusInfo> tpstatus = std::make_unique<StatusInfo>();
    std::unique_ptr<BaseAxisPositionVariableData> b1PositionData = std::make_unique<BaseAxisPositionVariableData>();
    std::unique_ptr<PositionData> b1origi = std::make_unique<PositionData>();
    std::unique_ptr<PositionData> b1crpos = std::make_unique<PositionData>();
};

namespace nymrobot {

    class ymconnect {
    private:
        StatusInfo status;
        MotomanController* controller;
        UINT32 restime = 10;
        bool switchVisualizeMode = false;
        std::vector<std::vector<float>> limitangle;

        float angle[6];
        nelems::mMesh* proMeshRb = nullptr;
        std::stringstream resultmsg;
        nui::StatusLogs* sttlogs = nullptr;
        ymconnect();
        ~ymconnect();

        ymconnect(const ymconnect&) = delete;
        ymconnect(ymconnect&&) = delete;
        ymconnect& operator=(const ymconnect&) = delete;
        ymconnect& operator=(ymconnect&&) = delete;

    public:
        static ymconnect& getInstance() {
            static ymconnect instance;
            return instance;
        }
        // Connect to robot
        void connect_robot();
		// Disconnect from robot
        void disconnect_robot(bool showmsg);
        // Render the Robot & UI
        void render();
        // Limangle for Robot
        void set_limitangle(const std::vector<std::vector<float>>& getlim) { limitangle = getlim; }
        // Check files in directory
        int check_files_in_directory();
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
