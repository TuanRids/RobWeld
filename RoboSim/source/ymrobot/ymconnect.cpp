#include "pch.h"
#include "ymconnect.h"
#include "windows.h"
#include <chrono>
#include "elems/vertex_holder.h"
#include <filesystem>

namespace fs = std::filesystem;

namespace nymrobot {

    ymconnect::ymconnect() : controller(nullptr) {
        YMConnect::OpenConnection("192.168.0.0", status, restime);
    }

    ymconnect::~ymconnect() {
        if (status.StatusCode == 0) {
            disconnect_robot(false);
        }
        delete controller;
    }

    void ymconnect::connect_robot() {
        static char ip_address[20] = "192.168.10.102";
        static char connect_content[100] = "Welcome";
        ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.0f, 0.3f, 0.2f, 1.0f));
        ImGui::Begin("GetIP", nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize);
        ImGui::SetWindowSize(ImVec2(400, 100));
        ImGui::SetNextItemWidth(150);
        ImGui::InputText("IP Address", ip_address, sizeof(ip_address));
        if (status.StatusCode == 0) { strcpy_s(connect_content, "Connected."); }
        ImGui::InputText("Status", connect_content, sizeof(connect_content));
        if (ImGui::Button("Connect") && status.StatusCode != 0) {
            std::string getip{ ip_address };
            strcpy_s(connect_content, "Welcome to OhLabs.");
            controller = YMConnect::OpenConnection(getip, status);
            if (status.StatusCode != 0) {
                std::stringstream ss; ss << status;
                MessageBox(NULL, ss.str().c_str(), "Error", MB_OK);
            }
            else {
                status = controller->ControlCommands->DisplayStringToPendant(connect_content);
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Disconnect")) {
            status = controller->ControlCommands->DisplayStringToPendant("Disconnect!");
            disconnect_robot(false);
            strcpy_s(connect_content, "Disconnect!");
        }
        ImGui::Separator();
        ImGui::End();
        ImGui::PopStyleColor();
    }

    void ymconnect::disconnect_robot(bool showmsg) {
        if (status.StatusCode == 0) {
            YMConnect::OpenConnection("192.168.0.0", status);
            controller = nullptr;
        }
        if (showmsg) { ImGui::OpenPopup("Disconnected"); }
    }

    void ymconnect::render() {
        if (!proMeshRb) {
            proMeshRb = &nelems::mMesh::getInstance();
        }
        connect_robot();
        if (status.StatusCode != 0) { return; }

        ImGui::Begin("Robot Control");
        if (ImGui::Button("Clear Fault")) {
            std::unique_ptr<StatusInfo> temptstt = std::make_unique<StatusInfo>();
            *temptstt = controller->Faults->ClearAllFaults();
            if (temptstt->StatusCode == 0) { resultmsg.str(" "); }
        }
        move_robot();
        read_robot();
        ImGui::Separator();
        if (resultmsg) { ImGui::Text(resultmsg.str().c_str()); }
        ImGui::End();
    }

    int ymconnect::check_files_in_directory() {
        const std::wstring& directory_path{ L".\\pysrc" };
        bool has_exe = false;
        bool has_py = false;
        if (!fs::exists(directory_path) || !fs::is_directory(directory_path)) {
            std::wcerr << L"Directory does not exist: " << directory_path << std::endl;
            return 0;
        }
        for (const auto& entry : fs::directory_iterator(directory_path)) {
            if (entry.is_regular_file()) {
                if (entry.path().filename() == L"postocpp.exe") { has_exe = true; }
                if (entry.path().filename() == L"postocpp.py") { has_py = true; }
            }
        }
        return has_exe ? 2 : (has_py ? 1 : 0);
    }
    struct JointMotionFunctor {
        auto operator()(ControlGroupId groupId, PositionData& posData, float speed) const {
            return JointMotion(groupId, posData, speed);
        }
    };

    struct LinearMotionFunctor {
        auto operator()(ControlGroupId groupId, PositionData& posData, float speed) const {
            return LinearMotion(groupId, posData, speed);
        }
    };

    void ymconnect::move_robot() {
        std::unique_ptr<StatusInfo> tpstatus = std::make_unique<StatusInfo>();
        ImGui::Separator();
        ImGui::Begin("Attributes: ");
        std::unique_ptr<BaseAxisPositionVariableData> b1PositionData = std::make_unique<BaseAxisPositionVariableData>();
        std::unique_ptr<PositionData> b1origi = std::make_unique<PositionData>();
        std::unique_ptr<PositionData> b1crpos = std::make_unique<PositionData>();

        static std::vector<std::vector<float>> rbpos(3, std::vector<float>(6, 0.0f));
        static float spdlinear{};
        static float spdjoint{};
        static int coumove{ 3 };

        ImGui::SetNextItemWidth(150);
        ImGui::InputInt("Times", &coumove, 1, 2);
        if (coumove < 1) { coumove = 1; }
        bool joinflag = ImGui::Button("Joint Move");
        bool circuflag = ImGui::Button("Circular Move");
        bool linMFlag = ImGui::Button("Linear Move");
        bool lineshpath = ImGui::Button("MovingPATH! NOT AVAILABLE");

        if (ImGui::Button("Loadpy")) {
            int checkfilepysrc = check_files_in_directory();
            std::vector<std::vector<double>> get6pos;
            if (checkfilepysrc == 1) { get6pos = readpysrc.get_values_from_python(); }
            else if (checkfilepysrc == 2) { get6pos = readpysrc.get_values_from_exe(); }

            if (rbpos.size() < get6pos.size()) {
                coumove = get6pos.size();
                rbpos.resize(get6pos.size(), std::vector<float>(6, 0.0f));
            }

            for (size_t i = 0; i < get6pos.size(); ++i) {
                for (size_t j = 0; j < 6; ++j) {
                    rbpos[i][j] = get6pos[i][j];
                }
            }
        }

        if (ImGui::Button("Home")) {
            *tpstatus = controller->MotionManager->ClearAllTrajectory();
            *tpstatus = controller->Variables->BasePositionVariable->Read(0, *b1PositionData);
            *tpstatus = controller->Kinematics->ConvertPosition(ControlGroupId::R1, b1PositionData->positionData, KinematicConversions::PulseToCartesianPos, *b1origi);
            b1origi->coordinateType = CoordinateType::RobotCoordinate;
            JointMotion r1home(ControlGroupId::R1, *b1origi, 3);
            *tpstatus = controller->MotionManager->AddPointToTrajectory(r1home);
            *tpstatus = controller->ControlCommands->SetServos(SignalStatus::ON);
            *tpstatus = controller->MotionManager->MotionStart();
        }

        ImGui::Text("Linear spd:"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
        ImGui::InputFloat("##LS", &spdlinear, 0.0f, 0.0f, "%.2f");  ImGui::SameLine();
        ImGui::Text("Joint spd:"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
        ImGui::InputFloat("##JS", &spdjoint, 0.0f, 0.0f, "%.2f");

        for (int j = 0; j < coumove; ++j) {
            if (rbpos.size() < static_cast<size_t>(coumove)) {
                rbpos.resize(coumove, std::vector<float>(6, 0.0f));
            }
            if (ImGui::Button(("OrgPos " + std::to_string(j)).c_str())) {
                for (int i = 0; i < 6; ++i) {
                    *tpstatus = controller->Variables->BasePositionVariable->Read(0, *b1PositionData);
                    *tpstatus = controller->Kinematics->ConvertPosition(ControlGroupId::R1, b1PositionData->positionData, KinematicConversions::PulseToCartesianPos, *b1origi);
                    rbpos[j][i] = b1origi->axisData[i];
                }
            }
            if (ImGui::Button(("CrtPos " + std::to_string(j)).c_str())) {
                for (int i = 0; i < 6; ++i) {
                    *tpstatus = controller->ControlGroup->ReadPositionData(ControlGroupId::R1, CoordinateType::BaseCoordinate, 0, 0, *b1crpos);
                    rbpos[j][i] = b1crpos->axisData[i];
                }
            }
            ImGui::Text(" X:"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
            ImGui::InputFloat(("##X" + std::to_string(j)).c_str(), &rbpos[j][0], 0.0f, 0.0f, "%.2f"); ImGui::SameLine();
            ImGui::Text(" Y:"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
            ImGui::InputFloat(("##Y" + std::to_string(j)).c_str(), &rbpos[j][1], 0.0f, 0.0f, "%.2f"); ImGui::SameLine();
            ImGui::Text(" Z:"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
            ImGui::InputFloat(("##Z" + std::to_string(j)).c_str(), &rbpos[j][2], 0.0f, 0.0f, "%.2f"); ImGui::SameLine();
            ImGui::Text("Rx:"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
            ImGui::InputFloat(("##RX" + std::to_string(j)).c_str(), &rbpos[j][3], 0.0f, 0.0f, "%.2f"); ImGui::SameLine();
            ImGui::Text("Ry:"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
            ImGui::InputFloat(("##RY" + std::to_string(j)).c_str(), &rbpos[j][4], 0.0f, 0.0f, "%.2f"); ImGui::SameLine();
            ImGui::Text("Rz:"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
            ImGui::InputFloat(("##RZ" + std::to_string(j)).c_str(), &rbpos[j][5], 0.0f, 0.0f, "%.2f");
        }

        if (lineshpath) {
            // Show moving path logic here (if available)
        }

        auto execute_move = [&](auto&& motion_functor) {
            for (int j = 0; j < coumove; ++j) {
                b1crpos->coordinateType = CoordinateType::RobotCoordinate;
                for (int i = 0; i < 6; ++i) {
                    b1crpos->axisData[i] = rbpos[j][i];
                }
                auto motion = motion_functor(ControlGroupId::R1, *b1crpos, (std::is_same_v<decltype(motion_functor), JointMotionFunctor>) ? spdjoint : spdlinear);
                *tpstatus = controller->MotionManager->AddPointToTrajectory(motion);
            }
            switchVisualizeMode = true;
            *tpstatus = controller->ControlCommands->SetServos(SignalStatus::ON);
            *tpstatus = controller->MotionManager->MotionStart();
            };

        if (joinflag) { execute_move(JointMotionFunctor{}); }
        if (linMFlag) { execute_move(LinearMotionFunctor{}); }
        if (circuflag) {
            b1crpos->coordinateType = CoordinateType::RobotCoordinate;
            for (int i = 0; i < 6; ++i) {
                b1crpos->axisData[i] = rbpos[0][i];
            }
            *tpstatus = controller->MotionManager->AddPointToTrajectory(JointMotion(ControlGroupId::R1, *b1crpos, spdjoint));

            for (int j = 2; j < coumove; j += 2) {
                b1crpos->coordinateType = CoordinateType::RobotCoordinate;
                CoordinateArray coorarr;
                for (int i = 0; i < 6; ++i) {
                    b1crpos->axisData[i] = rbpos[j][i];
                    coorarr[i] = rbpos[j - 1][i];
                }
                *tpstatus = controller->MotionManager->AddPointToTrajectory(CircularMotion(ControlGroupId::R1, *b1crpos, coorarr, spdlinear));
            }
            switchVisualizeMode = true;
            *tpstatus = controller->ControlCommands->SetServos(SignalStatus::ON);
            *tpstatus = controller->MotionManager->MotionStart();
        }

        ImGui::End();
    }

    void ymconnect::read_robot() {
        std::stringstream strget;
        StatusInfo tpstatus;
        PositionData raxisData{}, rposData{}, rjointangle{};
        static auto start = std::chrono::high_resolution_clock::now();
        ControllerStateData stateData{};

        auto elapsed = std::chrono::high_resolution_clock::now() - start;
        if (elapsed.count() > 0.1f) {
            controller->Status->ReadState(stateData);
            resultmsg.str(" ");
            resultmsg << stateData;

            tpstatus = controller->ControlGroup->ReadPositionData(ControlGroupId::R1, CoordinateType::BaseCoordinate, 0, 0, rposData);
            resultmsg << rposData;

            if (stateData.isAlarming) {
                AlarmHistory alarmHistoryData;
                controller->Faults->GetAlarmHistory(AlarmCategory::Minor, 3, alarmHistoryData);
                resultmsg << alarmHistoryData << std::endl;
            }

            auto getper = controller->MotionManager->GetMotionTargetProgress(ControlGroupId::R1, tpstatus);
            resultmsg << "\n RUN % : " << getper << std::endl;
            start = std::chrono::high_resolution_clock::now();
        }

        tpstatus = controller->ControlGroup->ReadPositionData(ControlGroupId::R1, CoordinateType::Pulse, 0, 0, raxisData);
        tpstatus = controller->Kinematics->ConvertPosition(ControlGroupId::R1, raxisData, KinematicConversions::PulseToJointAngle, rjointangle);

        for (int i = 0; i < 6; ++i) {
            angle[i] = rjointangle.axisData[i];
        }


        //for (int i = 0; i < 6; ++i) {
        //    if (angle[i] < limitangle[i][0] || angle[i] > limitangle[i][1]) {
        //        isOutOfLimit = true;
        //        break;
        //    }
        //}
        bool isOutOfLimit = std::any_of(std::begin(angle), std::end(angle), [this, i = 0](float a) mutable {
            return a < limitangle[i][0] || a > limitangle[i++][1];
            });

        if (isOutOfLimit && stateData.isRunning) {
            controller->MotionManager->MotionStop();
            controller->ControlCommands->SetServos(SignalStatus::OFF);
            MessageBox(NULL, "Error: Joint angle is out of limit", "Error", MB_OK);
        }
    }

    void ymconnect::get_angle(float& g1, float& g2, float& g3, float& g4, float& g5, float& g6) {
        if (status.StatusCode != 0) { return; }
        g1 = angle[0];
        g2 = angle[1];
        g3 = angle[2];
        g4 = angle[3];
        g5 = angle[4];
        g6 = angle[5];
    }

}


//#include "pch.h"
//#include "ymconnect.h"
//#include "windows.h"
//#include <chrono>
//#include "elems/vertex_holder.h"
//#include <filesystem>
//
//namespace fs = std::filesystem;
//
//namespace nymrobot
//{
//    //bool ymconnect::connect_trigger = false;
//    void ymconnect::connect_robot()
//    {
//
//        static char ip_address[20] = "192.168.10.102"; // Default IP address
//        static char connect_content[100] = "Welcome";
//        // set UI
//        ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.0f, 0.3f, 0.2f, 1.0f));
//        ImGui::Begin("GetIP", nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize);
//        ImGui::SetWindowSize(ImVec2(400, 100));
//        ImGui::SetNextItemWidth(150);
//
//        ImGui::InputText("IP Address", ip_address, sizeof(ip_address)); ImGui::SetNextItemWidth(150);
//
//        if (status.StatusCode == 0) { strcpy_s(connect_content, "Connected."); }
//        ImGui::InputText("Status", connect_content, sizeof(connect_content));
//
//        if (ImGui::Button("Connect") && status.StatusCode != 0) {
//            std::string getip{ ip_address };
//
//            // set content to make sure it is updated for 2nd time of connection
//            strcpy_s(connect_content, "Welcome to OhLabs.");
//            controller = YMConnect::OpenConnection(getip, status); // Open a connection to the robot controller
//
//            if (status.StatusCode != 0) {
//                std::stringstream ss;
//                ss << status;
//                std::string message = ss.str();
//                MessageBox(NULL, message.c_str(), "Error", MB_OK);
//            }
//            else
//            {
//                status = controller->ControlCommands->DisplayStringToPendant(connect_content);
//            }
//        }
//
//        ImGui::SameLine();
//        if (ImGui::Button("Disconnect")) {
//            status = controller->ControlCommands->DisplayStringToPendant((char*)"Disconnect!");
//            disconnect_robot(false);
//            strcpy_s(connect_content, "Disconnect!");
//
//        }
//
//        ImGui::Separator();
//        ImGui::End();
//        ImGui::PopStyleColor();
//    }
//
//    void ymconnect::disconnect_robot(bool showmsg)
//    {
//        if (status.StatusCode == 0)
//        {
//            // YMConnect::CloseConnection(controller);
//            YMConnect::OpenConnection("192.168.0.0", status); // Fake Login for destroy status
//            controller = nullptr;
//        }
//        if (showmsg)
//        { ImGui::OpenPopup("Disconnected"); }
//
//    }
//    void ymconnect::render()
//    {
//        // =========================================================================================================
//        // get ptr to the robot mesh
//        if (!proMeshRb)
//        {
//            proMeshRb = &nelems::mMesh::getInstance();
//        }
//
//        // if trigger is true, show the UI to connect to the robot
//        connect_robot();
//        if (status.StatusCode != 0) { return; }
//
//        // UI for controlling the Robot
//        ImGui::Begin("Robot Control");
//        if (ImGui::Button("Clear Fault"))
//        {
//            StatusInfo* temptstt = new StatusInfo();
//            *temptstt = controller->Faults->ClearAllFaults();
//            if (temptstt->StatusCode == 0) { resultmsg.str(" "); }
//            delete temptstt;
//        }
//        move_robot();
//        read_robot();
//        ImGui::Separator();
//
//        if (resultmsg) { ImGui::Text(resultmsg.str().c_str()); }
//        ImGui::End();
//    }
//
//    int ymconnect::check_files_in_directory()
//    {
//        const std::wstring& directory_path{ L".\\pysrc" };
//        bool has_exe = false;
//        bool has_py = false;
//
//        // Check if the directory exists
//        if (!fs::exists(directory_path) || !fs::is_directory(directory_path)) {
//            std::wcerr << L"Directory does not exist: " << directory_path << std::endl;
//            return 0;
//        }
//
//        // Iterate through the files in the directory
//        for (const auto& entry : fs::directory_iterator(directory_path)) {
//            if (entry.is_regular_file()) {
//                if (entry.path().filename() == L"postocpp.exe") {
//                    has_exe = true;
//                }
//                if (entry.path().filename() == L"postocpp.py") {
//                    has_py = true;
//                }
//            }
//        }
//
//        // Determine the return value based on the presence of the files
//        if (has_exe) {
//            return 2;
//        }
//        if (has_py) {
//            return 1;
//        }
//        return 0;
//    }
//
//    void ymconnect::move_robot() {
//        //===============================================================================================================================================================
//        // IMPORTANT Variables
//        std::unique_ptr<StatusInfo> tpstatus = std::make_unique<StatusInfo>();
//        ImGui::Separator();
//        ImGui::Begin("Attributes: ");
//        BaseAxisPositionVariableData* b1PositionData = new BaseAxisPositionVariableData();
//        PositionData* b1origi = new PositionData();
//        PositionData* b1crpos = new PositionData();
//
//        static std::vector<std::vector<float>> rbpos(3, std::vector<float>(6, 0.0f));
//        static float spdlinear{};
//        static float spdjoint{};
//        static int coumove{ 3 };
//
//        ImGui::SetNextItemWidth(150);
//        //===============================================================================================================================================================
//        // MAIN BUTTONS
//        // HOME, JOINT MOVE< CIRCULAR MOVE, LINEAR MOVE, SHOW PATH, LOADPY
//        ImGui::InputInt("Times", &coumove, 1, 2); ImGui::SameLine();
//        if (coumove < 1) { coumove = 1; }
//        bool joinflag = ImGui::Button("Joint Move"); ImGui::SameLine();
//        bool circuflag = ImGui::Button("Circular Move"); ImGui::SameLine();
//        bool linMFlag = ImGui::Button("Linear Move"); ImGui::SameLine();
//        bool lineshpath = ImGui::Button("show MovingPATH !NOT AVAILABLE");
//
//        if (ImGui::Button("Loadpy"))
//        {
//            int checkfilepysrc = check_files_in_directory(); // 0 = nothing, 1 = py, 2 = exe
//
//            std::vector<std::vector<double>> get6pos;
//            if (checkfilepysrc == 1)        { get6pos = readpysrc.get_values_from_python(); }
//            else if (checkfilepysrc==2)     { get6pos = readpysrc.get_values_from_exe(); }
//            else                            { goto endload; }
//
//            if (rbpos.size() < get6pos.size()) 
//            {
//                coumove = get6pos.size();
//                int numToAdd = get6pos.size() - rbpos.size();
//                for (int i = 0; i < numToAdd; ++i) 
//                {
//                    rbpos.push_back(std::vector<float>(6, 0.0f));
//                }
//            }
//            if (get6pos.size() > 0)
//            {
//                for (int i{ 0 }; i < get6pos.size(); i++)
//                {
//                    rbpos[i][0] = get6pos[i][0];
//                    rbpos[i][1] = get6pos[i][1];
//                    rbpos[i][2] = get6pos[i][2];
//                    rbpos[i][3] = get6pos[i][3];
//                    rbpos[i][4] = get6pos[i][4];
//                    rbpos[i][5] = get6pos[i][5];
//                }
//            }
//        endload:
//            ; // done
//        }
//        ImGui::SameLine();
//        if (ImGui::Button("Home"))
//        {
//            ///MAYBE ERROR
//            *tpstatus = controller->MotionManager->ClearAllTrajectory();
//
//            *tpstatus = controller->Variables->BasePositionVariable->Read(0, *b1PositionData);
//            *tpstatus = controller->Kinematics->ConvertPosition(ControlGroupId::R1, b1PositionData->positionData, KinematicConversions::PulseToCartesianPos, *b1origi);
//            b1origi->coordinateType = CoordinateType::RobotCoordinate;
//
//            JointMotion r1home(ControlGroupId::R1, *b1origi, 3);
//            *tpstatus = controller->MotionManager->AddPointToTrajectory(r1home);
//            *tpstatus = controller->ControlCommands->SetServos(SignalStatus::ON);
//            *tpstatus = controller->MotionManager->MotionStart();
//            goto end1;
//
//        }
//        ImGui::SameLine();
//        ImGui::Text("Linear spd:"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
//        ImGui::InputFloat("##LS", &spdlinear, 0.0f, 0.0f, "%.2f");  ImGui::SameLine();
//        ImGui::Text("Joint spd:"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
//        ImGui::InputFloat("##JS", &spdjoint, 0.0f, 0.0f, "%.2f");
//
//        //===============================================================================================================================================================
//        // Create the UI for coordinates
//        for (int j = 0; j < coumove; j++) {
//            if (rbpos.size() < coumove) {
//                int numToAdd = coumove - rbpos.size();
//                for (int i = 0; i < numToAdd; ++i) {
//                    rbpos.push_back(std::vector<float>(6, 0.0f));
//                }
//            }
//            if (ImGui::Button(("OrgPos " + std::to_string(j)).c_str()))
//            {
//                for (int i = 0; i < 6; i++)
//                {
//                    *tpstatus = controller->Variables->BasePositionVariable->Read(0, *b1PositionData);
//                    *tpstatus = controller->Kinematics->ConvertPosition(ControlGroupId::R1, b1PositionData->positionData, KinematicConversions::PulseToCartesianPos, *b1origi);
//                    rbpos[j][i] = b1origi->axisData[i];
//                }
//            }
//            ImGui::SameLine();
//            if (ImGui::Button(("CrtPos " + std::to_string(j)).c_str()))
//            {
//                for (int i = 0; i < 6; i++)
//                {
//                    *tpstatus = controller->ControlGroup->ReadPositionData(ControlGroupId::R1, CoordinateType::BaseCoordinate, 0, 0, *b1crpos);
//                    rbpos[j][i] = b1crpos->axisData[i];
//                }
//            }
//            ImGui::SameLine();
//
//            ImGui::Text(" X:"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
//            ImGui::InputFloat(("##X" + std::to_string(j)).c_str(), &rbpos[j][0], 0.0f, 0.0f, "%.2f"); ImGui::SameLine();
//            ImGui::Text(" Y:"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
//            ImGui::InputFloat(("##Y" + std::to_string(j)).c_str(), &rbpos[j][1], 0.0f, 0.0f, "%.2f"); ImGui::SameLine();
//            ImGui::Text(" Z:"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
//            ImGui::InputFloat(("##Z" + std::to_string(j)).c_str(), &rbpos[j][2], 0.0f, 0.0f, "%.2f"); ImGui::SameLine();
//
//            ImGui::Text("Rx:"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
//            ImGui::InputFloat(("##RX" + std::to_string(j)).c_str(), &rbpos[j][3], 0.0f, 0.0f, "%.2f"); ImGui::SameLine();
//            ImGui::Text("Ry:"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
//            ImGui::InputFloat(("##RY" + std::to_string(j)).c_str(), &rbpos[j][4], 0.0f, 0.0f, "%.2f"); ImGui::SameLine();
//            ImGui::Text("Rz:"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
//            ImGui::InputFloat(("##RZ" + std::to_string(j)).c_str(), &rbpos[j][5], 0.0f, 0.0f, "%.2f");
//        }
//
//        if (lineshpath)
//        {
//            /* nelems::oMesh linepath;
//             for (int j = 0; j < coumove; j++) {
//                 nelems::VertexHolder vertex;
//                 vertex.mPos = { rbpos[j][0],rbpos[j][1],rbpos[j][2] };
//                 vertex.mNormal = { 0.0f, 0.0f, 1.0f };
//                 linepath.add_vertex(vertex);
//             }
//             linepath.changeName("LINEPATH");
//             proMeshRb->pushback(linepath);*/
//        }
//
//        //===============================================================================================================================================================
//        // Moving space
//        if (joinflag) {
//            ImGui::SetTooltip("Should Use!");
//            for (int j = 0; j < coumove; j++) {
//                b1crpos->coordinateType = CoordinateType::RobotCoordinate;
//                for (int i = 0; i < 6; i++) {
//                    b1crpos->axisData[i] = rbpos[j][i];
//                }
//                JointMotion r1movel(ControlGroupId::R1, *b1crpos, spdjoint);
//                *tpstatus = controller->MotionManager->AddPointToTrajectory(r1movel);
//            }
//
//            switchVisualizeMode = true;
//            *tpstatus = controller->ControlCommands->SetServos(SignalStatus::ON);
//            *tpstatus = controller->MotionManager->MotionStart();
//        }
//        ImGui::SameLine();
//        if (linMFlag) {
//            for (int j = 0; j < coumove; j++) {
//                b1crpos->coordinateType = CoordinateType::RobotCoordinate;
//                for (int i = 0; i < 6; i++) {
//                    b1crpos->axisData[i] = rbpos[j][i];
//                }
//                LinearMotion r1movel(ControlGroupId::R1, *b1crpos, spdlinear);
//                *tpstatus = controller->MotionManager->AddPointToTrajectory(r1movel);
//            }
//            switchVisualizeMode = true;
//            *tpstatus = controller->ControlCommands->SetServos(SignalStatus::ON);
//            *tpstatus = controller->MotionManager->MotionStart();
//        }
//        ImGui::SameLine();
//        if (circuflag) {
//            // Linear from current to 1st points
//            b1crpos->coordinateType = CoordinateType::RobotCoordinate;
//            for (int i = 0; i < 6; i++) {
//                b1crpos->axisData[i] = rbpos[0][i];
//            }
//            JointMotion r1movel(ControlGroupId::R1, *b1crpos, spdjoint);
//            *tpstatus = controller->MotionManager->AddPointToTrajectory(r1movel);
//            // Circular 0.1.2, 2.3.4
//            for (int j = 2; j < coumove; j += 2) {
//                b1crpos->coordinateType = CoordinateType::RobotCoordinate;
//                CoordinateArray coorarr;
//                for (int i = 0; i < 6; i++) {
//                    b1crpos->axisData[i] = rbpos[j][i];
//                    coorarr[i] = rbpos[j - 1][i];
//                }
//                CircularMotion r1movel(ControlGroupId::R1, *b1crpos, coorarr, spdlinear);
//                *tpstatus = controller->MotionManager->AddPointToTrajectory(r1movel);
//            }
//
//            switchVisualizeMode = true;
//            *tpstatus = controller->ControlCommands->SetServos(SignalStatus::ON);
//            *tpstatus = controller->MotionManager->MotionStart();
//        }
//
//        end1: // 
//        delete b1PositionData;
//        delete b1crpos;
//        delete b1origi;
//
//        ImGui::End();
//    }
//
//    // TODO Update a button for increasing FPS
//    void ymconnect::read_robot()
//    {
//        // Setup for reading status
//        std::stringstream* strget = new std::stringstream();
//        StatusInfo tpstatus;
//        PositionData raxisData{}, rposData{};
//        PositionData rjointangle{};
//        static auto start = std::chrono::high_resolution_clock::now();
//        ControllerStateData stateData{};
//
//        std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - start;
//        if (elapsed.count() > 0.1f)
//        {
//            // read state
//            controller->Status->ReadState(stateData);
//            resultmsg.str(" ");
//            resultmsg << stateData;
//
//            // read the position
//
//            tpstatus = controller->ControlGroup->ReadPositionData(ControlGroupId::R1, CoordinateType::BaseCoordinate, 0, 0, rposData);
//            resultmsg << rposData;
//
//            if (stateData.isAlarming)
//            {
//                AlarmHistory alarmHistoryData;
//                controller->Faults->GetAlarmHistory(AlarmCategory::Minor, 3, alarmHistoryData);
//                resultmsg << alarmHistoryData << std::endl;
//            }
//            // read the Joint of robot
//            auto getper = controller->MotionManager->GetMotionTargetProgress(ControlGroupId::R1, tpstatus);
//            resultmsg << "\n RUN % : " << getper << std::endl;
//
//
//            start = std::chrono::high_resolution_clock::now();
//        }
//        tpstatus = controller->ControlGroup->ReadPositionData(ControlGroupId::R1, CoordinateType::Pulse, 0, 0, raxisData);
//        tpstatus = controller->Kinematics->ConvertPosition(ControlGroupId::R1, raxisData
//            , KinematicConversions::PulseToJointAngle, rjointangle);
//        angle[0] = rjointangle.axisData[0];
//        angle[1] = rjointangle.axisData[1];
//        angle[2] = rjointangle.axisData[2];
//        angle[3] = rjointangle.axisData[3];
//        angle[4] = rjointangle.axisData[4];
//        angle[5] = rjointangle.axisData[5];
//
//        bool isOutOfLimit = false;
//
//        for (int i = 0; i < 6; ++i) {
//            if (angle[i] < limitangle[i][0] || angle[i] > limitangle[i][1]) {
//                isOutOfLimit = true;
//                break;
//            }
//        }
//
//        if (isOutOfLimit && stateData.isRunning) {
//            controller->MotionManager->MotionStop();
//            controller->ControlCommands->SetServos(SignalStatus::OFF);
//
//            MessageBox(NULL, "Error: Joint angle is out of limit", "Error", MB_OK);
//        }
//        delete strget;
//    }
//
//}			