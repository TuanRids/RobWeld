#include "pch.h"
#include "ymconnect.h"
#include "windows.h"
#include <chrono>
#include "elems/vertex_holder.h"
#include <filesystem>

namespace fs = std::filesystem;

namespace nymrobot {

    std::vector<std::vector<float>> ymconnect::get6pos(0, std::vector<float>(6, 0.0f));

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
        //ImGui::SetWindowSize(ImVec2(400, 100));
        ImGui::SetNextItemWidth(120);
        ImGui::InputText("IP", ip_address, sizeof(ip_address));
        if (status.StatusCode == 0) { strcpy_s(connect_content, "Connected."); }
        ImGui::SetNextItemWidth(120);
        ImGui::InputText("Msg", connect_content, sizeof(connect_content));
        if (ImGui::Button("Connect") && status.StatusCode != 0) {
            std::string getip{ ip_address };
            strcpy_s(connect_content, "Welcome to OhLabs.");
            controller = YMConnect::OpenConnection(getip, status);
            if (status.StatusCode != 0) {
                std::stringstream ss; ss << status;
                *sttlogs << "Error: " + ss.str();
            }
            else {
                status = controller->ControlCommands->DisplayStringToPendant(connect_content);
                *sttlogs << "Connected";
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Disconnect")) {
            *sttlogs << "Disconnect" ;
            if (status.StatusCode == 0)
            {
                status = controller->ControlCommands->DisplayStringToPendant("Disconnect!");
                disconnect_robot(false);
            }
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
        // get proMeshRb and Statuslogs
        if (!proMeshRb) { proMeshRb = &nelems::mMesh::getInstance(); }

        connect_robot();

        static int x{ 200 }, y{ 400 };
        static float pos_x, pos_y,sizex,sizey;
        nui::FrameManage::getViewportSize(pos_x, pos_y);
        nui::FrameManage::get3DSize(sizex, sizey);
        ImGui::SetNextWindowPos(ImVec2(pos_x + 15+ sizex*0.83, pos_y + sizey * 0.25)); // Set the position of the frame
        ImGui::SetNextWindowSize(ImVec2(sizex*0.15, sizey*0.73)); // Set the size of the frame
        ImGui::Begin("Robot Status", nullptr,
            ImGuiWindowFlags_NoDocking | // Cannot be docked
            ImGuiWindowFlags_NoBackground | // Do not display background
            ImGuiWindowFlags_NoNavFocus); // Does not bring to front on focus

        if (ImGui::Button("Clear Fault") && status.StatusCode == 0) {
            resultmsg.str(" ");
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


    struct JointMotionFunctor {auto operator()(ControlGroupId groupId, PositionData& posData, float speed) const {
            return JointMotion(groupId, posData, speed);
        }};
    struct LinearMotionFunctor {auto operator()(ControlGroupId groupId, PositionData& posData, float speed) const {
            return LinearMotion(groupId, posData, speed);
        }};

    void ymconnect::move_robot() {
        static UIState ui_state;
        setup_MOVE_ui(ui_state);
        static auto LinepathStart = std::chrono::high_resolution_clock::now();

        double Linepathelapsed_seconds = std::chrono::duration_cast<std::chrono::duration<double>>
            (std::chrono::high_resolution_clock::now() - LinepathStart).count();

        // RENDER LINE PATH
        if (ui_state.lineshpath && Linepathelapsed_seconds > 0.4f) {
            LinepathStart = std::chrono::high_resolution_clock::now();

            std::shared_ptr<nelems::oMesh> newmesh = std::make_shared<nelems::oMesh>();
            if (!proMeshRb->get_mesh_byname("movepath__SKIP__", newmesh)) {
                newmesh = std::make_shared<nelems::oMesh>();
                newmesh->changeName("movepath__SKIP__");
                newmesh->ID = proMeshRb->getCurrentTimeMillis(0);
                newmesh->oMaterial.mColor = glm::vec3(1.0f, 0.0f, 0.0f);
            }
            else {
                newmesh->delete_buffers();
                newmesh->mVertexIndices.clear();
                newmesh->mVertices.clear();
            }
            nelems::VertexHolder vertex{};
            for (int i = 0; i < ui_state.coumove; ++i) {
                vertex.mPos = glm::vec3(ui_state.rbpos[i][0], ui_state.rbpos[i][1], ui_state.rbpos[i][2] ); 
                vertex.mNormal = glm::vec3(0.0f, 0.0f, 1.0f);
                newmesh->add_vertex(vertex);
                if (i > 0) {
                    newmesh->add_vertex_index(i - 1);
                    newmesh->add_vertex_index(i);
                }
            }
            newmesh->init();
            newmesh->selected = true;
            proMeshRb->add_mesh(*newmesh);
        }
        if (status.StatusCode != 0) { return; }
        // LAMBDA FOR MOTION
        auto execute_move = [&](auto&& motion_functor) {
            std::string stent = "Start the Motion to " + std::to_string(ui_state.coumove) + " points: ";
            for (int j = 0; j < ui_state.coumove; ++j) {
                ui_state.b1crpos->coordinateType = CoordinateType::RobotCoordinate;
                stent += "\n" + std::to_string(j) + ": ";
                for (int i = 0; i < 6; ++i) {
                    ui_state.b1crpos->axisData[i] = ui_state.rbpos[j][i];
                    stent += std::to_string(ui_state.rbpos[j][i]) + " ";
                }
                auto motion = motion_functor(ControlGroupId::R1, *ui_state.b1crpos, (std::is_same_v<decltype(motion_functor), JointMotionFunctor>) ? ui_state.spdjoint : ui_state.spdlinear);
                *ui_state.tpstatus = controller->MotionManager->AddPointToTrajectory(motion);
            }
            switchVisualizeMode = true;
            *ui_state.tpstatus = controller->ControlCommands->SetServos(SignalStatus::ON);
            *ui_state.tpstatus = controller->MotionManager->MotionStart();

            *sttlogs << stent;
            };
        // MOTION
        if (ui_state.joinflag) { execute_move(JointMotionFunctor{}); }
        if (ui_state.linMFlag) { execute_move(LinearMotionFunctor{}); }
        if (ui_state.circuflag) {
            ui_state.b1crpos->coordinateType = CoordinateType::RobotCoordinate;
            for (int i = 0; i < 6; ++i) {
                ui_state.b1crpos->axisData[i] = ui_state.rbpos[0][i];
            }
            *ui_state.tpstatus = controller->MotionManager->AddPointToTrajectory(JointMotion(ControlGroupId::R1, *ui_state.b1crpos, ui_state.spdjoint));

            for (int j = 2; j < ui_state.coumove; j += 2) {
                ui_state.b1crpos->coordinateType = CoordinateType::RobotCoordinate;
                CoordinateArray coorarr;
                for (int i = 0; i < 6; ++i) {
                    ui_state.b1crpos->axisData[i] = ui_state.rbpos[j][i];
                    coorarr[i] = ui_state.rbpos[j - 1][i];
                }
                *ui_state.tpstatus = controller->MotionManager->AddPointToTrajectory(CircularMotion(ControlGroupId::R1, *ui_state.b1crpos, coorarr, ui_state.spdlinear));
            }
            switchVisualizeMode = true;
            *ui_state.tpstatus = controller->ControlCommands->SetServos(SignalStatus::ON);
            *ui_state.tpstatus = controller->MotionManager->MotionStart();
            *sttlogs << "Start the Circular Motion";
        }
    }
  
    void ymconnect::setup_MOVE_ui(UIState& ui_state)
    {
        ImGui::Separator();
        ImGui::Begin("Attributes: ");
        static auto LinepathStart = std::chrono::high_resolution_clock::now();
        ImGui::SetNextItemWidth(80);
        ImGui::InputInt("##", &ui_state.coumove, 1, 2);
        if (ui_state.coumove < 1) { ui_state.coumove = 1; }; ImGui::SameLine();
        ui_state.joinflag = ImGui::Button("Joint"); ImGui::SameLine();
        ui_state.circuflag = ImGui::Button("Circular"); ImGui::SameLine();
        ui_state.linMFlag = ImGui::Button("Linear"); ImGui::SameLine();
        if (ImGui::Button(ui_state.lineshpath ? "Path On" : "Path Off")) {
            ui_state.lineshpath = !ui_state.lineshpath; // Toggle MovePath
            if (ui_state.lineshpath) { *sttlogs << "Render the Move Path"; }
            else { *sttlogs << "Stop the Move Path"; proMeshRb->delete_byname("movepath__SKIP__"); }
        }
        
        ImGui::SameLine();

        shmdata->getter_6pos(get6pos);
        if (ui_state.rbpos.size() < get6pos.size()) {
            ui_state.coumove = get6pos.size();
            ui_state.rbpos.resize(get6pos.size(), std::vector<float>(6, 0.0f));
        }
        if (get6pos.size() > 0) {
            for (size_t i = 0; i < get6pos.size(); ++i) {
                for (size_t j = 0; j < 6; ++j) {
                    ui_state.rbpos[i][j] = get6pos[i][j];
                }
            }
        }
        ImGui::SameLine(); // Check the trajectory
        if (ImGui::Button("Clear") && status.StatusCode == 0)
        {
            *ui_state.tpstatus = controller->MotionManager->ClearAllTrajectory();
            *ui_state.tpstatus = controller->MotionManager->ClearGroupTrajectory(ControlGroupId::R1);
            controller->ControlCommands->SetServos(SignalStatus::OFF);
            *ui_state.tpstatus = controller->MotionManager->MotionStart();
            *sttlogs << "trying to clear trajectory";
        }
        ImGui::SameLine();
        if (ImGui::Button("Home") && status.StatusCode == 0)
        {
            *ui_state.tpstatus = controller->Variables->BasePositionVariable->Read(0, *ui_state.b1PositionData);
            *ui_state.tpstatus = controller->Kinematics->ConvertPosition(ControlGroupId::R1, ui_state.b1PositionData->positionData, KinematicConversions::PulseToCartesianPos, *ui_state.b1origi);
            ui_state.b1origi->coordinateType = CoordinateType::RobotCoordinate;
            JointMotion r1home(ControlGroupId::R1, *ui_state.b1origi, 3);
            *ui_state.tpstatus = controller->MotionManager->AddPointToTrajectory(r1home);
            *ui_state.tpstatus = controller->ControlCommands->SetServos(SignalStatus::ON);
            *ui_state.tpstatus = controller->MotionManager->MotionStart();
            *sttlogs << "Go back to the Home pos";
        }
        ImGui::SameLine();
        ImGui::Text("Linear spd:"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
        ImGui::InputFloat("##LS", &ui_state.spdlinear, 0.0f, 0.0f, "%.2f");  ImGui::SameLine();
        ImGui::Text("Joint spd:"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
        ImGui::InputFloat("##JS", &ui_state.spdjoint, 0.0f, 0.0f, "%.2f");
        ImGui::Separator();
        for (int j = 0; j < ui_state.coumove; ++j) {
            if (ui_state.rbpos.size() < static_cast<size_t>(ui_state.coumove)) {
                ui_state.rbpos.resize(ui_state.coumove, std::vector<float>(6, 0.0f));
            }
            if (ImGui::Button(("OrgPos " + std::to_string(j)).c_str()) && status.StatusCode == 0) {
                for (int i = 0; i < 6; ++i) {
                    *ui_state.tpstatus = controller->Variables->BasePositionVariable->Read(0, *ui_state.b1PositionData);
                    *ui_state.tpstatus = controller->Kinematics->ConvertPosition(ControlGroupId::R1, ui_state.b1PositionData->positionData, KinematicConversions::PulseToCartesianPos, *ui_state.b1origi);
                    ui_state.rbpos[j][i] = ui_state.b1origi->axisData[i];
                }
                *sttlogs << "Update the Original Position for " << std::to_string(j);
            }
            ImGui::SameLine();
            if (ImGui::Button(("CrtPos " + std::to_string(j)).c_str()) && status.StatusCode == 0) {
                for (int i = 0; i < 6; ++i) {
                    *ui_state.tpstatus = controller->ControlGroup->ReadPositionData(ControlGroupId::R1, CoordinateType::BaseCoordinate, 0, 0, *ui_state.b1crpos);
                    ui_state.rbpos[j][i] = ui_state.b1crpos->axisData[i];
                }
                *sttlogs << "Update the Current Position for " + std::to_string(j);
            }
            ImGui::SameLine();
            ImGui::Text(" X:"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
            ImGui::InputFloat(("##X" + std::to_string(j)).c_str(), &ui_state.rbpos[j][0], 0.0f, 0.0f, "%.2f"); ImGui::SameLine();
            ImGui::Text(" Y:"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
            ImGui::InputFloat(("##Y" + std::to_string(j)).c_str(), &ui_state.rbpos[j][1], 0.0f, 0.0f, "%.2f"); ImGui::SameLine();
            ImGui::Text(" Z:"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
            ImGui::InputFloat(("##Z" + std::to_string(j)).c_str(), &ui_state.rbpos[j][2], 0.0f, 0.0f, "%.2f"); ImGui::SameLine();
            ImGui::Text("Rx:"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
            ImGui::InputFloat(("##RX" + std::to_string(j)).c_str(), &ui_state.rbpos[j][3], 0.0f, 0.0f, "%.2f"); ImGui::SameLine();
            ImGui::Text("Ry:"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
            ImGui::InputFloat(("##RY" + std::to_string(j)).c_str(), &ui_state.rbpos[j][4], 0.0f, 0.0f, "%.2f"); ImGui::SameLine();
            ImGui::Text("Rz:"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
            ImGui::InputFloat(("##RZ" + std::to_string(j)).c_str(), &ui_state.rbpos[j][5], 0.0f, 0.0f, "%.2f");
        }
        ImGui::End();
    }

    void ymconnect::read_robot() {
        std::stringstream strget;
        StatusInfo tpstatus;
        PositionData raxisData{}, rposData{}, rjointangle{};
        static auto start = std::chrono::high_resolution_clock::now();
        ControllerStateData stateData{};
        if (status.StatusCode != 0) {            return;        }
        auto elapsed = std::chrono::high_resolution_clock::now() - start;
        double elapsed_seconds = std::chrono::duration_cast<std::chrono::duration<double>>(elapsed).count();

        if (elapsed_seconds > 0.1f) {
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

        bool isOutOfLimit = std::any_of(std::begin(angle), std::end(angle), [this, i = 0](float a) mutable {
            return a < limitangle[i][0] || a > limitangle[i++][1];
            });

        if (isOutOfLimit && stateData.isRunning) {
            controller->MotionManager->MotionStop();
            controller->ControlCommands->SetServos(SignalStatus::OFF);
            *sttlogs << "Error: Joint angle is out of limit\n\t Let use Pendant to move the robot to the ";
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