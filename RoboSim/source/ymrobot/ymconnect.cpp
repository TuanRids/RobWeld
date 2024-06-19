#include "pch.h"
#include "ymconnect.h"
#include "windows.h"
#include <chrono>
#include "elems/vertex_holder.h"
#include <filesystem>

namespace fs = std::filesystem;

namespace nymrobot {

    ymconnect::ymconnect() : controller(nullptr), angle{} {
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
        ImGui::SetNextItemWidth(120);
        ImGui::InputText("IP Address", ip_address, sizeof(ip_address));
        if (status.StatusCode == 0) { strcpy_s(connect_content, "Connected."); }
        ImGui::SetNextItemWidth(120);
        ImGui::InputText("Status", connect_content, sizeof(connect_content));
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
        if (!sttlogs) { sttlogs = &nui::StatusLogs::getInstance(); }


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
        std::vector<std::filesystem::path> filep;
        filep.push_back( std::filesystem::current_path() / "pysrc" / "postocpp.exe");
        filep.push_back(std::filesystem::current_path() / "pysrc" / "postocpp.py");
        if (std::filesystem::exists(filep[0])) 
        { return 2; }
        if (std::filesystem::exists(filep[1])) 
        { return 1; }
        return 0;
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
        static auto LinepathStart = std::chrono::high_resolution_clock::now();
        ImGui::SetNextItemWidth(150);
        ImGui::InputInt("Times", &coumove, 1, 2);
        if (coumove < 1) { coumove = 1; }; ImGui::SameLine();
        bool joinflag = ImGui::Button("Joint Move"); ImGui::SameLine();
        bool circuflag = ImGui::Button("Circular Move"); ImGui::SameLine();
        bool linMFlag = ImGui::Button("Linear Move");ImGui::SameLine();
        bool lineshpath{ false };
        if (ImGui::Button(lineshpath ? "MovePath On" : "MovePath Off")) {
            lineshpath = !lineshpath; // Toggle MovePath
            if (lineshpath){ *sttlogs << "Render the Move Path"; }
            else { *sttlogs << "Stop the Move Path";proMeshRb->delete_byname("movepath__SKIP__"); }
        } ImGui::SameLine();


        if (ImGui::Button("Res path")) { proMeshRb->delete_byname("movepath__SKIP__"); }
        if (ImGui::Button("Loadpy")) {
            int checkfilepysrc = check_files_in_directory();
            std::vector<std::vector<double>> get6pos;
            if (checkfilepysrc == 1) { get6pos = readpysrc.get_values_from_python(); }
            else if (checkfilepysrc == 2) { get6pos = readpysrc.get_values_from_exe(); }
            else if (checkfilepysrc == 0) { goto end; }
            if (rbpos.size() < get6pos.size()) {
                coumove = get6pos.size();
                rbpos.resize(get6pos.size(), std::vector<float>(6, 0.0f));
            }

            for (size_t i = 0; i < get6pos.size(); ++i) {
                for (size_t j = 0; j < 6; ++j) {
                    rbpos[i][j] = get6pos[i][j];
                }
            }
        end:
            ;
        }
        ImGui::SameLine(); // Check the trajectory
        if (ImGui::Button("Clear Trajectory"))
        {
            *tpstatus = controller->MotionManager->ClearAllTrajectory();
            *tpstatus = controller->MotionManager->ClearGroupTrajectory(ControlGroupId::R1);
            *sttlogs << "trying to clear trajectory";
        }
        if (ImGui::Button("Home"))  // Check the trajectory
        {
            *tpstatus = controller->Variables->BasePositionVariable->Read(0, *b1PositionData);
            *tpstatus = controller->Kinematics->ConvertPosition(ControlGroupId::R1, b1PositionData->positionData, KinematicConversions::PulseToCartesianPos, *b1origi);
            b1origi->coordinateType = CoordinateType::RobotCoordinate;
            JointMotion r1home(ControlGroupId::R1, *b1origi, 3);
            *tpstatus = controller->MotionManager->AddPointToTrajectory(r1home);
            *tpstatus = controller->ControlCommands->SetServos(SignalStatus::ON);
            *tpstatus = controller->MotionManager->MotionStart();
            *sttlogs << "Go back to the Home pos";
        }
        ImGui::SameLine();
        ImGui::Text("Linear spd:"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
        ImGui::InputFloat("##LS", &spdlinear, 0.0f, 0.0f, "%.2f");  ImGui::SameLine();
        ImGui::Text("Joint spd:"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
        ImGui::InputFloat("##JS", &spdjoint, 0.0f, 0.0f, "%.2f");
        ImGui::Separator();
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
                *sttlogs << "Update the Original Position for " << std::to_string(j);
            }
            ImGui::SameLine();
            if (ImGui::Button(("CrtPos " + std::to_string(j)).c_str())) {
                for (int i = 0; i < 6; ++i) {
                    *tpstatus = controller->ControlGroup->ReadPositionData(ControlGroupId::R1, CoordinateType::BaseCoordinate, 0, 0, *b1crpos);
                    rbpos[j][i] = b1crpos->axisData[i];
                }
                *sttlogs << "Update the Current Position for " + std::to_string(j);

            }
            ImGui::SameLine();
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

        double Linepathelapsed_seconds = std::chrono::duration_cast<std::chrono::duration<double>>
            (std::chrono::high_resolution_clock::now() - LinepathStart).count();


        if (lineshpath && Linepathelapsed_seconds > 0.4f) {
            if (lineshpath && Linepathelapsed_seconds > 0.2) {
                LinepathStart = std::chrono::high_resolution_clock::now();

                std::shared_ptr<nelems::oMesh> newmesh = std::make_shared<nelems::oMesh>();
                if (!proMeshRb->get_mesh_byname("movepath__SKIP__", newmesh)) {
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
                for (int i = 0; i < coumove; ++i) {
                    vertex.mPos = glm::vec3(rbpos[i][0], rbpos[i][1], rbpos[i][2] + 138.845);
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
        }

        auto execute_move = [&](auto&& motion_functor) {
            std::string stent = "Start the Motion to " + std::to_string(coumove) + " points: ";
            for (int j = 0; j < coumove; ++j) {
                b1crpos->coordinateType = CoordinateType::RobotCoordinate;
                stent += "\n" + std::to_string(j) + ": ";
                for (int i = 0; i < 6; ++i) {
                    b1crpos->axisData[i] = rbpos[j][i];
                    stent += std::to_string(rbpos[j][i]) + " ";
                }
                auto motion = motion_functor(ControlGroupId::R1, *b1crpos, (std::is_same_v<decltype(motion_functor), JointMotionFunctor>) ? spdjoint : spdlinear);
                *tpstatus = controller->MotionManager->AddPointToTrajectory(motion);
            }
            switchVisualizeMode = true;
            *tpstatus = controller->ControlCommands->SetServos(SignalStatus::ON);
            *tpstatus = controller->MotionManager->MotionStart();
           
            *sttlogs << stent;
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
            *sttlogs << "Start the Circular Motion";
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