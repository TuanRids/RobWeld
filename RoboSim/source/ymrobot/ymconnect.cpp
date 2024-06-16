#include "pch.h"
#include "ymconnect.h"
#include "windows.h"
#include <chrono>
#include "elems/vertex_holder.h"

namespace nymrobot
{
    //bool ymconnect::connect_trigger = false;
    void ymconnect::connect_robot()
    {
        
        static char ip_address[20] = "192.168.10.102"; // Default IP address
        static char connect_content[100] = "Welcome to OhLabs";

        // set UI
        ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.0f, 0.3f, 0.2f, 1.0f));
        ImGui::Begin("GetIP", nullptr,  ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize );
        ImGui::SetWindowSize(ImVec2(400, 100));
        ImGui::SetNextItemWidth(150);

        ImGui::InputText("IP Address", ip_address, sizeof(ip_address)); ImGui::SetNextItemWidth(150);
        
        if (status.StatusCode == 0) { strcpy_s(connect_content, "Connected."); }
        ImGui::InputText("Status", connect_content, sizeof(connect_content));

        if (ImGui::Button("Connect") && status.StatusCode != 0) {
            std::string getip{ ip_address };
            
            // set content to make sure it is updated for 2nd time of connection
            strcpy_s(connect_content, "Welcome to OhLabs.");
            controller = YMConnect::OpenConnection(getip, status); // Open a connection to the robot controller

            if (status.StatusCode != 0) {
                std::stringstream ss;
                ss << status;
                std::string message = ss.str();
                MessageBox(NULL, message.c_str(), "Error", MB_OK);
            }
            else 
            {
                status = controller->ControlCommands->DisplayStringToPendant(connect_content);
            }
        }

        ImGui::SameLine();
        if (ImGui::Button("Disconnect")) {
            status = controller->ControlCommands->DisplayStringToPendant((char*)"Disconnect!");
            disconnect_robot(false);
            strcpy_s(connect_content, "Disconnect!");
            
        }

        ImGui::Separator();
        ImGui::End();
        ImGui::PopStyleColor();
    }

    void ymconnect::disconnect_robot(bool showmsg)
    {
        if (status.StatusCode == 0)
        { 
            YMConnect::CloseConnection(controller);
            YMConnect::OpenConnection("192.168.0.0", status); // Fake Login for destroy status
            controller = nullptr;
        }
        if (showmsg)
        { ImGui::OpenPopup("Disconnected"); }

    }
    void ymconnect::render()
    {
        // =========================================================================================================
        // get ptr to the robot mesh
        if (!proMeshRb)
        {
            proMeshRb = &nelems::mMesh::getInstance();
        }

        // if trigger is true, show the UI to connect to the robot
        connect_robot(); 
        if (status.StatusCode != 0 || controller->Status == NULL) { return; }
        
        // UI for controlling the Robot
        ImGui::Begin("Robot Control");
        if (ImGui::Button("Clear Fault"))
        {
            StatusInfo* temptstt = new StatusInfo();
            *temptstt = controller->Faults->ClearAllFaults();
            if (temptstt->StatusCode == 0) { resultmsg.str(" "); }
            delete temptstt;
        }

        if (call_move) { move_robot(); }
        read_robot();
        ImGui::Separator();

        if (resultmsg) { ImGui::Text(resultmsg.str().c_str()); }
		ImGui::End();
    }
    void ymconnect::move_robot() {
        std::unique_ptr<StatusInfo> tpstatus = std::make_unique<StatusInfo>();
        ImGui::Separator();
        ImGui::Begin("Attributes: ");

        BaseAxisPositionVariableData* b1PositionData = new BaseAxisPositionVariableData();
        PositionData* b1origi = new PositionData();
        PositionData* b1crpos = new PositionData();

        *tpstatus = controller->Variables->BasePositionVariable->Read(0, *b1PositionData);
        *tpstatus = controller->Kinematics->ConvertPosition(ControlGroupId::R1, b1PositionData->positionData, KinematicConversions::PulseToCartesianPos, *b1origi);
        *tpstatus = controller->ControlGroup->ReadPositionData(ControlGroupId::R1, CoordinateType::BaseCoordinate, 0, 0, *b1crpos);

        static std::vector<std::vector<float>> rbpos(3, std::vector<float>(6, 0.0f));
        static std::vector<float> spdlinear{};
        static std::vector<float> spdjoint{};
        static int coumove{ 3 };

        ImGui::SetNextItemWidth(150);
        ImGui::InputInt("Times", &coumove, 1, 2); ImGui::SameLine();
        if (ImGui::Button("Loadpy"))
        {
            std::vector<std::vector<double>> get6pos = readpysrc.get_values_from_python();
            for (int i{ 0 }; i < get6pos.size(); i++)
            {
				rbpos[i][0] = get6pos[i][0];
				rbpos[i][1] = get6pos[i][1];
				rbpos[i][2] = get6pos[i][2];
				rbpos[i][3] = get6pos[i][3];
				rbpos[i][4] = get6pos[i][4];
				rbpos[i][5] = get6pos[i][5];
            }
        }
        spdlinear.resize(coumove);
        spdjoint.resize(coumove);

        for (int j = 0; j < coumove; j++) {
            if (rbpos.size() < coumove) {
                int numToAdd = coumove - rbpos.size();
                for (int i = 0; i < numToAdd; ++i) {
                    rbpos.push_back(std::vector<float>(6, 0.0f));
                }
            }
            if (ImGui::Button(("OrgPos " + std::to_string(j)).c_str())) { for (int i = 0; i < 6; i++) { rbpos[j][i] = b1origi->axisData[i]; } }
            ImGui::SameLine();
            if (ImGui::Button(("CrtPos " + std::to_string(j)).c_str())) { for (int i = 0; i < 6; i++) { rbpos[j][i] = b1crpos->axisData[i]; } }
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
            ImGui::InputFloat(("##RZ" + std::to_string(j)).c_str(), &rbpos[j][5], 0.0f, 0.0f, "%.2f"); ImGui::SameLine();
            ImGui::Text("Linear spd:"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
            ImGui::InputFloat(("##LS" + std::to_string(j)).c_str(), &spdlinear[j], 0.0f, 0.0f, "%.2f");  ImGui::SameLine();
            ImGui::Text("Joint spd:"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
            ImGui::InputFloat(("##JS" + std::to_string(j)).c_str(), &spdjoint[j], 0.0f, 0.0f, "%.2f");

            if (spdlinear[j] <= 0 || spdlinear[j] > 100) { spdlinear[j] = 10; }
            if (spdjoint[j] <= 0 || spdjoint[j] > 1) { spdjoint[j] = 0.95; }
        }

        if (ImGui::Button("LinearPath !NOT AVAILABLE"))
        {
           /* nelems::oMesh linepath;
            for (int j = 0; j < coumove; j++) {
                nelems::VertexHolder vertex;
                vertex.mPos = { rbpos[j][0],rbpos[j][1],rbpos[j][2] };
                vertex.mNormal = { 0.0f, 0.0f, 1.0f };
                linepath.add_vertex(vertex);
            }
            linepath.changeName("LINEPATH");
            proMeshRb->pushback(linepath);*/
        }

        if (ImGui::Button("Joint Move")) {
            ImGui::SetTooltip("Should Use!");
            for (int j = 0; j < coumove; j++) {
                b1crpos->coordinateType = CoordinateType::RobotCoordinate;
                for (int i = 0; i < 6; i++) {
                    b1crpos->axisData[i] = rbpos[j][i];
                }
                JointMotion r1movel(ControlGroupId::R1, *b1crpos, spdjoint[j]);
                *tpstatus = controller->MotionManager->AddPointToTrajectory(r1movel);
            }

            switchVisualizeMode = true;
            *tpstatus = controller->ControlCommands->SetServos(SignalStatus::ON);
            *tpstatus = controller->MotionManager->MotionStart();
        }
        ImGui::SameLine();
        if (ImGui::Button("Linear Move")) {
            ImGui::SetTooltip("Dont use if you dont know what you are doing !");
            for (int j = 0; j < coumove; j++) {
                b1crpos->coordinateType = CoordinateType::RobotCoordinate;
                for (int i = 0; i < 6; i++) {
                    b1crpos->axisData[i] = rbpos[j][i];
                }
                LinearMotion r1movel(ControlGroupId::R1, *b1crpos, spdlinear[j]);
                *tpstatus = controller->MotionManager->AddPointToTrajectory(r1movel);
            }
            switchVisualizeMode = true;
            *tpstatus = controller->ControlCommands->SetServos(SignalStatus::ON);
            *tpstatus = controller->MotionManager->MotionStart();
        }
        ImGui::SameLine();
        if (ImGui::Button("Circular Move")) {
            ImGui::SetTooltip("current to 1st point = Linear, 1 -> 3 is circular based on 2nd points to calculate the curve. !");
            // Linear from current to 1st points
            b1crpos->coordinateType = CoordinateType::RobotCoordinate;
            for (int i = 0; i < 6; i++) {
                b1crpos->axisData[i] = rbpos[0][i];
            }
            JointMotion r1movel(ControlGroupId::R1, *b1crpos, spdjoint[0]);
            *tpstatus = controller->MotionManager->AddPointToTrajectory(r1movel);
            // Circular 0.1.2, 2.3.4
            for (int j = 2; j < coumove; j+=2) {
                b1crpos->coordinateType = CoordinateType::RobotCoordinate;
                CoordinateArray coorarr;
                for (int i = 0; i < 6; i++) {
                    b1crpos->axisData[i] = rbpos[j][i];
                    coorarr[i] = rbpos[j - 1][i];
                }
                CircularMotion r1movel(ControlGroupId::R1, *b1crpos, coorarr, spdlinear[j]);
                *tpstatus = controller->MotionManager->AddPointToTrajectory(r1movel);
            }
            switchVisualizeMode = true;
            *tpstatus = controller->ControlCommands->SetServos(SignalStatus::ON);
            *tpstatus = controller->MotionManager->MotionStart();
        }

        delete b1PositionData;
        delete b1crpos;
        delete b1origi;

        ImGui::End();
    }


    void ymconnect::read_robot()
    {
        // Setup for reading status
        ImGui::Separator();
        std::stringstream* strget = new std::stringstream();
        StatusInfo tpstatus;

        // read state
        ControllerStateData stateData{};
        controller->Status->ReadState(stateData);
        *strget << stateData;
        ImGui::Text("%s", strget->str().c_str());

        // read the position
        PositionData raxisData{}, rposData{}, rjointangle{};
        tpstatus = controller->ControlGroup->ReadPositionData(ControlGroupId::R1, CoordinateType::BaseCoordinate, 0, 0, rposData);
        *strget << rposData;
        std::string substr = strget->str().substr(strget->str().find("Axes"));

        if (stateData.isRunning)
        {
            ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "%s", substr.c_str()); //red color
        } 
        else
        {
            ImGui::Text("%s", substr.c_str());
        }
        if (stateData.isAlarming)
        {
            AlarmHistory alarmHistoryData;
            controller->Faults->GetAlarmHistory(AlarmCategory::Minor, 3, alarmHistoryData);
            resultmsg.str();
            resultmsg << alarmHistoryData << std::endl;
            std::cout << alarmHistoryData << std::endl;
        }

        // read the Joint of robot
        tpstatus = controller->ControlGroup->ReadPositionData(ControlGroupId::R1, CoordinateType::Pulse, 0, 0, raxisData);
        tpstatus = controller->Kinematics->ConvertPosition(ControlGroupId::R1, raxisData
            , KinematicConversions::PulseToJointAngle, rjointangle);
        angle1 = rjointangle.axisData[0];
        angle2 = rjointangle.axisData[1];
        angle3 = rjointangle.axisData[2];
        angle4 = rjointangle.axisData[3];
        angle5 = rjointangle.axisData[4];
        angle6 = rjointangle.axisData[5];
        delete strget;
        
        }
      


    
}