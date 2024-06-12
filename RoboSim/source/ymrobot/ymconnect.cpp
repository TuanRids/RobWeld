#include "pch.h"
#include "ymconnect.h"
#include "windows.h"
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
            YMConnect::OpenConnection("192.0.0.0", status, restime); // Fake Login for destroy status
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
        //  =========================================================================================================

        // if trigger is true, show the UI to connect to the robot
        if (connect_trigger) { connect_robot(); }
        // if connected, expand the UI for working with the Robot
        if (status.StatusCode != 0 || controller->Status == NULL) { return; }
        
        // UI for controlling the Robot
        ImGui::Begin("Robot Control", &connect_trigger);

        if (ImGui::Button("Clear Fault"))
        {
            StatusInfo* temptstt = new StatusInfo();
            *temptstt = controller->Faults->ClearAllFaults();
            if (temptstt->StatusCode == 0) { resultmsg.str(" "); }
            delete temptstt;

        }

        move_robot();
        read_robot();
        ImGui::Separator();
        if (resultmsg) { ImGui::Text(resultmsg.str().c_str()); }

		ImGui::End();
    }
    void ymconnect::move_robot()
    {
        std::unique_ptr<StatusInfo> tpstatus = std::make_unique<StatusInfo>();
        ImGui::Separator();

        // Bx by bz for movement
        static float bx{ 0 }, by{ 0 }, bz{ 0 }, brx{ 0 }, bry{ 0 }, brz{ 0 }, spdlinear{ 5 }, spdjoint{ 0.7 };

        ImGui::SetNextItemWidth(50);
        ImGui::InputFloat("X ", &bx, 0.0f, 0.0f, "%.2f"); ImGui::SetNextItemWidth(50); ImGui::SameLine();
        ImGui::InputFloat("Y ", &by, 0.0f, 0.0f, "%.2f"); ImGui::SetNextItemWidth(50); ImGui::SameLine();
        ImGui::InputFloat("Z ", &bz, 0.0f, 0.0f, "%.2f"); ImGui::SetNextItemWidth(50);
        ImGui::InputFloat("Rx", &brx, 0.0f, 0.0f, "%.2f"); ImGui::SetNextItemWidth(50); ImGui::SameLine();
        ImGui::InputFloat("Ry", &bry, 0.0f, 0.0f, "%.2f"); ImGui::SetNextItemWidth(50); ImGui::SameLine();
        ImGui::InputFloat("Rz", &brz, 0.0f, 0.0f, "%.2f"); ImGui::SetNextItemWidth(50);
        ImGui::InputFloat("Linear Speed", &spdlinear, 0.0f, 0.0f, "%.2f"); ImGui::SetNextItemWidth(50);
        ImGui::InputFloat("Joint Speed",  &spdjoint,  0.0f, 0.0f, "%.2f");
        if (spdlinear <= 0 || spdlinear > 100) { spdlinear = 10; }
        if (spdjoint <= 0 || spdjoint > 1)     { spdjoint = 0.7; }
        if (ImGui::Button("B6 Linear Move [!!!!]")) {
            BaseAxisPositionVariableData* b1PositionData = new BaseAxisPositionVariableData();
            PositionData* b1posconvert = new PositionData();

            *tpstatus = controller->Variables->BasePositionVariable->Read(0, *b1PositionData);
            *tpstatus = controller->Kinematics->ConvertPosition(ControlGroupId::R1, b1PositionData->positionData, KinematicConversions::PulseToCartesianPos, *b1posconvert);

            if (bx != 0.0f) { b1posconvert->axisData[0] += bx; }
            if (by != 0.0f) { b1posconvert->axisData[1] += by; }
            if (bz != 0.0f) { b1posconvert->axisData[2] += bz; }
            if (brx != 0.0f) { b1posconvert->axisData[3] += brx; }
            if (bry != 0.0f) { b1posconvert->axisData[4] += bry; }
            if (brz != 0.0f) { b1posconvert->axisData[5] += brz; }

            LinearMotion r1movel(ControlGroupId::R1, *b1posconvert, spdlinear);
            // check SLURBT neu can thi giam toc.

            *tpstatus = controller->MotionManager->AddPointToTrajectory(r1movel);
            *tpstatus = controller->ControlCommands->SetServos(SignalStatus::ON);
            *tpstatus = controller->MotionManager->MotionStart();

            // delete ptr
            delete b1PositionData;
            delete b1posconvert;
        }
        ImGui::SameLine();
        if (ImGui::Button("B6 Joint Move")) {
            // use dynamic ptr for avoiding using too much memory on stack
            RobotPositionVariableData* r1pos = new RobotPositionVariableData();
            PositionData* r1poscv = new PositionData();

            *tpstatus = controller->Variables->RobotPositionVariable->Read(0, *r1pos);
            *tpstatus = controller->Kinematics->ConvertPosition(ControlGroupId::R1, r1pos->positionData, KinematicConversions::PulseToCartesianPos, *r1poscv);

            if (bx != 0.0f) { r1poscv->axisData[0] += bx; }
            if (by != 0.0f) { r1poscv->axisData[1] += by; }
            if (bz != 0.0f) { r1poscv->axisData[2] += bz; }
            if (brx != 0.0f) { r1poscv->axisData[3] += brx; }
            if (bry != 0.0f) { r1poscv->axisData[4] += bry; }
            if (brz != 0.0f) { r1poscv->axisData[5] += brz; }

            JointMotion r1movej(ControlGroupId::R1,*r1poscv, spdjoint);
            *tpstatus = controller->MotionManager->AddPointToTrajectory(r1movej);
            *tpstatus = controller->ControlCommands->SetServos(SignalStatus::ON);
            *tpstatus = controller->MotionManager->MotionStart();

            // delete ptr
            delete r1pos;
            delete r1poscv;
        }
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
        }

        // read the Joint of robot
        tpstatus = controller->ControlGroup->ReadPositionData(ControlGroupId::R1, CoordinateType::Pulse, 0, 0, raxisData);
        tpstatus = controller->Kinematics->ConvertPosition(ControlGroupId::R1, raxisData
            , KinematicConversions::PulseToJointAngle, rjointangle);
        angle1 = -rjointangle.axisData[0];
        angle2 = rjointangle.axisData[1];
        angle3 = -rjointangle.axisData[2];
        angle4 = rjointangle.axisData[3];
        angle5 = -rjointangle.axisData[4];
        angle6 = rjointangle.axisData[5];
        delete strget;
        
        }
      


    
}