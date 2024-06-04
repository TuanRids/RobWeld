#include "pch.h"
#include "ymconnect.h"
#include "windows.h"
namespace nymrobot
{
    //bool ymconnect::connect_trigger = false;
    void ymconnect::connect_robot()
    {
        static char ip_address[20] = "192.168.10.102"; // Default IP address

        ImGui::SetNextWindowPos(ImVec2(300, 300));
        ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.0f, 0.3f, 0.2f, 1.0f));
        ImGui::Begin("GetIP", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoNav);
        ImGui::SetWindowSize(ImVec2(400, 100));

        ImGui::InputText("IP Address", ip_address, sizeof(ip_address));
        static char connect_content[100] = "Check Pendant Message to Connect...";
        if (status.StatusCode == 0) { strcpy_s(connect_content, "Connected."); }
        ImGui::InputText("Status", connect_content, sizeof(connect_content));

        if (ImGui::Button("Connect")) {
            std::string getip{ ip_address };
            MotomanController* rawController = YMConnect::OpenConnection(getip, status); // Open a connection to the robot controller

            if (status.StatusCode != 0) {
                std::stringstream ss;
                ss << status;
                std::string message = ss.str();
                MessageBox(NULL, message.c_str(), "Error", MB_OK);
            }
            else {
                controller.reset(rawController); // Convert raw pointer to shared_ptr
                status = controller->ControlCommands->DisplayStringToPendant(connect_content);
                MessageBox(NULL, "Connected", "Success", MB_OK);
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Close")) {
            ImGui::CloseCurrentPopup();
            connect_trigger = false;
        }
        ImGui::Separator();
        

        ImGui::End();
        ImGui::PopStyleColor();
    }

    void ymconnect::disconnect_robot(bool showmsg)
    {
        std::cout<< status.StatusCode << std::endl;
        if (status.StatusCode == 0)
        { YMConnect::CloseConnection(controller.get()); }
        if (showmsg)
        { ImGui::OpenPopup("Disconnected"); }

    }
    void ymconnect::render()
    {
        // if trigger is true, show the UI to connect to the robot
        if (connect_trigger)
        { connect_robot(); }
        // if connected, expand the UI for working with the Robot
        if (status.StatusCode != 0 || controller->Status == NULL) { return; }
		ImGui::Begin("Robot Control", &connect_trigger);
        move_robot();
        read_robot();
		ImGui::End();
    }
    void ymconnect::move_robot()
    {
        StatusInfo tpstatus;        std::stringstream ss;
        ImGui::Separator();
        /*static float mover_x = 0.1, mover_y=0.1, mover_z=0.1;     ImGui::SetNextItemWidth(100);
        ImGui::InputFloat("X", &mover_x, 0.01f, 100.0f);   ImGui::SetNextItemWidth(100); 
        ImGui::InputFloat("Y", &mover_y, 0.01f, 100.0f);   ImGui::SetNextItemWidth(100); 
        ImGui::InputFloat("Z", &mover_z, 0.01f, 100.0f);*/

        if (ImGui::Button("Move")) {
			RobotPositionVariableData r1PositionData{}; r1PositionData.positionData.coordinateType = CoordinateType::RobotCoordinate;
			//BaseAxisPositionVariableData b1PositionData{}; b1PositionData.positionData.coordinateType = CoordinateType::RobotCoordinate;
            //tpstatus = controller->Variables->BasePositionVariable->Read(1, b1PositionData);
            tpstatus = controller->Variables->RobotPositionVariable->Read(1, r1PositionData);

            LinearMotion r1Motion(ControlGroupId::R1, r1PositionData.positionData, 25.0);
            //JointMotion b1Motion(ControlGroupId::B1, b1PositionData.positionData, 25.0);
			std::cout << r1Motion.position << std::endl;

            tpstatus = controller->MotionManager->AddPointToTrajectory(r1Motion);

            tpstatus = controller->ControlCommands->SetServos(SignalStatus::ON);

            tpstatus = controller->MotionManager->MotionStart();

            /*
            PositionData crposition{};
            tpstatus = controller->ControlGroup->ReadPositionData(ControlGroupId::R1, CoordinateType::RobotCoordinate, 0, 0, crposition);
            PositionData destination;
			destination.coordinateType = CoordinateType::RobotCoordinate;
            //Destination position.
			destination.axisData = { crposition.axisData[0] + mover_x, crposition.axisData[1] + mover_y, crposition.axisData[2] + mover_z, crposition.axisData[3], crposition.axisData[4], crposition.axisData[5] };
            //Set coordinate system to RobotCoordinate.
            destination.coordinateType = CoordinateType::RobotCoordinate;

            //A motion with the default motion attributes using the position data in destination.
            //Speed of 25.0 mm/s. ControlGroupId R1.
            LinearMotion r1LinearMotion(ControlGroupId::R1, destination, 25);

            tpstatus = controller->MotionManager->AddPointToTrajectory(r1LinearMotion);
            tpstatus = controller->MotionManager->MotionStart();*/
            if (tpstatus.StatusCode!=0)
            {
                AlarmHistory alarmHistoryData;
                controller->Faults->GetAlarmHistory(AlarmCategory::Minor, 3, alarmHistoryData);
				std::cout << alarmHistoryData << std::endl;
            }
        }
    }
    void ymconnect::read_robot()
    {
        ImGui::Separator();
        std::stringstream ss;
        StatusInfo tpstatus;
        if (ImGui::Button("Read Status")) {
            ControllerStateData stateData{};
            controller->Status->ReadState(stateData);
			//TODO: Create getstring function for status
            ss << stateData;
			MessageBox(NULL, ss.str().c_str(), "Read Status", MB_OK);
        }
        if (ImGui::Button("Read Pos"))
        {
			PositionData positionData{};
            // Position of robot
            tpstatus = controller->ControlGroup->ReadPositionData(ControlGroupId::R1, CoordinateType::BaseCoordinate, 0, 0, positionData);
            ss << formatNumber(positionData) << "\n~~~DONE~~~\n";

            /// Rotation of joint angles
            for (int i = 11; i <= 18; ++i) {
                ControlGroupId groupId = static_cast<ControlGroupId>(i);
                tpstatus = controller->ControlGroup->ReadPositionData(groupId, CoordinateType::BaseCoordinate, 0, 0, positionData);
                ss << formatNumber(positionData);
            }

			MessageBox(NULL, ss.str().c_str(), "Robot Pos", MB_OK);


        }

    }
    std::string ymconnect::formatNumber( PositionData positionData) {
        std::ostringstream oss;
        std::string name[8] = { "X: ", "Y: ", "Z: ", "Rx: ", "Ry: ", "Rz: ", "Re: ", "Rw: "};
		for (int i{ 0 }; i < 8; i++) {
            if (i<3)
            {
                oss << "[" << name[i] << std::setw(6) << std::setprecision(3) << std::fixed << positionData.axisData[i] << "] - ";
			}
			else if (i==3)
			{
				oss << "\n[" << name[i] << std::setw(6) << std::setprecision(3) << std::fixed << positionData.axisData[i] << "] - ";
			}
            else
            {
				oss << "[" << name[i] << std::setw(6) << std::setprecision(3) << std::fixed << positionData.axisData[i] << "] ";
            }
		}
		oss << "\n";
        return oss.str();
    }

}