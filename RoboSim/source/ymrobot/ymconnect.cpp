#include "pch.h"
#include "ymconnect.h"
#include "windows.h"
namespace nymrobot
{
    //bool ymconnect::connect_trigger = false;
    void ymconnect::connect_robot()
    {
        static char ip_address[20] = "192.168.10.111"; // Default IP address

        ImGui::SetNextWindowPos(ImVec2(300, 300));
        ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.0f, 0.3f, 0.2f, 1.0f));
        ImGui::Begin("GetIP", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoNav);
        ImGui::SetWindowSize(ImVec2(400, 100));

        ImGui::InputText("IP Address", ip_address, sizeof(ip_address));
        static char connect_content[100] = "connecting to the Robot...";
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
        if (status.StatusCode == true)
        { moreUIRB(); }

        ImGui::End();
        ImGui::PopStyleColor();
    }

    void ymconnect::moreUIRB()
    {
        if (ImGui::Button("move robot")) { call_move = true; }
        if (ImGui::Button("read robot")) { call_read = true; }
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
        if (connect_trigger) { connect_robot(); }
        if (call_move)       { move_robot(); }
		if (call_read)       { read_robot(); }
    }
    void ymconnect::move_robot()
    {
        if (status.StatusCode != 0) { MessageBox(NULL, "Disconnected to the ROBOT!", "ERROR", MB_OK); return; }

        static float mover_x = 1, mover_y=1, mover_z=1;
        ImGui::SliderFloat("x", &mover_x, 0.01f, 100.0f);
        ImGui::SliderFloat("y", &mover_y, 0.01f, 100.0f);
        ImGui::SliderFloat("z", &mover_z, 0.01f, 100.0f);
        if (ImGui::Button("test_mode 1")) 
        {
            RobotPositionVariableData r1PositionData{};
            BaseAxisPositionVariableData b1PositionData{};

            status = controller->Variables->BasePositionVariable->Read(0, b1PositionData);
            status = controller->Variables->RobotPositionVariable->Read(0, r1PositionData);

            LinearMotion r1Motion(ControlGroupId::R1, r1PositionData.positionData, 25.0);
            JointMotion b1Motion(ControlGroupId::B1, b1PositionData.positionData, 25.0);
                
            status = controller->MotionManager->AddPointToTrajectory(r1Motion, b1Motion);

            status = controller->ControlCommands->SetServos(SignalStatus::ON);

            status = controller->MotionManager->MotionStart();

            std::cout << status << std::endl;
        }
        if (ImGui::Button("test_mode 2")) 
        {
            // Create a PositionData object with the desired target coordinates
            PositionData targetPosition(CoordinateType::RobotCoordinate, Figure(), 0, 0, { mover_x, mover_y, mover_z, 0.0, 0.0, 0.0, 0.0, 0.0 });

            // Create a LinearMotion target with the target position
            LinearMotion motionTarget(ControlGroupId::R1, targetPosition, 100.0);  // Assuming 100 mm/s speed

            // Add the motion target to the trajectory
            if (controller) {
                status = controller->MotionManager->AddPointToTrajectory(motionTarget);
                if (status.IsOk()) {
                    status = controller->MotionManager->MotionStart();
                    if (!status.IsOk()) {
                        std::cerr << "Error starting motion: " << status.Message << std::endl;
                    }
                }
                else {
                    std::cerr << "Error adding point to trajectory: " << status.Message << std::endl;
                }
            }
        }
        if (ImGui::Button("Collapse Move"))
        {
			call_move = false;
		}
        
    }
    void ymconnect::read_robot()
    {
        if (status.StatusCode != 0) { MessageBox(NULL, "Disconnected to the ROBOT!", "ERROR", MB_OK); return; }
        PositionData positionData{};
        status = controller->ControlGroup->ReadPositionData(ControlGroupId::R1, CoordinateType::Pulse, 0, 0, positionData);

        std::stringstream ss;
        ss << status << ". \n" << positionData;
        std::string message = ss.str();
        ImGui::Text(message.c_str());
        if (ImGui::Button("Collapse Read"))
        {
    		call_read = false;
        }
    }
}