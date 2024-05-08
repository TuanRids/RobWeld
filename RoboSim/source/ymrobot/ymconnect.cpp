#include "pch.h"
#include "ymconnect.h"
#include "windows.h"
namespace nymrobot
{
    bool ymconnect::connect_trigger = false;
    void ymconnect::connect_robot()
    {
        static char ip_address[20] = "192.168.10.111"; // Default IP address

        ImGui::SetNextWindowPos(ImVec2(300, 300));
        ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.0f, 0.3f, 0.2f, 1.0f));
        ImGui::Begin("GetIP", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoNav);
        ImGui::SetWindowSize(ImVec2(400, 100));

        ImGui::InputText("IP Address", ip_address, sizeof(ip_address));
        static char connect_content[100] = "Connected. Welcome";
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
        ImGui::End();
        ImGui::PopStyleColor();
    }
    void ymconnect::disconnect_robot(bool showmsg)
    {
        if (status.StatusCode == 0) { YMConnect::CloseConnection(controller.get()); }
        if (showmsg) { ImGui::OpenPopup("Disconnected"); }

    }
    void ymconnect::render()
    {
        // if trigger is true, show the UI to connect to the robot
        if (connect_trigger)
        { connect_robot(); }
        if (status.StatusCode != 0 || controller->Status == NULL) { return; }
        move_robot();
        //read_robot();
    }
    void ymconnect::move_robot()
    {
        
        ImGui::Begin("Move Robot");
        static float mover_x = 1, mover_y=1, mover_z=1;
        ImGui::SliderFloat("x", &mover_x, 0.01f, 100.0f);
        ImGui::SliderFloat("y", &mover_y, 0.01f, 100.0f);
        ImGui::SliderFloat("z", &mover_z, 0.01f, 100.0f);
        if (ImGui::Button("MOVE")) {
            bool te = true;
            if (te==true)
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
            else
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
        }
        ImGui::End();
        
    }
    void ymconnect::read_robot()
    {
        
        PositionData positionData{};
        status = controller->ControlGroup->ReadPositionData(ControlGroupId::R1, CoordinateType::Pulse, 0, 0, positionData);

        std::stringstream ss;
        ss << status << positionData;
        std::string message = ss.str();
        MessageBox(NULL, message.c_str(), "Information", MB_OK);
    }
}