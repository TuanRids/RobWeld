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
        if (connect_trigger)
        { connect_robot(); }
        if (status.StatusCode != 0 || controller->Status == NULL) { return; }
        move_robot();
        read_robot();
    }
    void ymconnect::move_robot()
    {
        if (status.StatusCode != 0) { MessageBox(NULL, "Disconnected to the ROBOT!", "ERROR", MB_OK); return; }

        static float mover_x = 1, mover_y=1, mover_z=1;
        ImGui::SliderFloat("x", &mover_x, 0.01f, 100.0f);
        ImGui::SliderFloat("Y", &mover_y, 0.01f, 100.0f);
        ImGui::SliderFloat("Z", &mover_z, 0.01f, 100.0f);
        if (ImGui::Button("MOVE")) {
            PositionData targetPosition(CoordinateType::RobotCoordinate, Figure(), 0, 0, { mover_x, mover_y, mover_z, 0.0, 0.0, 0.0, 0.0, 0.0 });
            LinearMotion motionTarget(ControlGroupId::R1, targetPosition, 100.0);
            if (controller) {
                // connect to MotionManager
                // MotionManagerInterface::AddPointToTrajectory(motionTarget);
                // add the point to the trajectory
                status = controller->MotionManager->AddPointToTrajectory(motionTarget);
                if (status.IsOk()) {
                    status = controller->MotionManager->MotionStart();
                    if (!status.IsOk()) {
                        std::cerr << "Error starting motion: " << status.Message << std::endl;
                    }
                }
                else {
                    std::cerr << "Error to connect with MotionManager: " << status.Message << std::endl;
                }
            }
        }
        ImGui::End();
    }
    void ymconnect::read_robot()
    {
       
        RobotPositionVariableData robotPositionVariableData{};
        status = controller->Variables->RobotPositionVariable->Read(1, robotPositionVariableData);

        std::stringstream ss;
        ss << status << robotPositionVariableData;
        std::string message = ss.str();
        ImGui::Text(message.c_str());
        if (ImGui::Button("Collapse Read"))
        {
    		call_read = false;
        }
    }
}