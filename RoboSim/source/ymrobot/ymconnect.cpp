#include "pch.h"
#include "ymconnect.h"
#include "windows.h"
#include <chrono>
#include "elems/vertex_holder.h"
#include <filesystem>
#include <future>
#include <chrono>

namespace fs = std::filesystem;

namespace nymrobot {

    std::vector<std::vector<float>> ymconnect::get6pos(0, std::vector<float>(6, 0.0f));
    char ymconnect::connect_content[100] = "Welcome";
    UIState ymconnect::ui_state{};
    ymconnect::~ymconnect() {
        if (controller) {
            disconnect_robot(false);
        }
        delete controller;
    }
    bool isNetworkAvailable(const std::string& ip) {
        std::string command = "cmd.exe /c ping -n 1 " + ip + " > nul 2>&1";

        // Setup the process attributes
        STARTUPINFO si;
        PROCESS_INFORMATION pi;
        ZeroMemory(&si, sizeof(si));
        si.cb = sizeof(si);
        si.dwFlags = STARTF_USESHOWWINDOW;
        si.wShowWindow = SW_HIDE;
        ZeroMemory(&pi, sizeof(pi));

        // Create the process
        if (CreateProcess(NULL, const_cast<char*>(command.c_str()), NULL, NULL, FALSE, CREATE_NO_WINDOW, NULL, NULL, &si, &pi)) {
            // Wait for the process to finish
            WaitForSingleObject(pi.hProcess, INFINITE);

            // Get the exit code
            DWORD exitCode;
            GetExitCodeProcess(pi.hProcess, &exitCode);

            // Close process and thread handles
            CloseHandle(pi.hProcess);
            CloseHandle(pi.hThread);

            return (exitCode == 0);
        }
        else {
            std::cerr << "Failed to create process: " << GetLastError() << std::endl;
            return false;
        }
    }

    void ymconnect::connect_robot() {
        static char ip_address[64] = "192.168.10.102";
        std::string temp_tcp;
        robinit->get_settings("robot_tcp", temp_tcp);
        strncpy_s(ip_address, temp_tcp.c_str(), sizeof(ip_address));

        ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.0f, 0.3f, 0.2f, 1.0f));
        ImGui::Begin("GetIP", nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize);
        //ImGui::SetWindowSize(ImVec2(400, 100));
        ImGui::SetNextItemWidth(120);
        ImGui::InputText("IP", ip_address, sizeof(ip_address));
        ImGui::SetNextItemWidth(120);
        ImGui::InputText("Msg", connect_content, sizeof(connect_content));

        if (ImGui::Button("Connect") && !controller) {
            std::string getip{ ip_address };
            if (isNetworkAvailable(getip)) {
                controller = YMConnect::OpenConnection(getip, status);
                if (status.StatusCode != 0) { std::stringstream ss; ss << status; *sttlogs << "Error: " + ss.str(); }
                else {
                    strcpy_s(connect_content, "Welcome to OhLabs.");
                    status = controller->ControlCommands->DisplayStringToPendant(connect_content);
                    *sttlogs << "Connected";
                }
            }
            else {
                *sttlogs << "Error: Cannot connect to " + getip + ". Check IP on your robot system.";
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Disconnect")) {
            if (controller)
            {
                status = controller->ControlCommands->DisplayStringToPendant("Disconnect!");
                disconnect_robot(false);

            }
        }
        ImGui::Separator();
        ImGui::End();
        ImGui::PopStyleColor();
    }

    void ymconnect::disconnect_robot(bool showmsg) {
        *sttlogs << "Disconnect" ;
        strcpy_s(connect_content, "Disconnect!");
        if (controller) { controller = nullptr; }
        if (showmsg)
        { ImGui::OpenPopup("Disconnected"); }
    }

    void ymconnect::render() {
        // get proMeshRb and Statuslogs
        if (!proMeshRb) { proMeshRb = &nelems::mMesh::getInstance(); }
        connect_robot();
        setup_MOVE_ui(ui_state);
        move_robot();
        auto future_read = std::async(std::launch::async, [this]() {read_robot(); });
        if (future_read.wait_for(std::chrono::seconds(3)) == std::future_status::timeout) 
        {
            *sttlogs << "out of time";
            disconnect_robot(true); return;
        }
                       
        static int x{ 200 }, y{ 400 };
        static float pos_x, pos_y,sizex,sizey;
        nui::FrameManage::getViewportSize(pos_x, pos_y);
        nui::FrameManage::get3DSize(sizex, sizey);
        ImGui::SetNextWindowPos(ImVec2(pos_x + 15+ sizex*0.83f, pos_y + sizey * 0.25f)); // Set the position of the frame
        ImGui::SetNextWindowSize(ImVec2(sizex*0.15f, sizey*0.73f)); // Set the size of the frame
        ImGui::Begin("Robot Status:", nullptr,
            ImGuiWindowFlags_NoDocking | // Cannot be docked
            ImGuiWindowFlags_NoBackground | // Do not display background
            ImGuiWindowFlags_NoNavFocus); // Does not bring to front on focus
        ImGui::Separator();
        if (resultmsg) {ImGui::TextColored(ImVec4(0.85f, 0.6f, 0.0f, 1.0f), resultmsg.str().c_str());        }
        ImGui::Separator();
        // set button for higher fps
        if (ImGui::Button("Get Cr Pos") && controller)
        {
            PositionData raxisData{};
            controller->ControlGroup->ReadPositionData(ControlGroupId::R1, CoordinateType::BaseCoordinate, 0, 0, raxisData);
            std::stringstream ss; ss << raxisData; std::string currentInfo = ss.str();
            std::replace(currentInfo.begin(), currentInfo.end(), '\n', ';');
            *sttlogs << "Position: " + currentInfo; std::cout << raxisData;
        }
        ImGui::End();
    }

    struct JointMotionFunctor {auto operator()(ControlGroupId groupId, PositionData& posData, float speed) const {
            return JointMotion(groupId, posData, speed);
        }};
    struct LinearMotionFunctor {auto operator()(ControlGroupId groupId, PositionData& posData, float speed) const {
            return LinearMotion(groupId, posData, speed);
        }};

    // Check the current pos to the 1st pos
    // check the pos it with pos it-1
    // If the distance is too small (XYZ): Alert.
    //
    void ymconnect::move_robot() {
        static auto LinepathStart = std::chrono::high_resolution_clock::now();

        auto nearest_pos = [&](const PositionData& pos1, const PositionData& pos2) {
            size_t id = 0;
            return std::all_of(pos1.axisData.begin(), pos1.axisData.begin() + 3, [&](float a)
                {return fabs(a - pos2.axisData[id++]) < 0.01f; }); };

        double Linepathelapsed_seconds = std::chrono::duration_cast<std::chrono::duration<double>>
            (std::chrono::high_resolution_clock::now() - LinepathStart).count();

        // RENDER LINE PATH
        if (ui_state.lineshpath && Linepathelapsed_seconds > 0.4f) {
            LinepathStart = std::chrono::high_resolution_clock::now();
            std::shared_ptr<nelems::oMesh> mesh = nullptr; static bool newmesh_flag = false;
            for (auto &it: *(proMeshRb->getMesh()))
            {
                if (std::string(it->oname).find("movepath__SKIP") != std::string::npos) {
                    mesh = it;
                    break; }
            }

            if (mesh == nullptr) {
                mesh = std::make_shared<nelems::oMesh>();
                mesh->changeName("movepath__SKIP__");
                mesh->ID = proMeshRb->getCurrentTimeMillis(0);
                mesh->oMaterial.mColor = glm::vec3(1.0f, 0.00f, 0.03f); newmesh_flag = true;
            }
            else {
                mesh->delete_buffers();
                mesh->mVertexIndices.clear();
                mesh->mVertices.clear();
            }
            nelems::VertexHolder vertex{}; unsigned int i = 0;
            for (auto it = ui_state.rbpos.begin(); it != ui_state.rbpos.end(); it++,i++) {
                if (std::distance(ui_state.rbpos.begin(), it) >= ui_state.coumove) { break; }                
                vertex.mPos = glm::vec3((*it)[0], (*it)[1], (*it)[2]);
                vertex.mNormal = glm::vec3(0.0f, 0.0f, 1.0f);
                mesh->add_vertex(vertex);
                if (i > 0) {
                    mesh->add_vertex_index(i - 1);
                    mesh->add_vertex_index(i);
                }
            }
            mesh->init();
            // mesh->selected = true;
            if (newmesh_flag) { proMeshRb->add_mesh(mesh); newmesh_flag = false; }
        }
        // Start Moving
        if (ui_state.START_Flag &&controller)
        {
            if (status.StatusCode != 0) { return; }
            ControllerStateData stateData{}; std::stringstream ss;
            if (controller) { controller->Status->ReadState(stateData); }
            ss << stateData;
            if (stateData.isAlarming) {
                AlarmHistory alarmHistoryData;
                if (controller) { controller->Faults->GetAlarmHistory(AlarmCategory::Minor, 3, alarmHistoryData); }
                ss << alarmHistoryData << std::endl; *sttlogs << "ERROR: "+ss.str();
            }


            *sttlogs << "Start Moving to " + std::to_string(ui_state.coumove) + " points.";
            for (auto it = ui_state.movTypes.begin(); it != ui_state.movTypes.end(); it++) {
                if (std::distance(ui_state.movTypes.begin(), it) >= ui_state.coumove) { break; }
                ui_state.b1workpos->coordinateType = CoordinateType::RobotCoordinate;
                int index = std::distance(ui_state.movTypes.begin(), it);
                std::copy(ui_state.rbpos[index].begin(), ui_state.rbpos[index].end(), ui_state.b1workpos->axisData.begin());

                //check index 0;
				if (index == 0) {
                    // check current pos with 1st pos
                    *ui_state.tpstatus = controller->ControlGroup->ReadPositionData(ControlGroupId::R1, CoordinateType::BaseCoordinate, 0, 0, *ui_state.b1crpos);
                    if (nearest_pos(*ui_state.b1workpos, *ui_state.b1crpos)) { *sttlogs << "The first point is too close to the robot."; return; }
				}

                if ((*it) == 0)
                {
                    *ui_state.tpstatus = controller->MotionManager->AddPointToTrajectory(LinearMotion(ControlGroupId::R1, *ui_state.b1workpos, ui_state.spdlinear));
                }
                else if ((*it) == 2)
                {
                    *ui_state.tpstatus = controller->MotionManager->AddPointToTrajectory(JointMotion(ControlGroupId::R1, *ui_state.b1workpos, ui_state.spdjoint));
                }
                else if ((*it) == 1) // Circular
                {
                    CoordinateArray coorarr;
                    std::copy(ui_state.rbpos[index-1].begin(), ui_state.rbpos[index - 1].end(), coorarr.begin());                    
                    *ui_state.tpstatus = controller->MotionManager->AddPointToTrajectory(CircularMotion(ControlGroupId::R1, *ui_state.b1workpos, coorarr, ui_state.spdlinear));
                }
                else if ((*it) == 3) { continue; } // Skip as mid point
            }
            *ui_state.tpstatus = controller->ControlCommands->SetServos(SignalStatus::ON);
            *ui_state.tpstatus = controller->MotionManager->MotionStart();
        }
    }
  
    void ymconnect::setup_MOVE_ui(UIState& ui_state)
    {
        ImGui::Separator();
        ImGui::Begin("Attributes: ");
        static auto LinepathStart = std::chrono::high_resolution_clock::now();
        ImGui::SetNextItemWidth(80); int step = 1, faststep = 2;
        ImGui::InputScalar("##", ImGuiDataType_U32, &ui_state.coumove,  &step, &faststep, NULL, ImGuiInputTextFlags_None);
        if (ui_state.coumove < 1) { ui_state.coumove = 1; }; ImGui::SameLine();
        ui_state.START_Flag = ImGui::Button("Start (R)"); ImGui::SameLine();
        
        if (ImGui::BeginPopupContextItem("RightStart", ImGuiPopupFlags_MouseButtonRight)) 
        { 
            if (ImGui::MenuItem((ui_state.SharedMemoryFlag?"Stop SharedMemory" : "start SharedMemory")))
            {
                ui_state.SharedMemoryFlag = !ui_state.SharedMemoryFlag;
                if (ui_state.SharedMemoryFlag) { *sttlogs << "Start Shared Memory"; }
                else { *sttlogs << "Stop Shared Memory"; }

            }
            if (ImGui::MenuItem(ui_state.lineshpath? "Hide Path": "Show Path"))
            {
                ui_state.lineshpath = !ui_state.lineshpath;
                if (ui_state.lineshpath) { *sttlogs << "Render the Move Path"; }
                else { *sttlogs << "Stop the Move Path"; proMeshRb->delete_byname("movepath__SKIP__"); }
            }
            ImGui::EndPopup();
        }

        ImGui::SameLine();
        if (ui_state.SharedMemoryFlag)
        { shmdata->getter_6pos(get6pos); }
        if (ui_state.rbpos.size() < get6pos.size()) {
            ui_state.rbpos.resize(get6pos.size(), std::vector<float>(6, 0.0f));
            ui_state.movTypes.resize(get6pos.size(), 1);
        }
        if (get6pos.size() > 0) {
            for (auto it = get6pos.begin(); it != get6pos.end(); ++it) {
                ui_state.rbpos[std::distance(get6pos.begin(), it)] = *it;
            }
        }
        ImGui::SameLine(); // Check the trajectory
        if (ImGui::Button("Stop") && controller)
        {
            std::unique_ptr<StatusInfo> temptstt = std::make_unique<StatusInfo>();
            *ui_state.tpstatus = controller->MotionManager->MotionStop();
            controller->ControlCommands->SetServos(SignalStatus::OFF);
            //*ui_state.tpstatus = controller->MotionManager->MotionStart();
            *sttlogs << "Trying to clear trajectory";
        }
        ImGui::SameLine(); // Check the trajectory
        if (ImGui::Button("Clear") && controller)
        {
            std::unique_ptr<StatusInfo> temptstt = std::make_unique<StatusInfo>();
            *temptstt = controller->Faults->ClearAllFaults();
            if (temptstt->StatusCode == 0) { resultmsg.str(" "); }

            *ui_state.tpstatus = controller->MotionManager->ClearAllTrajectory();
            controller->ControlCommands->SetServos(SignalStatus::OFF);
            //*ui_state.tpstatus = controller->MotionManager->MotionStart();
            *sttlogs << "Trying to clear trajectory";
        }
        ImGui::SameLine();
        if (ImGui::Button("Home") && controller)
        {
            //*ui_state.tpstatus = controller->Variables->BasePositionVariable->Read(0, *ui_state.b1PositionData);
            //*ui_state.tpstatus = controller->Kinematics->ConvertPosition(ControlGroupId::R1, ui_state.b1PositionData->positionData, KinematicConversions::PulseToCartesianPos, *ui_state.b1crpos);
            ui_state.b1crpos->coordinateType = CoordinateType::RobotCoordinate;
            ui_state.b1crpos->axisData = { 380,0,405,180,0,0 };
            JointMotion r1home(ControlGroupId::R1, *ui_state.b1crpos, 3);
            *ui_state.tpstatus = controller->MotionManager->AddPointToTrajectory(r1home);
            *ui_state.tpstatus = controller->ControlCommands->SetServos(SignalStatus::ON);
            *ui_state.tpstatus = controller->MotionManager->MotionStart();
            *sttlogs << "Go back to the Home pos";
        }
        ImGui::SameLine();
        ImGui::Text("Lin spd:"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
        ImGui::InputFloat("##LS", &ui_state.spdlinear, 0.0f, 0.0f, "%.2f");  ImGui::SameLine();
        ImGui::Text("Joint spd:"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
        ImGui::InputFloat("##JS", &ui_state.spdjoint, 0.0f, 0.0f, "%.2f");
        ImGui::Separator();

        std::vector<const char*> selection = { "Linear", "Circular", "Joint","Mid-Cur"};

        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 100.0f);
        ImGui::Text(" X:"); ImGui::SameLine(0.0f, 40.0f);
        ImGui::Text(" Y:"); ImGui::SameLine(0.0f, 40.0f);
        ImGui::Text(" Z:"); ImGui::SameLine(0.0f, 40.0f);
        ImGui::Text("Rx:"); ImGui::SameLine(0.0f, 40.0f);
        ImGui::Text("Ry:"); ImGui::SameLine(0.0f, 40.0f);
        ImGui::Text("Rz:"); ImGui::SameLine(0.0f, 40.0f);
        ImGui::Text("Type:");

        for (int j = 0; j < ui_state.coumove; ++j) {
            if (ui_state.rbpos.size() < static_cast<size_t>(ui_state.coumove)) {
                ui_state.rbpos.resize(ui_state.coumove, std::vector<float>(6, 0.0f));
                ui_state.movTypes.resize(ui_state.coumove, 0);
            }
            if (ImGui::Button(("Org " + std::to_string(j)).c_str()) && controller) {
                *ui_state.tpstatus = controller->Variables->BasePositionVariable->Read(0, *ui_state.b1PositionData);
                *ui_state.tpstatus = controller->Kinematics->ConvertPosition(ControlGroupId::R1, ui_state.b1PositionData->positionData, KinematicConversions::PulseToCartesianPos, *ui_state.b1crpos);
                std::copy(ui_state.b1crpos->axisData.begin(), ui_state.b1crpos->axisData.begin() + 6, ui_state.rbpos[j].begin());
                *sttlogs << "Update the Original Position for " << std::to_string(j);
            }
            ImGui::SameLine();
            if (ImGui::Button(("Cr " + std::to_string(j)).c_str()) && controller) {
                *ui_state.tpstatus = controller->ControlGroup->ReadPositionData(ControlGroupId::R1, CoordinateType::BaseCoordinate, 0, 0, *ui_state.b1workpos);
                std::copy(ui_state.b1workpos->axisData.begin(), ui_state.b1workpos->axisData.begin()+6, ui_state.rbpos[j].begin());
                *sttlogs << "Update the Current Position for " + std::to_string(j);
            }
            ImGui::SameLine(); ImGui::SetNextItemWidth(50);
            ImGui::InputFloat(("##X" + std::to_string(j)).c_str(), &ui_state.rbpos[j][0], 0.0f, 0.0f, "%.2f");
            ImGui::SameLine(); ImGui::SetNextItemWidth(50);
            ImGui::InputFloat(("##Y" + std::to_string(j)).c_str(), &ui_state.rbpos[j][1], 0.0f, 0.0f, "%.2f");
            ImGui::SameLine(); ImGui::SetNextItemWidth(50);
            ImGui::InputFloat(("##Z" + std::to_string(j)).c_str(), &ui_state.rbpos[j][2], 0.0f, 0.0f, "%.2f"); 
            ImGui::SameLine(); ImGui::SetNextItemWidth(50);
            ImGui::InputFloat(("##RX" + std::to_string(j)).c_str(), &ui_state.rbpos[j][3], 0.0f, 0.0f, "%.2f"); 
            ImGui::SameLine(); ImGui::SetNextItemWidth(50);
            ImGui::InputFloat(("##RY" + std::to_string(j)).c_str(), &ui_state.rbpos[j][4], 0.0f, 0.0f, "%.2f"); 
            ImGui::SameLine(); ImGui::SetNextItemWidth(50);
            ImGui::InputFloat(("##RZ" + std::to_string(j)).c_str(), &ui_state.rbpos[j][5], 0.0f, 0.0f, "%.2f"); 
            ImGui::SameLine(); ImGui::SetNextItemWidth(70);
            std::string label = "##movtyp" + std::to_string(j);
            ImGui::Combo(label.c_str(), &ui_state.movTypes[j], selection.data(), selection.size());
        }
        ImGui::End();
    }

    void ymconnect::read_robot() {
        std::stringstream strget;
        StatusInfo tpstatus;
        PositionData rposData{}, rjointangle{};
        if (status.StatusCode != 0) {return;} 
        resultmsg.str(" ");
               
        // Have to check controler before calling for avoiding nullptr due to disconnect from threading.
        if (controller) { controller->ControlGroup->ReadPositionData(ControlGroupId::R1, CoordinateType::Pulse, 0, 0, rposData); }
        if (controller) {controller->Kinematics->ConvertPosition(ControlGroupId::R1, rposData, KinematicConversions::PulseToJointAngle, rjointangle);}
        if (!controller) { return; }
        resultmsg << rposData; 
        std::copy(rjointangle.axisData.begin(), rjointangle.axisData.begin()+6, angle.begin());

        bool isOutOfLimit = std::any_of(std::begin(angle), std::end(angle), [this, i = 0](float a) mutable {
            return a < limitangle[i][0] || a > limitangle[i++][1];
            });

        if (isOutOfLimit) {
            if (controller) { controller->MotionManager->MotionStop(true); }
            if (controller) { controller->MotionManager->ClearAllTrajectory(); }
            if (controller) { controller->ControlCommands->SetServos(SignalStatus::OFF); }
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