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
        if (!controller){shmdata->status_robot(false);}
        else { shmdata->status_robot(true); }
        // get proMeshRb and Statuslogs
        if (!proMeshRb) { proMeshRb = &nelems::mMesh::getInstance(); }
        Robot_Controls_table();
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
        nui::FrameManage::GetViewPos(pos_x, pos_y);
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
            if (stateData.controlMode != ControllerStateData::ControlMode::Remote) {
                *sttlogs << "Please check the Key to REMOTE mode."; return;
            }
            if ( stateData.isInHold) {
                *sttlogs << "ROBOT is keeping in the HOLD Mode."; return;
            }

            *sttlogs << "Start Sending Rob Coor to " + std::to_string(ui_state.coumove) + " points.";
            
            
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
                    *ui_state.tpstatus = controller->MotionManager->AddPointToTrajectory(LinearMotion(ControlGroupId::R1, *ui_state.b1workpos, ui_state.spdlinear, ui_state.accdec));
                }
                else if ((*it) == 2)
                {
                    *ui_state.tpstatus = controller->MotionManager->AddPointToTrajectory(JointMotion(ControlGroupId::R1, *ui_state.b1workpos, ui_state.spdjoint, ui_state.accdec));
                }
                else if ((*it) == 1) // Circular
                {
                    CoordinateArray coorarr;
                    std::copy(ui_state.rbpos[index-1].begin(), ui_state.rbpos[index - 1].end(), coorarr.begin());                    
                    *ui_state.tpstatus = controller->MotionManager->AddPointToTrajectory(CircularMotion(ControlGroupId::R1, *ui_state.b1workpos, coorarr, ui_state.spdlinear, ui_state.accdec));
                }
                else if ((*it) == 3) { continue; } // Skip as mid point
            }
            *ui_state.tpstatus = controller->ControlCommands->SetServos(SignalStatus::ON);
            *ui_state.tpstatus = controller->MotionManager->MotionStart();
            std::cout << "deldone";
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
            ui_state.movTypes[0] = 2;
            // reset get6pos to size 0
            get6pos.clear();

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
            controller->Variables->BasePositionVariable->Read(0, *ui_state.b1PositionData);
            controller->Kinematics->ConvertPosition(ControlGroupId::R1, ui_state.b1PositionData->positionData, KinematicConversions::PulseToCartesianPos, *ui_state.b1crpos);
            JointMotion r1home(ControlGroupId::R1, *ui_state.b1crpos, 3);
            *ui_state.tpstatus = controller->MotionManager->AddPointToTrajectory(r1home);

            ui_state.b1crpos->coordinateType = CoordinateType::RobotCoordinate;
            ui_state.b1crpos->axisData = { 380.0f,0.0f,405.0f,-180.0f,0.0f,180.0f };
            JointMotion r2home(ControlGroupId::R1, *ui_state.b1crpos, 3);
            *ui_state.tpstatus = controller->MotionManager->AddPointToTrajectory(r2home);
            *ui_state.tpstatus = controller->ControlCommands->SetServos(SignalStatus::ON);
            *ui_state.tpstatus = controller->MotionManager->MotionStart();
            *sttlogs << "Go back to the Home pos";
        }
        ImGui::SameLine();
        ImGui::Text("Lin spd:"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
        ImGui::InputFloat("##LS", &ui_state.spdlinear, 0.0f, 0.0f, "%.2f");  ImGui::SameLine();
        ImGui::Text("Joint spd:"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
        ImGui::InputFloat("##JS", &ui_state.spdjoint, 0.0f, 0.0f, "%.2f"); ImGui::SameLine(); ImGui::SetNextItemWidth(30);
        // MotionAccelDecel accdec{ 20,20 };
        ImGui::InputDouble("ACC", &ui_state.accdec.accelRatio,0.0f,0.0f,"%.2f"); ImGui::SameLine(); ImGui::SetNextItemWidth(30);
        ImGui::InputDouble("DEC", &ui_state.accdec.decelRatio, 0.0f, 0.0f, "%.2f");
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


    void ymconnect::Robot_Controls_table()
    {
        // *****************************************************
        // A - Get base objects if not already retrieved
        if (base[0] == nullptr)
        {
            for (auto it = proMeshRb->getMesh()->begin(); it != proMeshRb->getMesh()->end(); it++)
            {
                auto mesh = *it;
                std::string name = std::string(mesh->oname);
                if (name.find("RBSIMBase_1") != std::string::npos) { base[0] = std::move(mesh); }
                else if (name.find("RBSIMBase_2") != std::string::npos) { base[1] = std::move(mesh); }
                else if (name.find("RBSIMBase_3") != std::string::npos) { base[2] = std::move(mesh); }
                else if (name.find("RBSIMBase_4") != std::string::npos) { base[3] = std::move(mesh); }
                else if (name.find("RBSIMBase_5") != std::string::npos) { base[4] = std::move(mesh); }
                else if (name.find("RBSIMBase_6") != std::string::npos) { base[5] = std::move(mesh); }
                else if (name.find("RBSIMBase_7") != std::string::npos) { base[6] = std::move(mesh); }
            }
        }
        // If no base objects found, return
        if (base[0] == nullptr) { return; }

        // *****************************************************
        // B - Initialize static variables for joint angles & RB Hand pos
        static float tolerance = 0.1f, pre[6]{ 0 }, prehand[3]{ 0 };
        // static std::vector<std::vector <float>> limangle{ 6, {-360,360} };
        // static bool CtrFlag = false;
        static std::vector<std::shared_ptr<nelems::oMesh>> OrgBase = {
            std::make_shared<nelems::oMesh>(*base[0]),
            std::make_shared<nelems::oMesh>(*base[1]),
            std::make_shared<nelems::oMesh>(*base[2]),
            std::make_shared<nelems::oMesh>(*base[3]),
            std::make_shared<nelems::oMesh>(*base[4]),
            std::make_shared<nelems::oMesh>(*base[5]),
            std::make_shared<nelems::oMesh>(*base[6])
        };

        ImGui::Begin("Robot Controls", nullptr);

        if (ImGui::BeginPopupContextItem("Robot Controls Popup", ImGuiPopupFlags_MouseButtonRight)) {
            if (ImGui::MenuItem("Toggle Control Flag")) {  VisualizeFlag = !VisualizeFlag; }
            ImGui::EndPopup();
        }

        prehand[0] = base[5]->oMaterial.position.x;
        prehand[1] = base[5]->oMaterial.position.y;
        prehand[2] = base[5]->oMaterial.position.z;
        // *****************************************************
        // C - Livesync & Control mode 
        // LiveSync Mode: 
        if (VisualizeFlag == false) {
            ImVec4 vecred(0.0f, 0.0f, 1.0f, 1.0f);
            for (int i = 0; i < 6; ++i) {
                ImGui::BeginChild((std::string("JointGroup") + std::to_string(i)).c_str(), ImVec2(110, 85), true);
                ImGui::BeginGroup();

                // Display joint angle
                //ImGui::TextColored(vecred, "Joint %d: %.2f", i + 1, ang[i]);
                ImGui::Text("Joint %d: %.2f", i + 1, angle[i]);
                // Display limits
                ImGui::Text("Min: "); ImGui::SameLine();
                ImGui::SetNextItemWidth(50);
                ImGui::InputFloat((std::string("##") + std::to_string(i) + "_0").c_str(), &limitangle[i][0], 0, 0, "%.2f");

                ImGui::Text("Max:"); ImGui::SameLine();
                ImGui::SetNextItemWidth(50);
                ImGui::InputFloat((std::string("##") + std::to_string(i) + "_1").c_str(), &limitangle[i][1], 0, 0, "%.2f");

                // End the group
                ImGui::EndGroup();
                ImGui::EndChild();

                if (i % 3 != 2) { ImGui::SameLine(); }
            }
        }
        // Control Mode:
        else
        {

            ImGui::SetNextItemWidth(100);
            ImGui::InputFloat("Joint 1", &angle[0], 1, 0.1, "%.2f");
            ImGui::SetNextItemWidth(100);
            ImGui::InputFloat("Joint 2", &angle[1], 1, 0.1, "%.2f");
            ImGui::SetNextItemWidth(100);
            ImGui::InputFloat("Joint 3", &angle[2], 1, 0.1, "%.2f");
            ImGui::SetNextItemWidth(100);
            ImGui::InputFloat("Joint 4", &angle[3], 1, 0.1, "%.2f");
            ImGui::SetNextItemWidth(100);
            ImGui::InputFloat("Joint 5", &angle[4], 1, 0.1, "%.2f");
            ImGui::SetNextItemWidth(100);
            ImGui::InputFloat("Joint 6", &angle[5], 1, 0.1, "%.2f");
            ImGui::Separator();

        }
        //*****************************************************
        // D - Caclculate for simulate the movement

        // D - 2 Joints Siumulation
        bool exceeds_tolerance = false;
        for (int i = 0; i < 6; ++i) {
            if (std::abs(angle[i] - pre[i]) > tolerance) {
                exceeds_tolerance = true;
                break;
            }
        }
        if (exceeds_tolerance)
        {
            for (int i = 0; i < 7; i++)
            {
                base[i]->mVertices.clear();
                base[i]->oMaterial.rotation = OrgBase[i]->oMaterial.rotation;
                base[i]->mVertices = OrgBase[i]->mVertices;
                base[i]->oMaterial.position = OrgBase[i]->oMaterial.position;
                base[i]->oMaterial.mOxyz = OrgBase[i]->oMaterial.mOxyz;
            }
            pre[0] = pre[1] = pre[2] = pre[3] = pre[4] = pre[5] = 0;
            // rotateJoint(6, ang[5], pre[5], tolerance, base, ang[5] - pre[5], 0, 0);
            rotateJoint(5, angle[5], pre[5], tolerance, base, -(angle[5] - pre[5]), 0, 0);
            rotateJoint(4, angle[4], pre[4], tolerance, base, 0, -(angle[4] - pre[4]), 0);
            rotateJoint(3, angle[3], pre[3], tolerance, base, -(angle[3] - pre[3]), 0, 0);
            rotateJoint(2, angle[2], pre[2], tolerance, base, 0, -(angle[2] - pre[2]), 0);
            rotateJoint(1, angle[1], pre[1], tolerance, base, 0, (angle[1] - pre[1]), 0);
            rotateJoint(0, angle[0], pre[0], tolerance, base, 0, 0, (angle[0] - pre[0]));
        }
        // Create buffers for each base
        for (auto& bs : base)
        {
            bs->delete_buffers();
            bs->create_buffers();
        }
        ImGui::End();
    }
    void ymconnect::rotateJoint(size_t jointIndex, float& ang, float& pre, const float tolerance,
        std::vector<std::shared_ptr <nelems::oMesh>>& base,
        float diffX, float diffY, float diffZ)
    {
        // Create rotation matrix using Eigen
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

        // Combine rotation matrices
        if (diffX != 0.0f) {
            float rad = glm::radians(diffX);
            Eigen::Matrix4f rotX;
            rotX << 1, 0, 0, 0,
                0, cos(rad), -sin(rad), 0,
                0, sin(rad), cos(rad), 0,
                0, 0, 0, 1;
            transform *= rotX;
        }
        if (diffY != 0.0f) {
            float rad = glm::radians(diffY);
            Eigen::Matrix4f rotY;
            rotY << cos(rad), 0, sin(rad), 0,
                0, 1, 0, 0,
                -sin(rad), 0, cos(rad), 0,
                0, 0, 0, 1;
            transform *= rotY;
        }
        if (diffZ != 0.0f) {
            float rad = glm::radians(diffZ);
            Eigen::Matrix4f rotZ;
            rotZ << cos(rad), -sin(rad), 0, 0,
                sin(rad), cos(rad), 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;
            transform *= rotZ;
        }

        ang = std::round(ang * 100.0f) / 100.0f;


        if (std::abs(ang - pre) > tolerance)
        {
            float diff = ang - pre;
            glm::vec3 center = base[jointIndex]->oMaterial.position;

            // Parallelize the loop using OpenMP
#pragma omp parallel for
            for (size_t i = jointIndex; i < base.size(); ++i) {
                for (auto& vertex : base[i]->mVertices) {
                    Eigen::Vector4f newPos = transform * Eigen::Vector4f(vertex.mPos.x - center.x, vertex.mPos.y - center.y, vertex.mPos.z - center.z, 1.0f);
                    vertex.mPos = glm::vec3(newPos.x() + center.x, newPos.y() + center.y, newPos.z() + center.z);
                }

                // Update oMaterial position
                Eigen::Vector4f centerPos(base[i]->oMaterial.position.x - center.x, base[i]->oMaterial.position.y - center.y, base[i]->oMaterial.position.z - center.z, 1.0f);
                Eigen::Vector4f newCenterPos = transform * centerPos;
                base[i]->oMaterial.position = glm::vec3(newCenterPos.x() + center.x, newCenterPos.y() + center.y, newCenterPos.z() + center.z);
            }
            pre = std::round(ang * 100.0f) / 100.0f;
        }
    }
}