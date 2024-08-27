#include "pch.h"
#include "property_panel.h"
#include <filesystem> 
#include <ctime>
#include "ui/HotkeyMenubar.h"


#include <cmath>
#include "elems/TranRBMatrix.h"
#include <cmath>
#include "chrono"

#include "thread"
#include "mutex"
using namespace Eigen;

namespace nui
{
    nui::Property_Panel::Property_Panel():
        mRobot(nullptr), sttlogs(nullptr), cmdrder(nullptr)
    {
        SceneView* sceneView = &nui::SceneView::getInstance();
        // rdMesh = sceneView->rdMesh;
        cmdrder = std::make_unique<nui::CMDReader>();

        // for (int i{ 0 }; i < 7; i++) { base.push_back(nullptr); }
        robinit = &RobInitFile::getinstance();
        ImGuiIO& io = ImGui::GetIO();
        std::string fontPath; robinit->get_settings("rob_font", fontPath);
        fontPath = "RobFonts\\" + fontPath + ".ttf";

        // Check if fontPath existed
        if (std::filesystem::exists(fontPath) && fontPath.size() > 10)
        { io.Fonts->AddFontFromFileTTF(fontPath.c_str(), 17.0f); }
        /*ImFontConfig fontConfig;
        fontConfig.OversampleH = 6;  
        fontConfig.OversampleV = 6;  
        fontConfig.GlyphExtraSpacing.x = 1.0f; 
        ImFont* customFont = io.Fonts->AddFontFromFileTTF(fontPath.c_str(), 17.0f, &fontConfig);
        if (customFont) {
            io.FontDefault = customFont;  
        }*/

        mRobot = &nymrobot::ymconnect::getInstance();
        sttlogs = &nui::StatusLogs::getInstance();
    }

    void Property_Panel::render(GLFWwindow* mWindow)
    {

        mRobot->render();

        static nui::HotkeyMenubar hotkey_manage;
        hotkey_manage.mMenuBar(mWindow);
        hotkey_manage.mHotkey(mWindow);
        
        ImGui::Begin("Properties");
        layer_frame(); // define selectedID
        coordinate_frame(); // show the position
        material_frame(); // show material properties
        camera_frame(); // show camera properties
        ImGui::End();
        if (cmdrder){cmdrder->readCMD();}
        obInfo_frame(); // show object info such as vertices and vertex 


        ImGui::Begin("StatusLogs", nullptr, ImGuiWindowFlags_HorizontalScrollbar | ImGuiWindowFlags_AlwaysVerticalScrollbar);
        static bool fullview_Flag = false;
        if (ImGui::BeginPopupContextItem("RightStart", ImGuiPopupFlags_MouseButtonRight))
        {
            if (ImGui::MenuItem("Restart"))
            {
                if (!cmdrder){ cmdrder = std::make_unique<nui::CMDReader>();}
                cmdrder->CMDClear();
                cmdrder->restart();
            }
            if (ImGui::MenuItem("Clear"))
            {
                cmdrder->CMDClear();
                cmdrder.reset();
                cmdrder = std::make_unique<nui::CMDReader>();
            }
            if (ImGui::MenuItem("Full")) {
                fullview_Flag = true;
            }
            ImGui::EndPopup();
        }
        // Flag for show or hidden
        static bool cmd_flag = true, mavis_flag = true, system_flag = true;
        ImGui::Checkbox("cmdLogs", &cmd_flag); ImGui::SameLine(); ImGui::Checkbox("Vision", &mavis_flag);
        ImGui::SameLine(); ImGui::Checkbox("System", &system_flag);

        static std::unique_ptr<cv::String> stt_content = std::make_unique<cv::String>();

        // color        
        for (auto stt_content = sttlogs->getStatus().begin(); stt_content != sttlogs->getStatus().end(); stt_content++){
            if ( stt_content == sttlogs->getStatus().begin() && stt_content->find("---") != std::string::npos) {
                *stt_content += "---";
                if (stt_content->size() > 80) {
                    size_t firstDashPos = stt_content->find("---")+4;
                    if (firstDashPos != std::string::npos) {
                        *stt_content = stt_content->substr(0, firstDashPos);
                    }
                }
                // print as Red color
                ImGui::TextWrapped(stt_content->substr(0, 10).c_str()); // time
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.45f, 0.20f, 1.0f)); ImGui::SameLine();
                ImGui::TextWrapped(stt_content->substr(10).c_str()); // content
                if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0)) { fullview_Flag = !fullview_Flag; }
                ImGui::PopStyleColor();
                continue;
            }

            if (stt_content->find("cmd Logs") != std::string::npos && cmd_flag) {
                // print as Red color
                ImGui::TextWrapped(stt_content->substr(0,10).c_str()); // time
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.65f, 0.0f, 1.0f)); ImGui::SameLine();
                ImGui::TextWrapped(stt_content->substr(10).c_str()); // content
                if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0)) {fullview_Flag = !fullview_Flag;}
                ImGui::PopStyleColor();
            }
            else if (stt_content->find("Machine Vision") != std::string::npos && mavis_flag)
            {
                // print as purple
                ImGui::TextWrapped(stt_content->substr(0, 10).c_str()); // time
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.5f, 1.0f, 0.5f, 1.0f)); ImGui::SameLine();
                ImGui::TextWrapped(stt_content->substr(10).c_str()); // content
                if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0)) {fullview_Flag = !fullview_Flag;}
                ImGui::PopStyleColor();
            }
            else if (system_flag)
            {
                // out if cmdLogs and Machine Vision
                if (stt_content->find("cmd Logs") != std::string::npos || stt_content->find("Machine Vision") != std::string::npos)
                {continue;}
                // print as yellow
                ImGui::TextWrapped(stt_content->substr(0, 10).c_str()); // time
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 0.0f, 1.0f)); ImGui::SameLine();
                ImGui::TextWrapped(stt_content->substr(10).c_str()); // content
                if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0)) {fullview_Flag = !fullview_Flag;}
                ImGui::PopStyleColor();
            }
        }
        ImGui::End();
        if (fullview_Flag)
        {
            ImGui::SetWindowPos(ImVec2(100, 20));
            static std::unique_ptr<bool> lock_frame = std::make_unique<bool>(); ;
            if (lock_frame == nullptr) { robinit->get_settings("lock_frame", *lock_frame); }
            
            if (!lock_frame)  { ImGui::Begin("Laser View", &fullview_Flag, ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoMove); }
            else  { ImGui::Begin("Laser View", &fullview_Flag, ImGuiWindowFlags_NoDocking); }

            for (auto stt_content = sttlogs->getStatus().begin(); stt_content != sttlogs->getStatus().end(); stt_content++) {
                if (stt_content->find("cmd Logs") != std::string::npos && cmd_flag) {
                    // print as Red color
                    ImGui::TextWrapped(stt_content->substr(0, 10).c_str()); // time
                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.65f, 0.0f, 1.0f)); ImGui::SameLine();
                    ImGui::TextWrapped(stt_content->substr(10).c_str()); // content
                    if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0)) { fullview_Flag = !fullview_Flag; }
                    ImGui::PopStyleColor();
                }
                else if (stt_content->find("Machine Vision") != std::string::npos && mavis_flag)
                {
                    // print as purple
                    ImGui::TextWrapped(stt_content->substr(0, 10).c_str()); // time
                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.5f, 1.0f, 0.5f, 1.0f)); ImGui::SameLine();
                    ImGui::TextWrapped(stt_content->substr(10).c_str()); // content
                    if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0)) { fullview_Flag = !fullview_Flag; }
                    ImGui::PopStyleColor();
                }
                else if (system_flag)
                {
                    // out if cmdLogs and Machine Vision
                    if (stt_content->find("cmd Logs") != std::string::npos || stt_content->find("Machine Vision") != std::string::npos)
                    {
                        continue;
                    }
                    // print as yellow
                    ImGui::TextWrapped(stt_content->substr(0, 10).c_str()); // time
                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 0.0f, 1.0f)); ImGui::SameLine();
                    ImGui::TextWrapped(stt_content->substr(10).c_str()); // content
                    if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0)) { fullview_Flag = !fullview_Flag; }
                    ImGui::PopStyleColor();
                }
            }
            ImGuiIO& io = ImGui::GetIO();
            ImVec2 mouse_pos = io.MousePos;
            ImVec2 window_pos = ImGui::GetWindowPos();
            ImVec2 window_size = ImGui::GetWindowSize();
            if (ImGui::IsMouseClicked(0) && !lock_frame) {
                if (mouse_pos.x < window_pos.x || mouse_pos.x > window_pos.x + window_size.x ||
                    mouse_pos.y < window_pos.y || mouse_pos.y > window_pos.y + window_size.y) {
                    fullview_Flag = false;
                }
            }

            ImGui::End();
        }



        // Robot_Controls_table();
    }
    
    void Property_Panel::camera_frame()
    {
        ImGui::SetNextWindowSize(ImVec2(ImGui::GetIO().DisplaySize.x * 0.15f, ImGui::GetIO().DisplaySize.y * 0.15f));
        static int axisLength{ 1000 };
        static float newnear{ 1.0f }, newfar{ 10000.0f }; // Far =0 => Render error
        static int newzoom(50); static int gridNum(110); static int gridStep(5);

        if (ImGui::CollapsingHeader("CameraSetting", ImGuiTreeNodeFlags_DefaultOpen))        {
            ImGui::PushItemFlag(ImGuiItemFlags_Disabled, false);
            ImGui::Separator();
            if (ImGui::Button("Advance Settings Vision"))
            {
                std::string VsionPath = "Vision\\CameraSettings\\gui_vision.exe";
                SECURITY_ATTRIBUTES sa;
                sa.nLength = sizeof(SECURITY_ATTRIBUTES);
                sa.bInheritHandle = TRUE;
                sa.lpSecurityDescriptor = NULL;

                // Prepare startup information
                STARTUPINFO si;
                ZeroMemory(&si, sizeof(STARTUPINFO));
                si.cb = sizeof(STARTUPINFO);
                si.dwFlags |= STARTF_USESHOWWINDOW; 
                si.wShowWindow = SW_HIDE;

                // Prepare process information
                PROCESS_INFORMATION pi;
                ZeroMemory(&pi, sizeof(PROCESS_INFORMATION));

                std::string command = VsionPath;

                // Create the process
                if (!CreateProcess(NULL, const_cast<LPSTR>(command.c_str()), NULL, NULL, FALSE, CREATE_NO_WINDOW, NULL, NULL, &si, &pi))
                {
                    DWORD error = GetLastError();
                    *sttlogs << " Failed to create process. Check source file. ";
                    return;
                }
                CloseHandle(pi.hProcess);
                CloseHandle(pi.hThread);
            }
            ImGui::SetNextItemWidth(150);
            ImGui::SliderFloat("Near", &newnear, 0.01f, 10.0f, "%.1f"); ImGui::SetNextItemWidth(150);
            ImGui::SliderFloat("Far", &newfar, 1.0f, 20000.0f, "%.0f"); ImGui::SetNextItemWidth(150);
            ImGui::SliderInt("ZSp", &newzoom, 0, 200); ImGui::SetNextItemWidth(150);
            ImGui::Separator(); ImGui::SetNextItemWidth(150);
            ImGui::SliderInt("GridNum", &gridNum, 0, 200); ImGui::SetNextItemWidth(150);
            ImGui::SliderInt("GridStep", &gridStep, 0, 200); ImGui::SetNextItemWidth(150);
            ImGui::Separator(); ImGui::SetNextItemWidth(150);
            ImGui::SliderInt("AxisLength", &axisLength, 0, 5000);
        }
        mCamera->set_near(newnear);
        mCamera->update_view_matrix();
        mCamera->update(mShader.get());

        mCamera->set_far(newfar);
        mCamera->update_view_matrix();
        mCamera->update(mShader.get());

        SceneView::getInstance().setZoom(newzoom);
        // grid
        rdMesh->createGridSys(gridNum, gridStep);
        rdMesh->set_axis_length(axisLength);
    }
    void Property_Panel::layer_frame()
    {
        if (ImGui::CollapsingHeader("Layer", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (ImGui::IsWindowFocused() || ImGui::IsWindowHovered()) { frameStatus["Layer"] = false; }

            static std::unordered_set<long long> selectedMeshes;
            static int lastSelectedIndex = -1; // To remember the last selected index for range selection
            static std::vector<bool> selectionStates; // To track selection states

            if (rdMesh->getMesh()->size() > 0)
            {
                // Resize selectionStates if necessary
                if (selectionStates.size() != rdMesh->getMesh()->size()) {
                    selectionStates.resize(rdMesh->getMesh()->size(), false);
                }
                for (auto it = rdMesh->getMesh()->begin(); it != rdMesh->getMesh()->end(); it++)
                {
                    selectionStates[it - rdMesh->getMesh()->begin()] = it->get()->selected;
                }
                ImGui::BeginChild("TableChild", ImVec2(0, 100), true, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_AlwaysVerticalScrollbar);
                ImGui::BeginTable("Objects", 3, ImGuiTableFlags_Resizable | ImGuiTableFlags_ScrollX);
                ImGui::TableSetupColumn("Sel");
                ImGui::TableSetupColumn("Name",ImGuiTableColumnFlags_WidthFixed, 80.0f);
                ImGui::TableSetupColumn("Hide");
                ImGui::TableHeadersRow();

                int i = -1; int j = -1;
                for (auto it = rdMesh->getMesh()->begin(); it != rdMesh->getMesh()->end(); it++)
                {
                    auto mesh = *it; i++;
                    if (check_skip(mesh)) { continue; }

                    ImGui::TableNextRow();
                    ImGui::TableNextColumn();

                    bool isSelected = selectionStates[i];
                    if (ImGui::Checkbox(("##Select" + std::to_string(i)).c_str(), &isSelected))
                    {
                        if (ImGui::GetIO().KeyShift && lastSelectedIndex != -1)
                        {
                            int start = (lastSelectedIndex < i) ? lastSelectedIndex : i;
                            int end = (lastSelectedIndex > i) ? lastSelectedIndex : i;
                            bool selectRange = !selectionStates[start];
                            for (auto it1 = rdMesh->getMesh()->begin(); it1 != rdMesh->getMesh()->end(); it1++)
                            {
                                j++;
                                selectionStates[j] = selectRange;
                                auto mesh = *it;
                                mesh->selected = selectRange;
                                if (selectRange) {
                                    selectedMeshes.insert(mesh->ID);
                                }
                                else {
                                    selectedMeshes.erase(mesh->ID);
                                }
                            }
                        }
                        else
                        {
                            selectionStates[i] = isSelected;
                            mesh->selected = isSelected;
                            if (isSelected)
                            {
                                selectedMeshes.insert(mesh->ID);
                                lastSelectedIndex = i;
                            }
                            else
                            {
                                selectedMeshes.erase(mesh->ID);
                            }
                        }
                    }

                    ImGui::TableNextColumn();
                    ImGui::Text(mesh->oname);
                    ImGui::TableNextColumn();
                    ImGui::Checkbox(("##Hide" + std::to_string(i)).c_str(), &mesh->hide);

                    if (selectionStates[i])
                    {
                        ImU32 yellowColorU32 = ImGui::ColorConvertFloat4ToU32(ImVec4(0.25f, 0.42f, 1.0f, 1.0f));
                        ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, yellowColorU32);
                    }
                }
                ImGui::EndTable();
                ImGui::EndChild();

            }
            ImGui::Separator();
        }
    }
    void Property_Panel::obInfo_frame()
    {
        static float pos_x, pos_y, sizex, sizey;
        GetViewPos(pos_x, pos_y);
        get3DSize(sizex, sizey);
        ImGui::SetNextWindowPos(ImVec2(pos_x + 15+ sizex * 0.83, pos_y + 35)); // Set the position of the frame
        ImGui::SetNextWindowSize(ImVec2(sizex * 0.15, sizey * 0.2)); // Set the size of the frame
        ImGui::Begin("Vertices & Vertex Indices", nullptr, 
            ImGuiWindowFlags_NoDocking | // Cannot be docked
            ImGuiWindowFlags_NoBackground | // Do not display background
            ImGuiWindowFlags_NoNavFocus); // Does not bring to front on focus
        //if (theme == "dark")
            //ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.6f, 0.8f, 0.6f, 1.0f)); // Set text color to white and 50% transparent
        //elsei
        
        for (auto it = rdMesh->getMesh()->begin(); it!= rdMesh->getMesh()->end(); it++)
        {
            auto mesh = *it;
            if (check_skip(mesh)) { continue; }            
            ImVec4 color = mesh->selected ? ImVec4(1.0f, 0.0f, 0.0f, 1.0f) : ImVec4(0.27f, 0.48f, 0.39f, 1.0f);
            ImGui::TextColored(color, "%s: %lld - %lld", mesh->oname, mesh->mVertices.size(), mesh->mVertexIndices.size());
        }
        ImGui::End();
    }
    void Property_Panel::coordinate_frame()
    {
        static float posrot_obj[7]; static long long PreObId = 0;
        static char aname[255]; 
        if (ImGui::CollapsingHeader("Coordinates", ImGuiTreeNodeFlags_DefaultOpen)) //&&mesh
        {
            bool pressOk = ImGui::Button("UPDATE");
            if (rdMesh->check_selected() == 1)
            {
                for (auto it = rdMesh->getMesh()->begin(); it != rdMesh->getMesh()->end(); it++)
                {
                    auto mesh = *it;
                    if (mesh->selected && PreObId != mesh->ID)
                    {
                        if (check_skip(mesh)) { continue; }
                        PreObId = mesh->ID;
                        strcpy_s(aname, sizeof(aname), mesh->oname);
                        posrot_obj[0] = mesh->oMaterial.position.x;
                        posrot_obj[1] = mesh->oMaterial.position.y;
                        posrot_obj[2] = mesh->oMaterial.position.z;
                        posrot_obj[3] = mesh->oMaterial.rotation.x;
                        posrot_obj[4] = mesh->oMaterial.rotation.y;
                        posrot_obj[5] = mesh->oMaterial.rotation.z;
                        break;
                    }
                }
                ImGui::Text("Name"); ImGui::SameLine();
                ImGui::InputText("##Name", aname, ImGuiInputTextFlags_EnterReturnsTrue);
                // Text color: Red
                ImGui::TextColored(ImVec4(0.5f, 0.80f, 0.9f, 1.0f), "x_Pos"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
                ImGui::InputFloat("##xPos", &posrot_obj[0], 0.0f, 0.0f, "%.3f"); ImGui::SameLine();

                ImGui::TextColored(ImVec4(0.5f, 0.80f, 0.9f, 1.0f), "x_Rot"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
                ImGui::InputFloat("##xRot", &posrot_obj[3], 0.0f, 0.0f, "%.3f");
                // Text color: Blue
                ImGui::TextColored(ImVec4(0.9f, 0.0f, 0.0f, 1.0f), "y_Pos"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
                ImGui::InputFloat("##yPos", &posrot_obj[1], 0.0f, 0.0f, "%.3f"); ImGui::SameLine();

                ImGui::TextColored(ImVec4(0.9f, 0.0f, 0.0f, 1.0f), "y_Rot"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
                ImGui::InputFloat("##yRot", &posrot_obj[4], 0.0f, 0.0f, "%.3f");
                // Text color: Purple
                ImGui::TextColored(ImVec4(0.75f, 0.72f, 0.25f, 1.0f), "z_Pos"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
                ImGui::InputFloat("##zPos", &posrot_obj[2], 0.0f, 0.0f, "%.3f"); ImGui::SameLine();

                ImGui::TextColored(ImVec4(0.75f, 0.72f, 0.25f, 1.0f), "z_Rot"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
                ImGui::InputFloat("##zRot", &posrot_obj[5], 0.0f, 0.0f, "%.3f");

                if (pressOk)
                {
                    for (auto it = rdMesh->getMesh()->begin(); it!= rdMesh->getMesh()->end(); it++)
                    {
                        auto mesh = *it; 
                        if (!check_skip(mesh)) {
                            if (mesh->selected == true) {  
                                float movex = posrot_obj[0] - mesh->oMaterial.position.x;
                                float movey = posrot_obj[1] - mesh->oMaterial.position.y;
                                float movez = posrot_obj[2] - mesh->oMaterial.position.z;
                                float rotatex = posrot_obj[3] - mesh->oMaterial.rotation.x;
                                float rotatey = posrot_obj[4] - mesh->oMaterial.rotation.y;
                                float rotatez = posrot_obj[5] - mesh->oMaterial.rotation.z;
                                if (std::abs(movex) > 0.01 || std::abs(movey) > 0.01 || std::abs(movez) > 0.01)
                                {
                                    uiaction.MoveOb_uiAction(movex, movey, movez);
                                }
                                if (std::abs(rotatex) > 0.01 || std::abs(rotatey) > 0.01 || std::abs(rotatez) > 0.01)
                                {

                                    uiaction.RotateOb_uiAction(rotatex, rotatey, rotatez);
                                }
                                // reset all posrot & aname;
                                std::fill_n(posrot_obj, 6, 0.0f);
                                PreObId = 0;
                            } 
                        }
                    }                    
                }
            }
            else
            {
                PreObId = 0;
                static float posrot[7];
                static char aname[50] = "";
                ImGui::Text("Name"); ImGui::SameLine(); ImGui::SetNextItemWidth(150);
                ImGui::InputText("##Name", aname, ImGuiInputTextFlags_EnterReturnsTrue);

                ImGui::TextColored(ImVec4(0.5f, 0.80f, 0.9f, 1.0f), "x_Pos"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
                ImGui::InputFloat("##xPos", &posrot[0], 0.0f, 0.0f, "%.3f"); ImGui::SameLine();
                ImGui::TextColored(ImVec4(0.5f, 0.80f, 0.9f, 1.0f), "x_Rot"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
                ImGui::InputFloat("##xRot", &posrot[3], 0.0f, 0.0f, "%.3f");

                ImGui::TextColored(ImVec4(0.9f, 0.0f, 0.0f, 1.0f), "y_Pos"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
                ImGui::InputFloat("##yPos", &posrot[1], 0.0f, 0.0f, "%.3f"); ImGui::SameLine();
                ImGui::TextColored(ImVec4(0.9f, 0.0f, 0.0f, 1.0f), "y_Rot"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
                ImGui::InputFloat("##yRot", &posrot[4], 0.0f, 0.0f, "%.3f");

                ImGui::TextColored(ImVec4(0.75f, 0.72f, 0.25f, 1.0f), "z_Pos"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
                ImGui::InputFloat("##zPos", &posrot[2], 0.0f, 0.0f, "%.3f"); ImGui::SameLine();
                ImGui::TextColored(ImVec4(0.75f, 0.72f, 0.25f, 1.0f), "z_Rot"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
                ImGui::InputFloat("##zRot", &posrot[5], 0.0f, 0.0f, "%.3f");

                if (rdMesh->check_selected() != 0)
                {
                    if (strlen(aname) != 0 && pressOk)
                    {
                        for (auto it = rdMesh->getMesh()->begin(); it != rdMesh->getMesh()->end(); it++)
                        {
                            auto mesh = *it;
                            if (check_skip(mesh)) { continue; }
                            if (mesh->selected) {
                                strncpy_s(mesh->oname, aname, sizeof(mesh->oname) - 1);
                                mesh->oname[sizeof(mesh->oname) - 1] = '\0';
                            }
                        }
                    }
                    // check if any elements of posrot is not 0
                    bool hasNonZeroElement = std::any_of(std::begin(posrot), std::end(posrot),
                        [](float value) { return value != 0.0f; });
                    if (hasNonZeroElement && pressOk)
                    {
                        // move and rotate the object if posrot is not 0
                        uiaction.MoveOb_uiAction(posrot[0], posrot[1], posrot[2]);
                        uiaction.RotateOb_uiAction(posrot[3], posrot[4], posrot[5]);
                        // reset all posrot & aname;
                        std::fill_n(posrot, 6, 0.0f);
                        std::fill_n(aname, sizeof(aname), '\0'); PreObId = 0;
                    }
                }
            }

        }
    }
    void Property_Panel::material_frame() {

        static ImVec4 clor = ImVec4(0.0f, 1.0f, 1.0f, 1.0f);
        static float rness = 0.5f; static float mlic = 0.5f;
        if (ImGui::CollapsingHeader("Material", ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImVec4 preClor = clor; float prerness = rness; float premlic = mlic;
            for (auto it = rdMesh->getMesh()->begin(); it != rdMesh->getMesh()->end(); it++)
            {
                auto mesh = *it;
                if (check_skip(mesh)) { continue; }
                if (mesh->selected == true)
                {
                    clor = ImVec4(mesh->oMaterial.mColor.x, mesh->oMaterial.mColor.y, mesh->oMaterial.mColor.z, 1.0f);
                    rness = mesh->oMaterial.mRoughness;
                    mlic = mesh->oMaterial.mMetallic;
                }
            }

            ImGui::SetNextItemWidth(150);
            ImGui::ColorEdit3("Color", (float*)&clor); ImGui::SetNextItemWidth(150);
            ImGui::SliderFloat("Roughness", &rness, 0.0f, 1.0f); ImGui::SetNextItemWidth(150);
            ImGui::SliderFloat("Metallic", &mlic, 0.0f, 1.0f); ImGui::SetNextItemWidth(150);
            for (auto it = rdMesh->getMesh()->begin(); it != rdMesh->getMesh()->end(); it++)
            {
                auto mesh = *it;
                if (mesh->selected == true)
                {
                    if ((preClor.x != clor.x || preClor.y != clor.y || preClor.z != clor.z || preClor.w != clor.w)) {
                        std::cout << mesh->oname << std::endl;
                        mesh->oMaterial.mColor = glm::vec3(clor.x, clor.y, clor.z);
                    }
                    if (prerness != rness) {
                        mesh->oMaterial.mRoughness = rness;
                    }
                    if (premlic != mlic) {
                        mesh->oMaterial.mMetallic = mlic;
                    }
                }
            }
        }
        /// Light
        if (ImGui::CollapsingHeader("Light", ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::Separator(); ImGui::SetNextItemWidth(150);
            nimgui::draw_vec3_widget("Position", mLight->mPosition, 80.0f); ImGui::SetNextItemWidth(150);
            static const char* items[] = { "Single Point Light", "WorldBox 8 Lights","WorldBox 32 Lights" ,"NoLights","Normal Adjustment"};
            ImGui::Combo("SetLight", &mLight->lightmode, items, IM_ARRAYSIZE(items)); ImGui::SetNextItemWidth(150);
            ImGui::SliderFloat("Light Intensity", &mLight->mStrength, 200.00f, 2000.0f); ImGui::SetNextItemWidth(150);
            if (mLight->lightmode == 4) { ImGui::SliderInt("Normal Vector: ", &mLight->InspectAngle, 1, 80); }

            /*if (mLight->mStrength == 0)
            {
                ImGui::TextWrapped("Light is off. Render by Shaded mode.");
            }
            else
            {
                ImGui::TextWrapped("Light is on. Normal Render mode.");
            }*/
        }


    }
    bool Property_Panel::check_skip(const std::shared_ptr<nelems::oMesh>& mesh)
    {
        if (std::string(mesh->oname).find("RBSIMBase_") != std::string::npos) { return true; }
		if (std::string(mesh->oname).find("movepath__SKIP__") != std::string::npos) { return true; }
        return false;
    }
    void Property_Panel::sh_performance()
    {
        static int x{ 200 }, y{ 200 };
        static float pos_x, pos_y;
        GetViewPos(pos_x, pos_y);

        static auto start = std::chrono::high_resolution_clock::now();
        static auto countime = std::chrono::high_resolution_clock::now();
        static std::vector<float> fps; 
        auto end = std::chrono::high_resolution_clock::now();

        std::chrono::duration<double> elapsed = end - start;
        start = end;
        float current_fps = std::round((1.0f / static_cast<float>(elapsed.count())) * 10) / 10.0f;

        if ( ( end- countime ).count() > 2.0f  )
        {
            if (current_fps > 120 ){current_fps = 120.0f;}
            else if (current_fps > 60) { current_fps = 60.0f; }

            if (fps.size() < 100) { fps.push_back(current_fps); }
            else { fps.erase(fps.begin()); fps.push_back(current_fps); }
            countime = end;
        }
        

        ImGui::SetNextWindowBgAlpha(0.0f);
        ImGui::SetNextWindowPos(ImVec2(pos_x +215, pos_y + 35));
        ImGui::SetNextWindowSize(ImVec2(200, 0), ImGuiCond_Always);

        ImGui::Begin("##Performance", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove);

        ImGui::PlotLines("", fps.data(), static_cast<int>(fps.size()));
        ImGui::SameLine();

        std::ostringstream ss;
        ss << std::fixed << std::setprecision(0) << current_fps;
        ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "FPS %s", ss.str().c_str());

        ImGui::End();

    }
}