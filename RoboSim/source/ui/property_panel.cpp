#include "pch.h"
#include "property_panel.h"
#include <filesystem> 
#include <ctime>
#include "ui/HotkeyMenubar.h"

#include "rigging/rigging.h"
#include "rigging/point.h"
#include <cmath>
#include "elems/TranRBMatrix.h"
#include <cmath>
#include "chrono"

#include "thread"
#include "mutex"
using namespace Eigen;

namespace nui
{
    // long long nui::Property_Panel::selectedID = 0;
    void Property_Panel::render(nui::SceneView* scene_view, GLFWwindow* mWindow)
    {
        if (!proMesh) {proMesh = &nelems::mMesh::getInstance();}

        mRobot->render();

        static nui::HotkeyMenubar hotkey_manage;
        hotkey_manage.mMenuBar(mWindow);
        hotkey_manage.mHotkey(mWindow);

        IPreceiver->render();
        
        ImGui::Begin("Properties");
        layer_frame(scene_view); // define selectedID
        coordinate_frame(); // show the position
        material_frame(scene_view); // show material properties
        camera_frame(scene_view); // show camera properties
        ImGui::End();

        obInfo_frame(); // show object info such as vertices and vertex 
        ImGui::Begin("StatusLogs", nullptr, ImGuiWindowFlags_HorizontalScrollbar | ImGuiWindowFlags_AlwaysVerticalScrollbar);
        ImGui::TextWrapped(sttlogs->getStatus().c_str());
        ImGui::End();


        // sh_performance();

        Robot_Controls_table();
    }
    
    void Property_Panel::camera_frame(nui::SceneView* scene_view)
    {
        ImGui::SetNextWindowSize(ImVec2(ImGui::GetIO().DisplaySize.x * 0.15f, ImGui::GetIO().DisplaySize.y * 0.15f));
        static int axisLength{ 1000 };
        static float newnear{ 1.0f }, newfar{ 10000.0f }; // Far =0 => Render error
        static int newzoom(50); static int gridNum(15); static int gridStep(40);

        if (ImGui::CollapsingHeader("CameraSetting", false))        {
            ImGui::PushItemFlag(ImGuiItemFlags_Disabled, false);
            ImGui::Separator();
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
        SceneView::getInstance().setNear(newnear);
        SceneView::getInstance().setFar(newfar);
        SceneView::getInstance().setZoom(newzoom);
        // grid
        proMesh->createGridSys(gridNum, gridStep);
        proMesh->set_axis_length(axisLength);
    }
    void Property_Panel::layer_frame(nui::SceneView* scene_view)
    {
        if (ImGui::CollapsingHeader("Layer", ImGuiTreeNodeFlags_DefaultOpen)) {
            nui::FrameManage::setCrActiveGui("Layer", ImGui::IsWindowFocused() || ImGui::IsWindowHovered());

            static int lastSelectedIndex = -1; // To remember the last selected index for range selection
            static std::vector<bool> selectionStates; // To track selection states

            if (proMesh->getMesh()->size() > 0)
            {
                // Resize selectionStates if necessary
                if (selectionStates.size() != proMesh->getMesh()->size()) {
                    selectionStates.resize(proMesh->getMesh()->size(), false);
                }

                ImGui::BeginChild("TableChild", ImVec2(0, 180), true, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_AlwaysVerticalScrollbar);
                ImGui::BeginTable("Objects", 3, ImGuiTableFlags_Resizable | ImGuiTableFlags_ScrollX);
                ImGui::TableSetupColumn("Sel");
                ImGui::TableSetupColumn("Name");
                ImGui::TableSetupColumn("Hide");
                ImGui::TableHeadersRow();

                int i = -1; int j = -1;
                for (auto it = proMesh->getMesh()->begin(); it != proMesh->getMesh()->end(); it++)
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
                            for (auto it1 = proMesh->getMesh()->begin(); it1 != proMesh->getMesh()->end(); it1++)
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
        nui::FrameManage::getViewportSize(pos_x, pos_y);
        nui::FrameManage::get3DSize(sizex, sizey);
        ImGui::SetNextWindowPos(ImVec2(pos_x + 15+ sizex * 0.83, pos_y + 35)); // Set the position of the frame
        ImGui::SetNextWindowSize(ImVec2(sizex * 0.15, sizey * 0.2)); // Set the size of the frame
        ImGui::Begin("Vertices & Vertex Indices", nullptr, 
            ImGuiWindowFlags_NoDocking | // Cannot be docked
            ImGuiWindowFlags_NoBackground | // Do not display background
            ImGuiWindowFlags_NoNavFocus); // Does not bring to front on focus
        //if (theme == "dark")
            //ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.6f, 0.8f, 0.6f, 1.0f)); // Set text color to white and 50% transparent
        //elsei
        
        for (auto it = proMesh->getMesh()->begin(); it!= proMesh->getMesh()->end(); it++)
        {
            auto mesh = *it;
            if (check_skip(mesh)) { continue; }            
            ImVec4 color = mesh->selected ? ImVec4(1.0f, 0.0f, 0.0f, 1.0f) : ImVec4(0.27f, 0.78f, 0.69f, 1.0f);
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
            if (proMesh->check_selected() == 1)
            {
                for (auto it = proMesh->getMesh()->begin(); it != proMesh->getMesh()->end(); it++)
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
                ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "x_Pos"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
                ImGui::InputFloat("##xPos", &posrot_obj[0], 0.0f, 0.0f, "%.3f"); ImGui::SameLine();

                ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "x_Rot"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
                ImGui::InputFloat("##xRot", &posrot_obj[3], 0.0f, 0.0f, "%.3f");
                // Text color: Blue
                ImGui::TextColored(ImVec4(0.0f, 0.0f, 1.0f, 1.0f), "y_Pos"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
                ImGui::InputFloat("##yPos", &posrot_obj[1], 0.0f, 0.0f, "%.3f"); ImGui::SameLine();

                ImGui::TextColored(ImVec4(0.0f, 0.0f, 1.0f, 1.0f), "y_Rot"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
                ImGui::InputFloat("##yRot", &posrot_obj[4], 0.0f, 0.0f, "%.3f");
                // Text color: Purple
                ImGui::TextColored(ImVec4(0.5f, 0.0f, 0.5f, 1.0f), "z_Pos"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
                ImGui::InputFloat("##zPos", &posrot_obj[2], 0.0f, 0.0f, "%.3f"); ImGui::SameLine();

                ImGui::TextColored(ImVec4(0.5f, 0.0f, 0.5f, 1.0f), "z_Rot"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
                ImGui::InputFloat("##zRot", &posrot_obj[5], 0.0f, 0.0f, "%.3f");

                if (pressOk)
                {
                    for (auto it = proMesh->getMesh()->begin(); it!= proMesh->getMesh()->end(); it++)
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

                ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "x_Pos"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
                ImGui::InputFloat("##xPos", &posrot[0], 0.0f, 0.0f, "%.3f"); ImGui::SameLine();
                ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "x_Rot"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
                ImGui::InputFloat("##xRot", &posrot[3], 0.0f, 0.0f, "%.3f");

                ImGui::TextColored(ImVec4(0.0f, 0.0f, 1.0f, 1.0f), "y_Pos"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
                ImGui::InputFloat("##yPos", &posrot[1], 0.0f, 0.0f, "%.3f"); ImGui::SameLine();
                ImGui::TextColored(ImVec4(0.0f, 0.0f, 1.0f, 1.0f), "y_Rot"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
                ImGui::InputFloat("##yRot", &posrot[4], 0.0f, 0.0f, "%.3f");

                ImGui::TextColored(ImVec4(0.5f, 0.0f, 0.5f, 1.0f), "z_Pos"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
                ImGui::InputFloat("##zPos", &posrot[2], 0.0f, 0.0f, "%.3f"); ImGui::SameLine();
                ImGui::TextColored(ImVec4(0.5f, 0.0f, 0.5f, 1.0f), "z_Rot"); ImGui::SameLine(); ImGui::SetNextItemWidth(50);
                ImGui::InputFloat("##zRot", &posrot[5], 0.0f, 0.0f, "%.3f");

                if (proMesh->check_selected() != 0)
                {
                    if (strlen(aname) != 0 && pressOk)
                    {
                        for (auto it = proMesh->getMesh()->begin(); it != proMesh->getMesh()->end(); it++)
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
    void Property_Panel::material_frame(nui::SceneView* scene_view) {

        static ImVec4 clor = ImVec4(0.0f, 1.0f, 1.0f, 1.0f);
        static float rness = 0.5f; static float mlic = 0.5f;
        if (ImGui::CollapsingHeader("Material", false))
        {
            ImVec4 preClor = clor; float prerness = rness; float premlic = mlic;
            for (auto it = proMesh->getMesh()->begin(); it != proMesh->getMesh()->end(); it++)
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
            for (auto it = proMesh->getMesh()->begin(); it != proMesh->getMesh()->end(); it++)
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
        if (ImGui::CollapsingHeader("Light", false))
        {
            ImGui::Separator(); ImGui::SetNextItemWidth(150);
            nimgui::draw_vec3_widget("Position", scene_view->get_light()->mPosition, 80.0f); ImGui::SetNextItemWidth(150);
            static const char* items[] = { "Single Point Light", "WorldBox 8 Lights","WorldBox 32 Lights" ,"NoLights" };
            ImGui::Combo("SetLight", &scene_view->get_light()->lightmode, items, IM_ARRAYSIZE(items)); ImGui::SetNextItemWidth(150);
            ImGui::SliderFloat("Light Intensity", &scene_view->get_light()->mStrength, 0.00f, 1000.0f);

            if (scene_view->get_light()->mStrength == 0)
            {
                ImGui::TextWrapped("Light is off. Render by Shaded mode.");
            }
            else
            {
                ImGui::TextWrapped("Light is on. Normal Render mode.");
            }
        }


    }
    void SaveIniFile(const std::string& key, const std::string& value) {
        std::ifstream inputFile("robosim_ini.dat");
        json j;
        if (inputFile.is_open()) {
            inputFile >> j;
            inputFile.close();
        }

        j[key] = value;

        std::ofstream outputFile("robosim_ini.dat");
        if (!outputFile.is_open()) {
            std::cerr << "Failed to open file for writing: robosim_ini.dat" << std::endl;
            return;
        }
        outputFile << j.dump(4);
        outputFile.close();

        std::cout << "Theme saved to robosim_ini.dat" << std::endl;
        MessageBox(NULL, "Please restart the software to apply the new theme.", "Restart required", MB_OK);
    }
    void Property_Panel::draft_chart()
    {
        ImGui::Begin("Draft Chart", nullptr);
        ImGui::Text("Draft Chart");

        ImGui::End();
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
        nui::FrameManage::getViewportSize(pos_x, pos_y);

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
    void nui::Property_Panel::Robot_Controls_table()
    {
        // *****************************************************
        // A - Get base objects if not already retrieved
        if (base[0] == nullptr)
        {
            for (auto it = proMesh->getMesh()->begin(); it != proMesh->getMesh()->end(); it++)
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
        static float tolerance = 0.1f;
        static float ang[6]{ 0 };
        static float pre[6]{ 0 };
        static float prehand[3]{ 0 };
        static std::vector<std::vector <float>> limangle{ 6, {-180,180} };
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

        static bool CtrFlag = false;
        if (ImGui::BeginPopupContextItem("Robot Controls Popup", ImGuiPopupFlags_MouseButtonRight)) {
            if (ImGui::MenuItem("Toggle Control Flag")) {
                CtrFlag = !CtrFlag;                mRobot->setSwitchVisualize();
            }
            ImGui::EndPopup();
        }

        prehand[0] = base[5]->oMaterial.position.x;
        prehand[1] = base[5]->oMaterial.position.y;
        prehand[2] = base[5]->oMaterial.position.z;
        // *****************************************************
        // C - Livesync & Control mode 
        // LiveSync Mode: 
        if (CtrFlag == false) {
            // Get joint angles
            mRobot->get_angle(ang[0], ang[1], ang[2], ang[3], ang[4], ang[5]);
            // Show Joints Data
            ImVec4 vecred(0.0f, 0.0f, 1.0f, 1.0f); 

            for (int i = 0; i < 6; ++i) {
                // Start a new child for each joint group
                ImGui::BeginChild((std::string("JointGroup") + std::to_string(i)).c_str(), ImVec2(110, 85), true);
                ImGui::BeginGroup();

                // Display joint angle
                ImGui::TextColored(vecred, "Joint %d: %.2f", i + 1, ang[i]);

                // Display limits
                ImGui::Text("Min: "); ImGui::SameLine();
                ImGui::SetNextItemWidth(50);
                ImGui::InputFloat((std::string("##") + std::to_string(i) + "_0").c_str(), &limangle[i][0], 0, 0, "%.2f");

                ImGui::Text("Max:"); ImGui::SameLine();
                ImGui::SetNextItemWidth(50);
                ImGui::InputFloat((std::string("##") + std::to_string(i) + "_1").c_str(), &limangle[i][1], 0, 0, "%.2f");

                // End the group
                ImGui::EndGroup();
                ImGui::EndChild();

                // Add some spacing between groups
                if (i % 3 == 2) {
                    //ImGui::Dummy(ImVec2(5.0f, 5.0f));  // Add a larger space after each row of three joints
                }
                else {
                    ImGui::SameLine();  // Adjust spacing as needed
                }
            }


            mRobot->set_limitangle(limangle);
        }
        // Control Mode:
        else
        {

            ImGui::SetNextItemWidth(100);
            ImGui::InputFloat("Joint 1", &ang[0], 1, 0.1, "%.2f");
            ImGui::SetNextItemWidth(100);
            ImGui::InputFloat("Joint 2", &ang[1], 1, 0.1, "%.2f");
            ImGui::SetNextItemWidth(100);
            ImGui::InputFloat("Joint 3", &ang[2], 1, 0.1, "%.2f");
            ImGui::SetNextItemWidth(100);
            ImGui::InputFloat("Joint 4", &ang[3], 1, 0.1, "%.2f");
            ImGui::SetNextItemWidth(100);
            ImGui::InputFloat("Joint 5", &ang[4], 1, 0.1, "%.2f");
            ImGui::SetNextItemWidth(100);
            ImGui::InputFloat("Joint 6", &ang[5], 1, 0.1, "%.2f");
            ImGui::Separator();

            if (mRobot->getSwitchVisualize()) { CtrFlag = false; }
        }
        //*****************************************************
        // D - Caclculate for simulate the movement

        // D - 2 Joints Siumulation
        bool exceeds_tolerance = false;
        for (int i = 0; i < 6; ++i) {
            if (std::abs(ang[i] - pre[i]) > tolerance) {
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
            rotateJoint(5, ang[5], pre[5], tolerance, base, -(ang[5] - pre[5]), 0, 0);
            rotateJoint(4, ang[4], pre[4], tolerance, base, 0, -(ang[4] - pre[4]), 0);
            rotateJoint(3, ang[3], pre[3], tolerance, base, -(ang[3] - pre[3]), 0, 0);
            rotateJoint(2, ang[2], pre[2], tolerance, base, 0, -(ang[2] - pre[2]), 0);
            rotateJoint(1, ang[1], pre[1], tolerance, base, 0, (ang[1] - pre[1]), 0);
            rotateJoint(0, ang[0], pre[0], tolerance, base, 0, 0, (ang[0] - pre[0]));
        }


        // Create buffers for each base
        for (auto& bs : base) 
        { 
            bs->delete_buffers();
            bs->create_buffers(); }
        ImGui::End();
    }
    void nui::Property_Panel::rotateJoint(size_t jointIndex, float& ang, float& pre, const float tolerance,
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


        if (std::abs(ang - pre) > tolerance) {

            //GPU GLSL
            if (mctshader)
            {
                /// Future if necessary
            }
            //CPU
            else
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
            }

            pre = std::round(ang * 100.0f) / 100.0f;
        }
    }
}