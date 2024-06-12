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


using namespace Eigen;

namespace nui
{
    // long long nui::Property_Panel::selectedID = 0;
    void Property_Panel::render(nui::SceneView* scene_view, GLFWwindow* mWindow)
    {
        if (!proMesh) {
            proMesh = &nelems::mMesh::getInstance();
        }
        // robotic arm
        if (!mRobot)
        {
            mRobot = &nymrobot::ymconnect::getInstance();
        }
        //****************************************************

        static nui::HotkeyMenubar hotkey_manage;
        hotkey_manage.mMenuBar(mWindow);
        hotkey_manage.mHotkey(mWindow);
        hotkey_manage.commandLogs(); // show command logs

        //****************************************************
        //Main Properties
        layer_frame(scene_view); // define selectedID

        if (ImGui::Begin("Properties"))
        {
            
            obInfo_frame(); // show object info such as vertices and vertex indices
            ImGui::Separator();
            coordinate_frame(); // show the position
            material_frame(scene_view); // show material properties
            ImGui::End();

        }
        mRobot->render();
        camera_frame(scene_view); // show camera properties
        
        Robot_Controls_table();
    }
    ////===========================================================================================
    //// Main Frames 
    void Property_Panel::camera_frame(nui::SceneView* scene_view)
    {
        ImGui::SetNextWindowSize(ImVec2(ImGui::GetIO().DisplaySize.x * 0.15f, ImGui::GetIO().DisplaySize.y * 0.15f));
        if (ImGui::Begin("CameraSetting", nullptr))
        {
            ImGui::PushItemFlag(ImGuiItemFlags_Disabled, false);
            ImGui::Separator();
            static float newnear{ 1.0f }, newfar{ 10000.0f }; // Far =0 => Render error
            static int newzoom(50); static int gridNum(15); static int gridStep(40);
            ImGui::SetNextItemWidth(150);
            ImGui::SliderFloat("Near", &newnear, 0.01f, 10.0f, "%.1f"); ImGui::SetNextItemWidth(150);
            ImGui::SliderFloat("Far", &newfar, 1.0f, 20000.0f, "%.0f"); ImGui::SetNextItemWidth(150);
            ImGui::SliderInt("ZSp", &newzoom, 0, 200); ImGui::SetNextItemWidth(150);
            ImGui::Separator(); ImGui::SetNextItemWidth(150);
            ImGui::SliderInt("GridNum", &gridNum, 0, 200); ImGui::SetNextItemWidth(150);
            ImGui::SliderInt("GridStep", &gridStep, 0, 200); ImGui::SetNextItemWidth(150);
            ImGui::Separator(); ImGui::SetNextItemWidth(150);
            static int axisLength{ 1000 };
            ImGui::SliderInt("AxisLength", &axisLength, 0, 5000);


            SceneView::getInstance().setNear(newnear);
            SceneView::getInstance().setFar(newfar);
            SceneView::getInstance().setZoom(newzoom);
            ImGui::End();
            // grid
            proMesh->createGridSys(gridNum, gridStep);
            proMesh->set_axis_length(axisLength);

        }
    }
    void Property_Panel::layer_frame(nui::SceneView* scene_view)
    {
        ImGui::Begin("Layer", nullptr);
        nui::FrameManage::setCrActiveGui("Layer", ImGui::IsWindowFocused() || ImGui::IsWindowHovered());

        static int lastSelectedIndex = -1; // To remember the last selected index for range selection
        static std::vector<bool> selectionStates; // To track selection states

        if (proMesh->size() > 0)
        {
            // Resize selectionStates if necessary
            if (selectionStates.size() != proMesh->size()) {
                selectionStates.resize(proMesh->size(), false);
            }

            ImGui::SetNextWindowSize(ImVec2(ImGui::GetIO().DisplaySize.x * 0.15f, ImGui::GetIO().DisplaySize.y * 0.15f));
            ImGui::BeginTable("Objects", 3, ImGuiTableFlags_Resizable | ImGuiTableFlags_ScrollX);
            ImGui::TableSetupColumn("Sel");
            ImGui::TableSetupColumn("Name");
            ImGui::TableSetupColumn("Hide");
            ImGui::TableHeadersRow();

            for (int i = 0; i < proMesh->size(); i++)
            {
                auto mesh = proMesh->get_mesh_ptr(i);
                if (std::string(mesh->oname).find("RBSIMBase_") != std::string::npos) { continue; }
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

                        for (int j = start; j <= end; j++)
                        {
                            selectionStates[j] = selectRange;
                            auto mesh = proMesh->get_mesh_ptr(j);
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
        }
        ImGui::Separator();
        ImGui::End();
    }



    void Property_Panel::obInfo_frame()
    {
        static int x{ 200 }, y{ 200 };
        static float pos_x, pos_y;
        nui::FrameManage::getViewportSize(pos_x, pos_y); 
        ImGui::SetNextWindowPos(ImVec2(pos_x + 15, pos_y + 35)); // Set the position of the frame
        ImGui::SetNextWindowSize(ImVec2(x, y)); // Set the size of the frame
        ImGui::Begin("H1", nullptr,
            ImGuiWindowFlags_NoTitleBar | // Do not display title bar
            ImGuiWindowFlags_NoCollapse | // Cannot collapse
            ImGuiWindowFlags_NoDocking | // Cannot be docked
            ImGuiWindowFlags_NoBackground | // Do not display background
            ImGuiWindowFlags_NoNavFocus); // Does not bring to front on focus
        //if (theme == "dark")
            //ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.6f, 0.8f, 0.6f, 1.0f)); // Set text color to white and 50% transparent
        //elsei


        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.7f, 0.3f, 0.0f, 1.0f)); // Set text color to red and 50% transparent
        ImGui::Text("Vertices & Vertex Indices"); ImGui::Separator();
        if (proMesh->check_selected() != 0)
        {
            for (int i = 0; i < proMesh->size(); i++)
            {
                mesh = proMesh->get_mesh_ptr(i);
                if (mesh->selected == true)
                {
                    ImGui::Text(mesh->oname); ImGui::SameLine();
                    ImGui::Text(": %lld - %lld", mesh->mVertices.size(), mesh->mVertexIndices.size());
                }
            }
        }
        else
        {
            for (int i = 0; i < proMesh->size(); i++) {
                mesh = proMesh->get_mesh_ptr(i);
                if (std::string(mesh->oname).find("RBSIMBase_") != std::string::npos) { continue; }
                ImGui::Text(mesh->oname); ImGui::SameLine();
                ImGui::Text(": %lld - %lld", mesh->mVertices.size(), mesh->mVertexIndices.size());
            }
        }
        ImGui::PopStyleColor();
        x = ImGui::GetWindowWidth(); y = ImGui::GetWindowHeight();
        ImGui::End();
    }
    void Property_Panel::coordinate_frame()
    {

        if (ImGui::CollapsingHeader("Coordinates", ImGuiTreeNodeFlags_DefaultOpen)) //&&mesh
        {
            ImGui::Separator();
            ImGui::Text("Add arrow");
            ImGui::Text("Add front, right and top view");
            ImGui::Text("Add Delete = delete objects");
            ImGui::Separator();
            bool pressOk = ImGui::Button("UPDATE");
            if (proMesh->check_selected() == 1)
            {
                static float posrot_obj[7]; static long long PreObId = 0;
                static char aname[255];
                for (int i = 0; i < proMesh->size(); i++)
                {
                    mesh = proMesh->get_mesh_ptr(i);
                    if (mesh->selected && PreObId != mesh->ID)
                    {
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
                    for (int i = 0; i < proMesh->size(); i++)
                    {
                        mesh = proMesh->get_mesh_ptr(i);
                        if (mesh->selected) { break; }
                    }
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
            else
            {
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
                        // change name if aname is not empty
                        for (int j = 0; j < proMesh->size(); j++)
                        {
                            mesh = proMesh->get_mesh_ptr(j);
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
                        std::fill_n(aname, sizeof(aname), '\0');
                    }
                }
            }

        }
    }
    void Property_Panel::material_frame(nui::SceneView* scene_view) {

        static ImVec4 clor = ImVec4(0.0f, 1.0f, 1.0f, 1.0f);
        static float rness = 0.5f; static float mlic = 0.5f;
        if (ImGui::CollapsingHeader("Material", ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImVec4 preClor = clor; float prerness = rness; float premlic = mlic;
            for (int i = 0; i < proMesh->size(); i++)
            {
                mesh = proMesh->get_mesh_ptr(i);
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
            for (int i = 0; i < proMesh->size(); i++)
            {
                mesh = proMesh->get_mesh_ptr(i);
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
            nimgui::draw_vec3_widget("Position", scene_view->get_light()->mPosition, 80.0f); ImGui::SetNextItemWidth(150);
            static const char* items[] = { "Single Point Light", "WorldBox 8 Lights", "NoLights"};
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

    //================================================================================================

    void nui::Property_Panel::Robot_Controls_table()
    {
        // *****************************************************
        // Get base objects if not already retrieved
        if (base[0] == nullptr)
        {
            for (int i = 0; i < proMesh->size(); i++)
            {
                mesh = proMesh->get_mesh_ptr(i);
                std::string name = std::string(mesh->oname);
                if      (name.find("RBSIMBase_1") != std::string::npos) { base[0] = std::move(mesh); }
                else if (name.find("RBSIMBase_2") != std::string::npos) { base[1] = std::move(mesh); }
                else if (name.find("RBSIMBase_3") != std::string::npos) { base[2] = std::move(mesh); }
                else if (name.find("RBSIMBase_4") != std::string::npos) { base[3] = std::move(mesh); }
                else if (name.find("RBSIMBase_5") != std::string::npos) { base[4] = std::move(mesh); }
                else if (name.find("RBSIMBase_6") != std::string::npos) { base[5] = std::move(mesh); }
            }
        }

        // If no base objects found, return
        if (base[0] == nullptr) { return; }

        // *****************************************************
        // Initialize static variables for joint angles
        static float tolerance = 0.1f;
        static float ang1 = 0, ang2 = 0, ang3 = 0, ang4 = 0, ang5 = 0, ang6 = 0;
        static float pre1 = 0, pre2 = 0, pre3 = 0, pre4 = 0, pre5 = 0, pre6 = 0;
        static float tem1 = 0, tem2 = 0, tem3 = 0, tem4 = 0, tem5 = 0, tem6 = 0;

        // OrgBase is used as the main rotation reference to avoid overlapping matrix calculations
        static std::vector<std::shared_ptr<nelems::oMesh>> OrgBase = {
            std::make_shared<nelems::oMesh>(*base[0]),
            std::make_shared<nelems::oMesh>(*base[1]),
            std::make_shared<nelems::oMesh>(*base[2]),
            std::make_shared<nelems::oMesh>(*base[3]),
            std::make_shared<nelems::oMesh>(*base[4]),
            std::make_shared<nelems::oMesh>(*base[5])
        };

        std::vector<std::shared_ptr<nelems::oMesh>> Joints = {
            std::make_shared<nelems::oMesh>(*base[0]),
            std::make_shared<nelems::oMesh>(*base[1]),
            std::make_shared<nelems::oMesh>(*base[2]),
            std::make_shared<nelems::oMesh>(*base[3]),
            std::make_shared<nelems::oMesh>(*base[4]),
            std::make_shared<nelems::oMesh>(*base[5])
        };

        ImGui::Begin("Robot Controls", nullptr);
        static bool CtrFlag = false;
        if (ImGui::Button(CtrFlag ? "Visualize" : "LiveSync")) { CtrFlag = !CtrFlag; }

        // LiveSync Mode: 
        if (CtrFlag == false) { 
            // *****************************************************
            // Set Switching Color
            static auto lastSwitchTime = std::chrono::steady_clock::now();
            static bool useRed = true;
            ImVec4 redcode = ImVec4(1.0f, 0.0f, 0.0f, 1.0f);
            ImVec4 bluecode = ImVec4(0.0f, 0.0f, 1.0f, 1.0f);

            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::seconds>(now - lastSwitchTime).count() >= 0.5)
            {  useRed = !useRed;   lastSwitchTime = now;  }

            // Get joint angles
            mRobot->get_angle(ang1, ang2, ang3, ang4, ang5, ang6);

            // Show Joints Data
            ImVec4 vecred = useRed ? redcode : bluecode;
            ImGui::TextColored(vecred, "Joint 1: %.2f", ang1); ImGui::SameLine();
            ImGui::TextColored(vecred, "Joint 4: %.2f", ang4);
            ImGui::TextColored(vecred, "Joint 2: %.2f", ang2); ImGui::SameLine();
            ImGui::TextColored(vecred, "Joint 5: %.2f", ang5);
            ImGui::TextColored(vecred, "Joint 3: %.2f", ang3); ImGui::SameLine();
            ImGui::TextColored(vecred, "Joint 6: %.2f", ang6);
            mRobot->trigger_call_move(false);
        }
        // Visualize Mode:
        else
        {
            mRobot->trigger_call_move(true);
            // UI for controlling joint angles
            ImGui::SetNextItemWidth(100);
            ImGui::SliderFloat("Joint 1", &ang1, -180.0f, 180.0f, "%.2f");

            ImGui::SetNextItemWidth(100);
            ImGui::SliderFloat("Joint 2", &ang2, -180.0f, 180.0f, "%.2f");

            ImGui::SetNextItemWidth(100);
            ImGui::SliderFloat("Joint 3", &ang3, -180.0f, 180.0f, "%.2f");

            ImGui::SetNextItemWidth(100);
            ImGui::SliderFloat("Joint 4", &ang4, -180.0f, 180.0f, "%.2f");

            ImGui::SetNextItemWidth(100);
            ImGui::SliderFloat("Joint 5", &ang5, -180.0f, 180.0f, "%.2f");

            ImGui::SetNextItemWidth(100);
            ImGui::SliderFloat("Joint 6", &ang6, -180.0f, 180.0f, "%.2f");

        }
        

        
        

        // *****************************************************
        // Check if any angle has been adjusted
        static bool ResFlag = false;
        if ((tem1 != ang1 || tem2 != ang2 || tem3 != ang3 || tem4 != ang4 || tem5 != ang5 || tem6 != ang6) && !ResFlag)
        {
            // Save current angles to temporary variables
            tem1 = ang1; tem2 = ang2; tem3 = ang3; tem4 = ang4; tem5 = ang5; tem6 = ang6;

            // Reset angles and objects to original state
            ang1 = ang2 = ang3 = ang4 = ang5 = ang6 = 0;
            for (int j = 0; j < 2; j++)
            {
                for (int i = 0; i < 6; i++)
                {
                    base[i]->oMaterial.rotation = OrgBase[i]->oMaterial.rotation;
                    base[i]->mVertices = OrgBase[i]->mVertices;
                    base[i]->oMaterial.position = OrgBase[i]->oMaterial.position;
                    base[i]->oMaterial.mOxyz = OrgBase[i]->oMaterial.mOxyz;
                }

                // Rotate joints to reset all statuses
                rotateJoint(5, ang6, pre6, tolerance, Joints, base, ang6 - pre6, 0, 0);
                rotateJoint(4, ang5, pre5, tolerance, Joints, base, 0, ang5 - pre5, 0);
                rotateJoint(3, ang4, pre4, tolerance, Joints, base, ang4 - pre4, 0, 0);
                rotateJoint(2, ang3, pre3, tolerance, Joints, base, 0, ang3 - pre3, 0);
                rotateJoint(1, ang2, pre2, tolerance, Joints, base, 0, ang2 - pre2, 0);
                rotateJoint(0, ang1, pre1, tolerance, Joints, base, 0, 0, ang1 - pre1);
            }
            ResFlag = true;
        }
        // If angles have changed, update joint rotations
        if (tem1 != ang1 || tem2 != ang2 || tem3 != ang3 || tem4 != ang4 || tem5 != ang5 || tem6 != ang6)
        {
            Joints = {
                std::make_shared<nelems::oMesh>(*base[0]),
                std::make_shared<nelems::oMesh>(*base[1]),
                std::make_shared<nelems::oMesh>(*base[2]),
                std::make_shared<nelems::oMesh>(*base[3]),
                std::make_shared<nelems::oMesh>(*base[4]),
                std::make_shared<nelems::oMesh>(*base[5])
            };

            // Restore angles from temporary variables
            ang1 = tem1; ang2 = tem2; ang3 = tem3; ang4 = tem4; ang5 = tem5; ang6 = tem6;
            mRobot->get_angle(ang1, ang2, ang3, ang4, ang5, ang6);

            // Calculate new joint rotations based on angles
            rotateJoint(5, ang6, pre6, tolerance, Joints, base, ang6 - pre6, 0, 0);
            rotateJoint(4, ang5, pre5, tolerance, Joints, base, 0, ang5 - pre5, 0);
            rotateJoint(3, ang4, pre4, tolerance, Joints, base, ang4 - pre4, 0, 0);
            rotateJoint(2, ang3, pre3, tolerance, Joints, base, 0, ang3 - pre3, 0);
            rotateJoint(1, ang2, pre2, tolerance, Joints, base, 0, ang2 - pre2, 0);
            rotateJoint(0, ang1, pre1, tolerance, Joints, base, 0, 0, ang1 - pre1);

            ResFlag = false;
        }

        // Create buffers for each base
        for (auto& bs : base) { bs->create_buffers(); }
        ImGui::End();
    }


    void nui::Property_Panel::rotateJoint(size_t jointIndex, float& ang, float& pre, const float tolerance,
        std::vector< std::shared_ptr <nelems::oMesh>>& joints, std::vector<std::shared_ptr <nelems::oMesh>>& base,
        float diffX, float diffY, float diffZ)
    {
        ang = std::round(ang * 100.0f) / 100.0f;
        if (std::abs(ang - pre) > tolerance) {
            float diff = ang - pre;
            if (jointIndex == 0)
            {
                std::cout << 0 << std::endl;
            }
            for (size_t i = jointIndex; i < joints.size(); ++i) {
                joints[i]->applyTransformation(base[jointIndex]->oMaterial.position, diffX, diffY, diffZ);
                base[i]->mVertices = joints[i]->mVertices;
                base[i]->oMaterial.position = joints[i]->oMaterial.position;
                base[i]->oMaterial.mOxyz = joints[i]->oMaterial.mOxyz;
                // base[i]->create_buffers();
            }
            base[jointIndex]->oMaterial.rotation = joints[jointIndex]->oMaterial.rotation;

            pre = std::round(ang * 100.0f) / 100.0f;
        }     
    }
}