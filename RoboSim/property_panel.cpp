#include "pch.h"
#include "property_panel.h"
#include <filesystem> 
#include <ctime>
#include "ui/HotkeyMenubar.h"

#include "rigging/rigging.h"
#include "rigging/point.h"
#include <cmath>


namespace nui
{
    // long long nui::Property_Panel::selectedID = 0;
    void Property_Panel::render(nui::SceneView* scene_view, GLFWwindow* mWindow)
    {
        if (!proMesh) {
            proMesh = &nelems::mMesh::getInstance();
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
            if (!mRobocn) { mRobocn = std::make_unique<robocn>(); }
            if (mRobocn)
            {
                if (ImGui::Button("connect")) { mRobocn->connect(); }
                if (ImGui::Button("Get Pos")) { mRobocn->get_pos(); }
                if (ImGui::Button("Get Tor")) { mRobocn->get_tor(); }
                if (ImGui::Button("Set Move")) { mRobocn->set_mov(); }
            }
            obInfo_frame(); // show object info such as vertices and vertex indices
            ImGui::Separator();
            coordinate_frame(); // show the position
            material_frame(scene_view); // show material properties
            ImGui::End();

        }

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

        if (proMesh->size() > 0)
        {
            ImGui::SetNextWindowSize(ImVec2(ImGui::GetIO().DisplaySize.x * 0.15f, ImGui::GetIO().DisplaySize.y * 0.15f));
            ImGui::BeginTable("Objects", 3, ImGuiTableFlags_Resizable | ImGuiTableFlags_ScrollX);
            ImGui::TableSetupColumn("Sel");
            ImGui::TableSetupColumn("Name");
            ImGui::TableSetupColumn("Hide");
            ImGui::TableHeadersRow();

            for (int i = 0; i < proMesh->size(); i++)
            {
                nelems::mMesh::getInstance().get_mesh_ptr(i, mesh);
                ImGui::TableNextRow();
                ImGui::TableNextColumn();

                bool isSelected = mesh->selected;
                bool justSelected = false;
                static bool SeReselect = false;
                if (ImGui::Checkbox(("##Select" + std::to_string(i)).c_str(), &isSelected))
                {
                    if (ImGui::GetIO().KeyShift && lastSelectedIndex != -1)
                    {
                        // Shift is held down, select range
                        int start = (lastSelectedIndex < i) ? lastSelectedIndex : i;
                        int end = (lastSelectedIndex > i) ? lastSelectedIndex : i;
                        if (!SeReselect)
                        {
                            for (int j = start; j <= end; j++)
                            {
                                nelems::mMesh::getInstance().get_mesh_ptr(j, mesh);
                                mesh->selected = true;
                                selectedMeshes.insert(mesh->ID);
                            }
                            SeReselect = true;
                        }
                        else {
                            for (int j = start; j <= end; j++)
                            {
                                nelems::mMesh::getInstance().get_mesh_ptr(j, mesh);
                                mesh->selected = false;
                                selectedMeshes.insert(mesh->ID);
                            }
                            SeReselect = false;
                        }
                    }
                    else
                    {
                        // Normal single selection
                        mesh->selected = !mesh->selected;
                        if (mesh->selected)
                        {
                            selectedMeshes.insert(mesh->ID);
                            lastSelectedIndex = i;
                        }
                        else
                        {
                            selectedMeshes.erase(mesh->ID);
                        }
                        justSelected = true;
                    }
                }

                ImGui::TableNextColumn();
                ImGui::Text(mesh->oname);
                ImGui::TableNextColumn();
                ImGui::Checkbox(("##Hide" + std::to_string(i)).c_str(), &mesh->hide);

                if (mesh->selected || justSelected)
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
        if (!pos_x || !pos_y) { nui::FrameManage::getViewportSize(pos_x, pos_y); }
        ImGui::SetNextWindowPos(ImVec2(pos_x + 5, pos_y + 35)); // Set the position of the frame
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
                proMesh->get_mesh_ptr(i, mesh);
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
                proMesh->get_mesh_ptr(i, mesh);
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
                    proMesh->get_mesh_ptr(i, mesh);
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
                        proMesh->get_mesh_ptr(i, mesh);
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
                            proMesh->get_mesh_ptr(j, mesh);
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
                proMesh->get_mesh_ptr(i, mesh);
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
                proMesh->get_mesh_ptr(i, mesh);
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
        // get base objects
        for (int i{ 0 }; i < proMesh->size(); i++)
        {
            // Error when taking base1, base2, base3, base4, base5, base6
            proMesh->get_mesh_ptr(i, mesh);
            std::string name = std::string(mesh->oname);
            if (name.find("base 1") != std::string::npos) 
            { proMesh->get_mesh_ptr(i, base1); }
            else if (name.find("base 2") != std::string::npos) 
            { proMesh->get_mesh_ptr(i, base2); }
			else if (name.find("base 3") != std::string::npos) 
            { proMesh->get_mesh_ptr(i, base3); }
			else if (name.find("base 4") != std::string::npos) 
            { proMesh->get_mesh_ptr(i, base4); }
			else if (name.find("base 5") != std::string::npos) 
            { proMesh->get_mesh_ptr(i, base5); }
			else if (name.find("base 6") != std::string::npos) 
            { proMesh->get_mesh_ptr(i, base6); }
        }
        static float tolerance = 0.1f;
        if (!base1) { return; }
        ImGui::Begin("Robot Controls", nullptr);
        // base 1 - z 

        static float base1_rot_prev = base1->oMaterial.rotation.z;
        ImGui::SliderFloat("Base1 Deg", &base1_rot_prev, -180.0f, 180.0f, "%.1f");
        // Check the absolute value of the difference
        if (std::abs(base1_rot_prev - base1->oMaterial.rotation.z) > tolerance)
        {
            float diff = base1_rot_prev - base1->oMaterial.rotation.z;
            base1->rotate(0, 0, diff);
            base2->rotate(0, 0, diff, base1->oMaterial.position);
            base3->rotate(0, 0, diff, base1->oMaterial.position);
            base4->rotate(0, 0, diff, base1->oMaterial.position);
            base5->rotate(0, 0, diff, base1->oMaterial.position);
            base6->rotate(0, 0, diff, base1->oMaterial.position);
            base1->create_buffers(); base2->create_buffers(); base3->create_buffers();
            base4->create_buffers(); base5->create_buffers(); base6->create_buffers();
            base1_rot_prev = base1->oMaterial.rotation.z;
        }

        // base 2 - y
        static float base2_rot_prev = base2->oMaterial.rotation.y;
        ImGui::SliderFloat("Base2 Deg", &base2_rot_prev, -180.0f, 180.0f, "%.1f");
        // Check the absolute value of the difference
        if (std::abs(base2_rot_prev - base2->oMaterial.rotation.y) > tolerance)
        {
			float diff = base2_rot_prev - base2->oMaterial.rotation.y;
			base2->rotate(0, diff, 0);
			base3->rotate(0, diff, 0, base2->oMaterial.position);
			base4->rotate(0, diff, 0, base2->oMaterial.position);
			base5->rotate(0, diff, 0, base2->oMaterial.position);
			base6->rotate(0, diff, 0, base2->oMaterial.position);
			base2->create_buffers(); base3->create_buffers(); base4->create_buffers();
			base5->create_buffers(); base6->create_buffers();
			base2_rot_prev = base2->oMaterial.rotation.y;
		}
        // base 3 - y
        static float base3_rot_prev = base3->oMaterial.rotation.y;
        ImGui::SliderFloat("Base3 Deg", &base3_rot_prev, -180.0f, 180.0f, "%.1f");
        // Check the absolute value of the difference
        if (std::abs(base3_rot_prev - base3->oMaterial.rotation.y) > tolerance)
        {
            float diff = base3_rot_prev - base3->oMaterial.rotation.y;
            base3->rotate(0, diff, 0);
            base4->rotate(0, diff, 0, base3->oMaterial.position);
            base5->rotate(0, diff, 0, base3->oMaterial.position);
            base6->rotate(0, diff, 0, base3->oMaterial.position);
            base3->create_buffers(); base4->create_buffers(); base5->create_buffers(); base6->create_buffers();
            base3_rot_prev = base3->oMaterial.rotation.y;
        }
        // base 4 - x
        static float base4_rot_prev = base4->oMaterial.rotation.x;
        ImGui::SliderFloat("Base4 Deg", &base4_rot_prev, -180.0f, 180.0f, "%.1f");
        // Check the absolute value of the difference
        if (std::abs(base4_rot_prev - base4->oMaterial.rotation.x) > tolerance)
        {
			float diff = base4_rot_prev - base4->oMaterial.rotation.x;
			base4->rotate(diff, 0, 0);
			base5->rotate(diff, 0, 0, base4->oMaterial.position);
			base6->rotate(diff, 0, 0, base4->oMaterial.position);
			base4->create_buffers(); base5->create_buffers(); base6->create_buffers();
			base4_rot_prev = base4->oMaterial.rotation.x;
		}
        // base 5 - y
        static float base5_rot_prev = base5->oMaterial.rotation.y;
        ImGui::SliderFloat("Base5 Deg", &base5_rot_prev, -180.0f, 180.0f, "%.1f");
        // Check the absolute value of the difference
        if (std::abs(base5_rot_prev - base5->oMaterial.rotation.y) > tolerance)
        {
            float diff = base5_rot_prev - base5->oMaterial.rotation.y;
            base5->rotate(0, diff, 0);
            base6->rotate(0, diff, 0, base5->oMaterial.position);
            base5->create_buffers(); base6->create_buffers();
            base5_rot_prev = base5->oMaterial.rotation.y;
        }
        // base 6 - x
        static float base6_rot_prev = base6->oMaterial.rotation.x;
        ImGui::SliderFloat("Base6 Deg", &base6_rot_prev, -180.0f, 180.0f, "%.1f");
        // Check the absolute value of the difference
        if (std::abs(base6_rot_prev - base6->oMaterial.rotation.x) > tolerance)
        {
			float diff = base6_rot_prev - base6->oMaterial.rotation.x;
			base6->rotate(diff, 0, 0);
			base6->create_buffers();
			base6_rot_prev = base6->oMaterial.rotation.x;
		}


        ImGui::End();
    }
}