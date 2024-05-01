#include "pch.h"
#include "property_panel.h"
#include <filesystem> 
#include <ctime>
#include "ui/HotkeyMenubar.h"

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
            ImGui::CollapsingHeader("Properties", ImGuiTreeNodeFlags_DefaultOpen); // settings
            nui::FrameManage::setCrActiveGui("Properties", ImGui::IsWindowFocused() || ImGui::IsWindowHovered()); // setting

            obInfo_frame(); // show object info such as vertices and vertex indices

            ImGui::Separator();
            coordinate_frame();
            material_frame(scene_view); // show material properties
            ImGui::End();
        }
        // Another Frames
        camera_frame(scene_view); // show camera properties
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
            static float newfov{ 45.0f }, newnear{ 1.0f }, newfar{ 400.0f };
            static int newzoom(5);
            ImGui::SliderFloat("Fov", &newfov, 1.0f, 99.0f, "%.0f");
            ImGui::SliderFloat("Near", &newnear, 0.01f, 10.0f, "%.1f");
            ImGui::SliderFloat("Far", &newfar, 0.1f, 400.0f, "%.0f");
            ImGui::SliderInt("ZSp", &newzoom, 0, 20);
            SceneView::getInstance().setFov(newfov);
            SceneView::getInstance().setNear(newnear);
            SceneView::getInstance().setFar(newfar);
            SceneView::getInstance().setZoom(newzoom);
            ImGui::End();
        }
    }
    void Property_Panel::layer_frame(nui::SceneView* scene_view)
    {
        ImGui::Begin("Layer", nullptr);
        nui::FrameManage::setCrActiveGui("Layer", ImGui::IsWindowFocused() || ImGui::IsWindowHovered());
        if (proMesh->size() > 0)
        {
            ImGui::SetNextWindowSize(ImVec2(ImGui::GetIO().DisplaySize.x * 0.15f, ImGui::GetIO().DisplaySize.y * 0.15f));
            //ImGui::PushItemFlag(ImGuiItemFlags_Disabled, false);
            ImGui::BeginTable("Objects", 3, ImGuiTableFlags_Resizable | ImGuiTableFlags_ScrollX);
            ImGui::TableSetupColumn("Set");
            ImGui::TableSetupColumn("Name");
            ImGui::TableSetupColumn("Hide");
            ImGui::TableHeadersRow();

            for (int i = 0; i < proMesh->size(); i++)
            {
                nelems::mMesh::getInstance().get_mesh_ptr(i, mesh);
                ImGui::TableNextRow();

                ImGui::TableNextColumn();
                bool isSelected = ImGui::Checkbox(("##Select" + std::to_string(i)).c_str(), &mesh->selected);
                // TODO UPDATE LATER FOR THE CTRL SELECT AND SINGLE SELECT
                if (isSelected) {
                    if (mesh->selected) { selectedMeshes.insert(mesh->ID); }
                    else { selectedMeshes.erase(mesh->ID); }
                }
                ImGui::TableNextColumn();
                ImGui::Text(mesh->oname);
                ImGui::TableNextColumn();
                ImGui::Checkbox(("##Hide" + std::to_string(i)).c_str(), &mesh->hide);

                if (mesh->selected) {
                    ImU32 yellowColorU32 = ImGui::ColorConvertFloat4ToU32(ImVec4(0.25f, 0.42f, 1.0f, 1.0f));
                    ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, yellowColorU32);
                }
            }
            ImGui::EndTable();
        }
        ImGui::End();
    }
    void Property_Panel::obInfo_frame()
    {
        if (proMesh)
        {
            if (proMesh->check_selected() != 1) { return; } // 0 select or select more than 1 will return
            for (int i = 0; i < proMesh->size(); i++)
            {
                proMesh->get_mesh_ptr(i, mesh);
                if (mesh->selected == true)
                {
                    break;
                }
            }
            ImGui::BeginTable("MeshTable", 2, ImGuiTableFlags_Borders);


            ImGui::TableNextRow();

            ImGui::TableNextColumn();
            ImGui::Text("Name");
            ImGui::TableNextColumn();
            ImGui::InputText("##Name", mesh->oname, ImGuiInputTextFlags_EnterReturnsTrue);

            ImGui::TableNextColumn();
            ImGui::Text("ID");
            ImGui::TableNextColumn();
            ImGui::Text("%lld", mesh->ID);

            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("Vertices");
            ImGui::TableNextColumn();
            ImGui::Text("%lld", mesh->mVertices.size());

            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("VertexIndices");
            ImGui::TableNextColumn();
            ImGui::Text("%lld", mesh->mVertexIndices.size());
            ImGui::EndTable();

        }
    }
    void Property_Panel::coordinate_frame()
    {

        if (ImGui::CollapsingHeader("Coordinates", ImGuiTreeNodeFlags_DefaultOpen)) //&&mesh
        {
            ImGui::Separator();
            ImGui::Text("Add 3D grid viewport");
            ImGui::Text("Add front, right and top view");
            ImGui::Separator();
            static float posrot[6];
            
            static char aname[10] = "";
            ImGui::Text("Name"); ImGui::SameLine();
            ImGui::InputText("##Name", aname, ImGuiInputTextFlags_EnterReturnsTrue);

            ImGui::Text("x_Pos"); ImGui::SameLine();
            ImGui::InputFloat("##xPos", &posrot[0], 0.0f, 0.0f, "%.3f");

            ImGui::Text("y_Pos"); ImGui::SameLine();
            ImGui::InputFloat("##yPos", &posrot[1], 0.0f, 0.0f, "%.3f");

            ImGui::Text("z_Pos"); ImGui::SameLine();
            ImGui::InputFloat("##zPos", &posrot[2], 0.0f, 0.0f, "%.3f");

            ImGui::Text("x_Rot"); ImGui::SameLine();
            ImGui::InputFloat("##xRot", &posrot[3], 0.0f, 0.0f, "%.3f");

            ImGui::Text("y_Rot"); ImGui::SameLine();
            ImGui::InputFloat("##yRot", &posrot[4], 0.0f, 0.0f, "%.3f");

            ImGui::Text("z_Rot"); ImGui::SameLine();
            ImGui::InputFloat("##zRot", &posrot[5], 0.0f, 0.0f, "%.3f");
            if (proMesh->check_selected() != 0)
            {
                // check if any elements of posrot is not 0
                bool hasNonZeroElement = std::any_of(std::begin(posrot), std::end(posrot), 
                                        [](float value) { return value != 0.0f; });
                if ((hasNonZeroElement) && ImGui::Button("OK"))
                {
                    uiaction.MoveOb_uiAction(posrot[0], posrot[1], posrot[2]);
                    uiaction.RotateOb_uiAction(posrot[3], posrot[4], posrot[5]);
                    // reset all posrot & aname;
                    std::fill_n(posrot, 6, 0.0f);
                    std::fill_n(aname, sizeof(aname), '\0');
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
            ImGui::ColorEdit3("Color", (float*)&clor);
            ImGui::SliderFloat("Roughness", &rness, 0.0f, 1.0f);
            ImGui::SliderFloat("Metallic", &mlic, 0.0f, 1.0f);

            for (int i = 0; i < proMesh->size(); i++)
            {
                proMesh->get_mesh_ptr(i, mesh);
                if (mesh->selected == true)
                {
                    if ((preClor.x != clor.x ||  preClor.y != clor.y || preClor.z != clor.z || preClor.w != clor.w)) {
                        std::cout << mesh->oname << std::endl;
                        mesh->oMaterial.mColor = glm::vec3(clor.x, clor.y, clor.z);
                    }
                    if (prerness != rness) {
                        mesh->oMaterial.roughness = rness;
                    }
                    if (premlic != mlic) {
                        mesh->oMaterial.metallic = mlic;
                    }
                }
            }
        }
        /// Light
        if (ImGui::CollapsingHeader("Light", ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::Separator();
            nimgui::draw_vec3_widget("Position", scene_view->get_light()->mPosition, 80.0f);
            ImGui::SliderInt("Light Intensity", &scene_view->get_light()->mStrength, 0, 1000);
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
    void Property_Panel::SaveIniFile(const std::string& key, const std::string& value) {
        // save theme to file
        json j;
        j[key] = value;
        std::ofstream file("robosim_ini.dat");
        if (!file.is_open()) {
            std::cerr << "Failed to open file for writing: robosim_ini.dat" << std::endl;
            return;
        }
        file << j.dump(4);
        std::cout << "Theme saved to robosim_ini.dat" << std::endl;
        // show restart required message
        MessageBox(NULL, "Please restart the software to apply the new theme.", "Restart required", MB_OK);
    }

    ////=============================================================================================
    //// MENUBAR AND HOTKEYS
    
}