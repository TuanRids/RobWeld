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
            static float newnear{ 1.0f }, newfar{ 0.0f };
            static int newzoom(10); static int gridNum(50); static int gridStep(1);

            ImGui::SliderFloat("Near", &newnear, 0.01f, 10.0f, "%.1f");
            ImGui::SliderFloat("Far", &newfar, 0.0f, 2000.0f, "%.0f");
            ImGui::SliderInt("ZSp", &newzoom, 0, 20);
            ImGui::SliderInt("GridNum", &gridNum, 0, 100);
            ImGui::SliderInt("GridStep", &gridStep, 0, 10);

            SceneView::getInstance().setNear(newnear);
            SceneView::getInstance().setFar(newfar);
            SceneView::getInstance().setZoom(newzoom);
            ImGui::End();
            // grid
            proMesh->createGridSys(gridNum, gridStep);

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
        ImGui::Separator();
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
            ImGui::Text("Add arrow");
            ImGui::Text("Add front, right and top view");
            ImGui::Text("Add Delete = delete objects");
            ImGui::Separator();
            bool pressOk = ImGui::Button("UPDATE");
            if (proMesh->check_selected() == 1)
            {
                static float posrot_obj[7]; static long long PreObId = 0;
                for (int i = 0; i < proMesh->size(); i++)
                {
                    proMesh->get_mesh_ptr(i, mesh);
                    if (mesh->selected && PreObId != mesh->ID)
                    {
                        PreObId = mesh->ID;
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
                ImGui::InputText("##Name", mesh->oname, ImGuiInputTextFlags_EnterReturnsTrue);

                ImGui::Text("x_Pos"); ImGui::SameLine();
                ImGui::InputFloat("##xPos", &posrot_obj[0], 0.0f, 0.0f, "%.3f");
                
                ImGui::Text("y_Pos"); ImGui::SameLine();
                ImGui::InputFloat("##yPos", &posrot_obj[1], 0.0f, 0.0f, "%.3f");

                ImGui::Text("z_Pos"); ImGui::SameLine();
                ImGui::InputFloat("##zPos", &posrot_obj[2], 0.0f, 0.0f, "%.3f");

                ImGui::Text("x_Rot"); ImGui::SameLine();
                ImGui::InputFloat("##xRot", &posrot_obj[3], 0.0f, 0.0f, "%.3f");

                ImGui::Text("y_Rot"); ImGui::SameLine();
                ImGui::InputFloat("##yRot", &posrot_obj[4], 0.0f, 0.0f, "%.3f");

                ImGui::Text("z_Rot"); ImGui::SameLine();
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
                    if (std::abs(movex) > 0.01 || std::abs(movey) > 0.01 || std::abs(movez) > 0.01 )
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
                    if (hasNonZeroElement && pressOk )
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
        static float rness = 0.5f; static float mlic = 0.5f; static float mtrans = 0.0f;
        if (ImGui::CollapsingHeader("Material", ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImVec4 preClor = clor; float prerness = rness; float premlic = mlic;
            ImGui::ColorEdit3("Color", (float*)&clor);
            ImGui::SliderFloat("Roughness", &rness, 0.0f, 1.0f);
            ImGui::SliderFloat("Metallic", &mlic, 0.0f, 1.0f);
            ImGui::SliderFloat("Transparency", &mtrans, 0.0f, 1.0f);
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
                    if (mtrans != mtrans) {
                        mesh->oMaterial.mTransparency = mtrans;
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

}