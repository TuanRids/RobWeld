#include "pch.h"
#include "property_panel.h"



namespace nui
{
    // long long nui::Property_Panel::selectedID = 0;
    void Property_Panel::render(nui::SceneView* scene_view)
    {
        if (!proMesh) {
            proMesh = &nelems::mMesh::getInstance();
        }

        //================================================================================================
        /// Main Properties
        MenuBar();

        //****************************************************
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
        // Another Frame
        uiaction.Command_Logs();
        
        // Another Frames
        camera_frame(scene_view); // show camera properties

    }
    
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
                ImGui::SliderFloat("Far", &newfar, 0.1f, 400.0f, "%.1f");
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
            ImGui::BeginTable("Objects", 2 , ImGuiTableFlags_Resizable);
            ImGui::TableSetupColumn("Set");
            ImGui::TableSetupColumn("Name");
            ImGui::TableHeadersRow();

            for (int i = 0; i < proMesh->size(); i++)
            {
                nelems::mMesh::getInstance().get_mesh_ptr(i, mesh);
                ImGui::TableNextRow();

                ImGui::TableNextColumn();
                if (ImGui::Checkbox(("##Select" + std::to_string(i)).c_str(), &mesh->selected)) {
                    if (mesh->selected) {
                        selectedMeshes.insert(mesh->ID);
                        
                    }
                    else {
                        selectedMeshes.erase(mesh->ID);
                    }
                }

                ImGui::TableNextColumn();
                ImGui::Text(mesh->oname);
                if (mesh->selected){
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
            if (proMesh->check_selected()!=1){return;} // 0 select or select more than 1 will return
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
        
        if (ImGui::CollapsingHeader("Coordinates", ImGuiTreeNodeFlags_DefaultOpen) ) //&&mesh
        {
            ImGui::Separator();
            ImGui::Text("ADD HISTORY COMMANDS LOGS");
            ImGui::Text("Add coordinate position, rotation, and scale");
            ImGui::Text("remove default bar, add new menubar ass a title bar");
            ImGui::Separator();

        }
    }

    void Property_Panel::material_frame(nui::SceneView* scene_view) {
        if (ImGui::CollapsingHeader("Material", ImGuiTreeNodeFlags_DefaultOpen))
        {
            if (proMesh->check_selected() != 1) { return; } // 0 select or more than 1 will return. 

            for (int i = 0; i < proMesh->size(); i++)
            {
                proMesh->get_mesh_ptr(i, mesh);
                if (mesh->selected == true)
                {
                    break;
                }
            }
            if (mesh)
            {
                ImGui::ColorEdit3("Color", (float*)&(mesh->oMaterial.mColor));
                ImGui::SliderFloat("Roughness", &mesh->oMaterial.roughness, 0.0f, 1.0f);
                ImGui::SliderFloat("Metallic", &mesh->oMaterial.metallic, 0.0f, 1.0f);
            }
            else {
                static ImVec4 disabledColor = ImVec4(0.5f, 0.5f, 0.5f, 1.0f);// gray out the widgets
                static float disabledFloat = 0.5f; // 0.5f is the default value
                ImGui::ColorEdit3("Color", (float*)&disabledColor);
                ImGui::SliderFloat("Roughness", &disabledFloat, 0.0f, 1.0f);
                ImGui::SliderFloat("Metallic", &disabledFloat, 0.0f, 1.0f);
            }
        }
        /// Light
        if (ImGui::CollapsingHeader("Light", ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::Separator();
            nimgui::draw_vec3_widget("Position", scene_view->get_light()->mPosition, 80.0f);
            ImGui::SliderInt("Light Intensity", &scene_view->get_light()->mStrength, 1, 1000);
        }
        

    }
    
    void Property_Panel::OpenFileDialog()
    {
        OPENFILENAME ofn;
        char szFile[260] = { 0 };

        ZeroMemory(&ofn, sizeof(ofn));
        ofn.lStructSize = sizeof(ofn);
        ofn.hwndOwner = NULL;
        ofn.lpstrFile = szFile;
        ofn.nMaxFile = sizeof(szFile);
        ofn.lpstrFilter = "FBX Files (*.fbx)\0*.fbx\0OBJ Files (*.obj)\0*.obj\0STL Files (*.stl)\0*.stl\0All Files (*.*)\0*.*\0";
        ofn.nFilterIndex = 1;
        ofn.lpstrFileTitle = NULL;
        ofn.nMaxFileTitle = 0;
        ofn.lpstrInitialDir = mCurrentFile.c_str();
        ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;

        if (GetOpenFileName(&ofn) == TRUE)
        {
            std::string filePath = ofn.lpstrFile;
            std::cout << filePath << std::endl;
            mCurrentFile = filePath.substr(filePath.find_last_of("/\\") + 1);

            mMeshLoadCallback(filePath);
        }
    }

    void Property_Panel::MenuBar()
    {
        if (ImGui::BeginMainMenuBar())
        {
            if (ImGui::BeginMenu("File"))
            {
                if (ImGui::MenuItem("New"))
                {
                    //NewScene();
                }
                if (ImGui::MenuItem("Load","Ctrl+L"))
                {

                    nelems::RobsFileIO::LoadFromFile(*proMesh);
                }
                if (ImGui::MenuItem("Import", "Ctrl+I"))
                {
                    OpenFileDialog();
                }
                if (ImGui::MenuItem("Save", "Ctrl+S"))
                {

                    if (proMesh->size() == 0) {
                        std::cout << "Error: Mesh not loaded" << std::endl;
                    }
                    else
                    {
                        nelems::RobsFileIO::SaveToFile(*proMesh);
                    }
                }
                if (ImGui::MenuItem("Save As", "Ctrl+Shift+S"))
                {
                    //SaveSceneAs();
                }
                if (ImGui::MenuItem("Exit"))
                {
                    //Exit();
                }
                ImGui::EndMenu();
            }
            if (ImGui::BeginMenu("Edit")) {
                if (ImGui::MenuItem("Undo", "Ctrl+Z")) {
                    uiaction.undocmd();
                }
                if (ImGui::MenuItem("Redo", "Ctrl+Y")) {
                    uiaction.redocmd();
                }
                //// LOI FIX BUG!!!!!!!!!!!!!!!
                if (ImGui::MenuItem("Move", "m")) {

                    uiaction.MoveOb_uiAction();
                }
                ImGui::EndMenu();
            }
            ImGui::EndMainMenuBar();

        }

    }
}