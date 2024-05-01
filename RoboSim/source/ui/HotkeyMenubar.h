#pragma once
#include "pch.h"
#include "ui/scene_view.h"
#include "ui/uiAction.h"
#include "nlohmann/json.hpp"


using json = nlohmann::json;
namespace nui {
    // Manage the hotkey and menubar
    // all actions will move to uiAction
	class HotkeyMenubar {
    private:
        static uiAction uiaction;
        nui::SceneView* scene_view;
        std::string mCurrentFile;
        

    public:
        HotkeyMenubar() : scene_view(nullptr) {
            scene_view = &nui::SceneView::getInstance();
            
        }
        ////====    ==============MENU BAR==================
        void commandLogs(){ uiaction.Command_Logs(); }
        void mMenuBar(GLFWwindow* mWindow)
        {

            if (ImGui::BeginMainMenuBar())
            {
                if (ImGui::BeginMenu("File"))
                {
                    if (ImGui::MenuItem("New"))
                    {
                        //NewScene();
                    }
                    if (ImGui::MenuItem("Load", "Ctrl+L"))
                    {
                        uiaction.LoadFromFile();
                    }
                    if (ImGui::MenuItem("Import", "Ctrl+I"))
                    {
                        OpenFileDialog();
                    }
                    if (ImGui::MenuItem("Save", "Ctrl+S"))
                    {

                        uiaction.SaveToFile();
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
                if (ImGui::BeginMenu("Edit"))
                {
                    if (ImGui::MenuItem("Undo", "Ctrl+Z")) {
                        uiaction.undocmd();
                    }
                    if (ImGui::MenuItem("Redo", "Ctrl+Y")) {
                        uiaction.redocmd();
                    }
                    //// LOI FIX BUG!!!!!!!!!!!!!!!
                    if (ImGui::MenuItem("Move", "Ctrl+M") || glfwGetKey(mWindow, GLFW_KEY_M) == GLFW_PRESS) {
                        //uiaction.MoveOb_uiAction(NULL);
                    }
                    if (ImGui::MenuItem("Rotate", "Ctrl+R")) {

                        uiaction.RotateOb_uiAction();
                    }
                    ImGui::EndMenu();
                }
                if (ImGui::BeginMenu("View"))
                {
                    if (ImGui::BeginMenu("Theme"))
                    {
                        if (ImGui::MenuItem("Dark"))
                        {
                            SaveIniFile("theme", "dark");
                        }
                        if (ImGui::MenuItem("Light"))
                        {
                            SaveIniFile("theme", "light");
                        }
                        ImGui::EndMenu();
                    }
                    if (ImGui::BeginMenu("RenderMode"))
                    {
                        const char* menuItems[] = { "Points", "WireFrame", "Surface"
                                            ,"Points-Wire", "Points-Face", "Wire-Face", "Point-Wire-Face" };
                        for (int i = 0; i < sizeof(menuItems) / sizeof(menuItems[0]); ++i) {
                            if (ImGui::MenuItem(menuItems[i])) {
                                scene_view->set_render_mode(menuItems[i]);
                            }
                        }
                        ImGui::EndMenu();
                    }
                    if (ImGui::BeginMenu("Font"))
                    {
                        static std::vector<std::string> menuFont;
                        if (menuFont.empty()) {
                            const std::filesystem::path fontFolderPath = "C:/Windows/Fonts";

                            for (const auto& entry : std::filesystem::directory_iterator(fontFolderPath)) {
                                if (entry.path().extension() == ".ttf") {
                                    menuFont.push_back(entry.path().filename().stem().string());
                                }
                            }
                        }
                        for (int i = 0; i < menuFont.size(); ++i) {
                            if (ImGui::MenuItem(menuFont[i].c_str())) {
                                if (std::filesystem::exists("C:/Windows/Fonts/" + std::string(menuFont[i]) + ".ttf")) {
                                    SaveIniFile("font", menuFont[i]);
                                }
                                else {
                                    MessageBox(NULL, "Font not found in C:/Windows/Fonts/", "Font not found", MB_OK);
                                }
                            }
                        }
                        ImGui::EndMenu();
                    }
                    ImGui::EndMenu();
                }
                if (ImGui::BeginMenu("Object"))
                {
                    //TODO: UPDATED THIS FEATURE FOR CAMERA ROTATION
                    if (ImGui::MenuItem("New Coor center point")) {

                    }

                    ImGui::EndMenu();
                }
                ImGui::EndMainMenuBar();

            }
        }
        void mHotkey(GLFWwindow* mWindow)
        {
            static bool waitloop[1]{ false }; // Use for mini popup recall the ImGui in main Loops
            static double lastPressTime = 0.0;
            static const double debounceDelay = 0.1;


            std::time_t currentTime = std::time(nullptr);
            //normal keyboard
            // m = MoveOb
            bool lCtr = glfwGetKey(mWindow, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS;
            bool rCtr = glfwGetKey(mWindow, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS;
            bool tdelay = currentTime - lastPressTime > debounceDelay;

            if (((glfwGetKey(mWindow, GLFW_KEY_M) == GLFW_PRESS && (lCtr || rCtr) && tdelay)) || waitloop[0])
            {
                lastPressTime = currentTime; waitloop[0] = true;
                uiaction.MoveOb_uiAction(waitloop[0]);
            }

            if (glfwGetKey(mWindow, GLFW_KEY_R) == GLFW_PRESS && (lCtr || rCtr) && tdelay)
            { lastPressTime = currentTime; uiaction.RotateOb_uiAction(); }


            if (glfwGetKey(mWindow, GLFW_KEY_Z) == GLFW_PRESS && (lCtr || rCtr) && tdelay)
            { lastPressTime = currentTime; uiaction.undocmd(); }

            if (glfwGetKey(mWindow, GLFW_KEY_Y) == GLFW_PRESS && (lCtr || rCtr) && tdelay)
            { lastPressTime = currentTime; uiaction.redocmd(); }

            if (glfwGetKey(mWindow, GLFW_KEY_I) == GLFW_PRESS && (lCtr || rCtr) && tdelay)
            { lastPressTime = currentTime;  OpenFileDialog(); }

            if (glfwGetKey(mWindow, GLFW_KEY_S) == GLFW_PRESS && (lCtr || rCtr) && tdelay)
            { lastPressTime = currentTime;  uiaction.SaveToFile(); }

            if (glfwGetKey(mWindow, GLFW_KEY_L) == GLFW_PRESS && (lCtr || rCtr) && tdelay)
            { lastPressTime = currentTime;  uiaction.SaveToFile(); }

            // TODO: move this and camera to scene UI component?
            if (nui::FrameManage::getCrActiveGui("ViewPort") == true)
            {

                if (glfwGetKey(mWindow, GLFW_KEY_W) == GLFW_PRESS)
                {
                    scene_view->on_mouse_wheel(0.4f);
                }

                if (glfwGetKey(mWindow, GLFW_KEY_S) == GLFW_PRESS)
                {
                    scene_view->on_mouse_wheel(-0.4f);
                }

                if (glfwGetKey(mWindow, GLFW_KEY_F) == GLFW_PRESS)
                {
                    scene_view->reset_view();
                }

                double x, y;
                glfwGetCursorPos(mWindow, &x, &y);
                scene_view->on_mouse_move(x, y, nelems::Input::GetPressedButton(mWindow));
            }
        }

        ////====    ==============OPEN FILE DIALOG==========
        void SaveIniFile(const std::string& key, const std::string& value) {
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
        void OpenFileDialog()
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
                mCurrentFile = filePath.substr(filePath.find_last_of("/\\") + 1);
                scene_view->setlink_meshloadcallback(filePath);
                
            }
        }
	};
}