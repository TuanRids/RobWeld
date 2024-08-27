#include "pch.h"
#include "HotkeyMenubar.h"
#include "ui/RdrawChart.h"
#include <windows.h>
#include <commdlg.h>
#include <shlwapi.h> // Required for PathRemoveFileSpec
#pragma comment(lib, "shlwapi.lib")

namespace nui {
	bool HotkeyMenubar::waitloop[6] = { false,false,false,false,false,false };
	bool HotkeyMenubar::shint = true;
    static std::unique_ptr<RdrawChart> drchart = std::make_unique<RdrawChart>();

	void HotkeyMenubar::mMenuBar(GLFWwindow* mWindow)
    {
        std::lock_guard<std::mutex> lock(mtx);
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
                if (ImGui::MenuItem("Settings"))
                {
                    OptionSetting_Flag = true;
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
                if (ImGui::MenuItem("Move", "Ctrl+M") || glfwGetKey(mWindow, GLFW_KEY_M) == GLFW_PRESS) {
                    uiaction.MoveOb_uiAction(waitloop[0]);
                }
                if (ImGui::MenuItem("Rotate", "Ctrl+R")) {

                    uiaction.RotateOb_uiAction(waitloop[1]);
                }
                if (ImGui::MenuItem("RandomColor", "Ctrl+R")) {

                    uiaction.Random_Color();
                }
                ImGui::EndMenu();
            }
            if (ImGui::BeginMenu("View"))
            {
                if (ImGui::BeginMenu("RenderMode"))
                {
                    const char* menuItems[7] = { "Points", "WireFrame", "Surface" };
                    for (int i = 0; i < 3; ++i) {
                        if (ImGui::MenuItem(menuItems[i])) {
                            scene_view->set_render_mode(menuItems[i]);
                        }
                    }
                    ImGui::EndMenu();
                }
                ImGui::EndMenu();
            }

            if (ImGui::BeginMenu("Robot"))
            {
                if (ImGui::MenuItem("fromPY"))
                {
                    //
                }
                ImGui::EndMenu();
            }


            if (ImGui::BeginMenu("Chart"))
            {
                if (ImGui::MenuItem("Show"))
                {
                    drchart->setChartFlag(true);
                }
                if (ImGui::MenuItem("Hide"))
				{
					drchart->setChartFlag(false);
					
				}
				ImGui::EndMenu();
            }
            ImGui::EndMainMenuBar();
        }
    }

    void HotkeyMenubar::mHotkey(GLFWwindow* mWindow)

    {
        if (OptionSetting_Flag) { OptionSettings(); }

        drchart->render();
        static double lastPressTime{ 0 };
        static const double debounceDelay = 0.1;


        double currentTime = static_cast<double>(std::time(nullptr));
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
        if (glfwGetKey(mWindow, GLFW_KEY_DELETE) == GLFW_PRESS)
        { lastPressTime = currentTime;  uiaction.Del_selected_objects(); }
        // TODO: move this and camera to scene UI component?
        if (scene_view->getCrActiveGui("ViewPort") == true)
        {
            float panspeed = 0.04f;
            if (glfwGetKey(mWindow, GLFW_KEY_W) == GLFW_PRESS)
            {
                scene_view->on_mouse_wheel(panspeed * 3);
            }
            if (glfwGetKey(mWindow, GLFW_KEY_S) == GLFW_PRESS)
            {
                scene_view->on_mouse_wheel(-1 * panspeed * 3);
            }
            if (glfwGetKey(mWindow, GLFW_KEY_A) == GLFW_PRESS)
            {
                scene_view->on_mouse_move(-1 * panspeed, 0, nelems::EInputButton::key_A);
            }
            if (glfwGetKey(mWindow, GLFW_KEY_D) == GLFW_PRESS)
            {
                scene_view->on_mouse_move(panspeed, 0, nelems::EInputButton::key_D);
            }
            if (glfwGetKey(mWindow, GLFW_KEY_Q) == GLFW_PRESS)
            {
                scene_view->on_mouse_move(0, panspeed, nelems::EInputButton::key_Q);
            }
            if (glfwGetKey(mWindow, GLFW_KEY_E) == GLFW_PRESS)
            {
                scene_view->on_mouse_move(0, -1 * panspeed, nelems::EInputButton::key_E);
            }
            if (glfwGetKey(mWindow, GLFW_KEY_R) == GLFW_PRESS)
            {
                scene_view->set_rotation_center();
            }
            if (glfwGetKey(mWindow, GLFW_KEY_F) == GLFW_PRESS)
            {
                scene_view->reset_view();
            }

            double x, y;
            glfwGetCursorPos(mWindow, &x, &y);
            scene_view->on_mouse_move(x, y, nelems::Input::GetPressedButton(mWindow));
            if (nelems::Input::GetPressedButton(mWindow) == nelems::EInputButton::Right ||
                nelems::Input::GetPressedButton(mWindow) == nelems::EInputButton::Left ||
                nelems::Input::GetPressedButton(mWindow) == nelems::EInputButton::Middle)
            {
                // hint(shint);
            }
        }
    }
    
    void HotkeyMenubar::OptionSettings()
    {
        static const char* theme_items[] = { "Dark", "Light","DarkGreen"};
        static char robot_tcp[64] = "192.168.10.102";
        static const char* creat_item[] = { "Fast", "Medium", "Slow" };
        static char pythonPath[256] = "C:\\Users\\FSAM\\AppData\\Local\\Programs\\Python\\Python312\\python.exe";
        static char scriptPath[256] = "E:\\Quan\\AutoRoboticInspection-V1\\VIKO_UltraRobot\\src\\Infer_software.py";
        static char workDir[256] = "E:\\Quan\\AutoRoboticInspection-V1\\VIKO_UltraRobot";
        static int theme_idx = 0;
        static int creating_speed = 0;
        static float SSXA_Ratio = 3.0f;
        static bool lock_frame = true;
        // Update for MeshImporterOption

        // Load settings
        static bool loading_flag = true;
        if (loading_flag)
        {
            try
            {
                robinit->get_settings("theme", theme_idx);
                // robinit->get_settings("rob_font", rob_font);
                robinit->get_settings("SSXA_Ratio", SSXA_Ratio);
                robinit->get_settings("robot_tcp", robot_tcp);
                robinit->get_settings("creating_speed", creating_speed);
                robinit->get_settings("pythonPath", pythonPath);
                robinit->get_settings("scriptPath", scriptPath);
                robinit->get_settings("workDir", workDir);
				robinit->get_settings("lock_frame", lock_frame);
            }
            catch (const std::exception& e) {};
            loading_flag = false;
        }
        // UI
        ImGui::Begin("Settings", nullptr, ImGuiWindowFlags_NoDocking);
        ImGui::Combo("Theme", &theme_idx, theme_items, IM_ARRAYSIZE(theme_items));
        if (ImGui::Button("Choose Font")) { OpenFontDialog(NULL); }
        ImGui::SameLine();
        ImGui::Text("Font: %s", rob_font.c_str());

        ImGui::Separator(); ImGui::Separator();
        ImGui::InputFloat("SSXA Ratio", &SSXA_Ratio, 1.0f, 6.0f);
        ImGui::InputText("Robot TCP Input", robot_tcp, sizeof(robot_tcp));
        ImGui::Combo("ReBuild Mesh Level", &creating_speed, creat_item, IM_ARRAYSIZE(creat_item));

        ImGui::InputText("Python Path", pythonPath, sizeof(pythonPath));
        ImGui::InputText("Script Path", scriptPath, sizeof(scriptPath));
        ImGui::InputText("Working Directory", workDir, sizeof(workDir));

        ImGui::Checkbox("Unlock All Frames:", &lock_frame);

        if (ImGui::Button("Save")) {
            robinit->update_settings("theme", theme_idx);
            robinit->update_settings("rob_font", rob_font);
            robinit->update_settings("SSXA_Ratio", SSXA_Ratio);
            robinit->update_settings("robot_tcp", std::string(robot_tcp));
            robinit->update_settings("creating_speed", creating_speed);
            robinit->update_settings("pythonPath", std::string(pythonPath));
            robinit->update_settings("scriptPath", std::string(scriptPath));
            robinit->update_settings("workDir", std::string(workDir));
            robinit->update_settings("lock_frame", lock_frame);
            robinit->SaveInit_encode();
            OptionSetting_Flag = false;
            loading_flag = true;
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel")) {
            OptionSetting_Flag = false;
            loading_flag = true;
        }
        ImGui::End();
    }




    void HotkeyMenubar::OpenFileDialog()
    {
        OPENFILENAME ofn;
        char szFile[260] = { 0 };

        ZeroMemory(&ofn, sizeof(ofn));
        ofn.lStructSize = sizeof(ofn);
        ofn.hwndOwner = NULL;
        ofn.lpstrFile = szFile;
        ofn.nMaxFile = sizeof(szFile);
        ofn.lpstrFilter = { "FBX (*.fbx)\0*.fbx\0"
                            "OBJ (*.obj)\0*.obj\0"
                            "STL (*.stl)\0*.stl\0"
                            "glTF (*.glTF)\0*.glTF\0"
                            "All (*.*)\0*.*\0" };
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

    void HotkeyMenubar::hint(bool show)
    {
        std::lock_guard<std::mutex> lock(mtx);
        robinit->get_settings("theme",theme);
        static float pos_x, pos_y;
        scene_view->GetViewPos(pos_x, pos_y);
        ImGui::SetNextWindowPos(ImVec2(pos_x + 5, pos_y + 250 + 25));
        ImGui::Begin("Hint", nullptr,
            ImGuiWindowFlags_NoTitleBar | // Do not display title bar
            ImGuiWindowFlags_NoBackground | // Do not display background
            ImGuiWindowFlags_NoBringToFrontOnFocus); // Does not bring to front on focus
        if (theme == "dark")
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.6f, 0.8f, 0.6f, 0.3f)); // Set text color to white and 50% transparent
        else
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.3f, 0.0f, 0.0f, 0.3f)); // Set text color to red and 50% transparent
        ImGui::Text("Special Hotkey");
        ImGui::Text("W: Pan Up");
        ImGui::Text("S: Pan Down");
        ImGui::Text("A: Pan Left");
        ImGui::Text("D: Pan Right");
        ImGui::Text("Q: Pan Forward");
        ImGui::Text("E: Pan Backward");
        ImGui::Text("*R: Focus to Selected Object");
        ImGui::PopStyleColor();
        ImGui::End();
    }

    void HotkeyMenubar::OpenFontDialog(HWND owner) {
        OPENFILENAME ofn;       // Common dialog box structure
        char szFile[260] = { 0 }; // Buffer for file name

        // Initialize OPENFILENAME structure
        ZeroMemory(&ofn, sizeof(ofn));
        ofn.lStructSize = sizeof(ofn);
        ofn.hwndOwner = owner;
        ofn.lpstrFile = szFile;
        ofn.nMaxFile = sizeof(szFile);
        ofn.lpstrFilter = "Fonts\0*.TTF;*.OTF\0All\0*.*\0";
        ofn.nFilterIndex = 1;
        ofn.lpstrFileTitle = NULL;
        ofn.nMaxFileTitle = 0;
        ofn.lpstrInitialDir = "RobFonts"; // Set initial directory to the RobFonts folder
        ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;

        std::string extFileName;

        // Display the Open dialog box 
        if (GetOpenFileName(&ofn) == TRUE) {
            std::filesystem::path selectedPath(szFile);
            std::string directoryPath = selectedPath.parent_path().string();
            const std::string fileNameStem = selectedPath.stem().string(); // Extract the stem from the selected file

            // Search in the directory of the selected file
            bool found = false;
            for (const auto& entry : std::filesystem::directory_iterator(directoryPath)) {
                if (entry.is_regular_file() && entry.path().filename().stem() == fileNameStem) {
                    rob_font = entry.path().filename().stem().string();
                    found = true;
                    break; // Stop the loop once the file is found
                }
            }

            if (!found) {
                rob_font = ""; // Clear the filename if no match is found
            }
        }
    }

}