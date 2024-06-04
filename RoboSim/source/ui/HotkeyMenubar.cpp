#include "pch.h"
#include "HotkeyMenubar.h"
namespace nui {
	bool HotkeyMenubar::waitloop[6] = { false,false,false,false,false,false };
	bool HotkeyMenubar::shint = true;

	void HotkeyMenubar::mMenuBar(GLFWwindow* mWindow)
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
                    const char* menuItems[7] = { "Points", "WireFrame", "Surface" };
                    for (int i = 0; i < 3; ++i) {
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

            if (ImGui::BeginMenu("Robot"))
            {
                if (ImGui::MenuItem("Connect")) { ym_con->set_connect_trigger(true); }
                ImGui::EndMenu();
            }
            ImGui::EndMainMenuBar();
        }
    }

    void HotkeyMenubar::mHotkey(GLFWwindow* mWindow)

    {


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
        if (glfwGetKey(mWindow, GLFW_KEY_R) == GLFW_PRESS && (lCtr || rCtr) && tdelay || waitloop[1])
        {
            lastPressTime = currentTime; waitloop[1] = true;
            uiaction.RotateOb_uiAction(waitloop[1]);
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
        if (nui::FrameManage::getCrActiveGui("ViewPort") == true)
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
                hint(shint);
            }
        }
    }

    void HotkeyMenubar::SaveIniFile(const std::string& key, const std::string& value)
    {
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
        nrender::UIContext::get_theme(theme);
        static float pos_x, pos_y;
        if (!pos_x || !pos_y) { nui::FrameManage::getViewportSize(pos_x, pos_y); }
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

}