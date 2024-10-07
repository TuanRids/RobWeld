#include "pch.h"

#include "ui_context.h"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "ui\property_panel.h"

#pragma warning( push )
#pragma warning( disable : 26819) //3rd party library
#include "nlohmann/json.hpp"
#pragma warning( pop ) 
#include "cfreader.h"
namespace nrender
{
    std::string UIContext::theme = "Light";
    bool UIContext::init(nwindow::IWindow* window)
    {
        __super::init(window);

        // GL 4.5 + GLSL 450
        const char* glsl_version = "#version 450";

        // Setup Dear ImGui context
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImGuiIO& io = ImGui::GetIO(); (void)io;
        io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;       // Enable Keyboard Controls
        io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;           // Enable Docking
        io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;         // Enable Multi-Viewport / Platform Windows

        auto& colors = ImGui::GetStyle().Colors;


        if (!robinit) { robinit = &RobInitFile::getinstance(); }
        // TODO: ADJUST ALL COLOR           #TODOCOLOR TODO COLOR
        theme = robinit->get_settings("theme");
        // ini var
        ThemeColors ct_colors = get_colortheme(theme);

        colors[ImGuiCol_WindowBg] =         Cfigreader(theme, "winbg", ImVec4(0.13f, 0.14f, 0.26f, 1.0f));
        colors[ImGuiCol_MenuBarBg] =        Cfigreader(theme, "menubarbg", ImVec4(0.08f, 0.15f, 0.21f, 1.0f));
        colors[ImGuiCol_Separator] =        Cfigreader(theme, "separator", ImVec4(0.35f, 0.21f, 0.16f, 1.0f));
        colors[ImGuiCol_Header] =           Cfigreader(theme, "header", ImVec4(0.32f, 0.2f, 0.4f, 1.0f));
        colors[ImGuiCol_HeaderHovered] =    Cfigreader(theme, "headerhovered", ImVec4(0.3f, 0.3f, 0.3f, 1.0f));
        colors[ImGuiCol_HeaderActive] =     Cfigreader(theme, "headeractive", ImVec4(0.23f, 0.24f, 0.40f, 1.0f));
        colors[ImGuiCol_Button] =           Cfigreader(theme, "button", ImVec4(0.13f, 0.14f, 0.40f, 1.0f));
        colors[ImGuiCol_ButtonHovered] =    Cfigreader(theme, "buttonhovered", ImVec4(0.3f, 0.3f, 0.3f, 1.0f));
        colors[ImGuiCol_ButtonActive] =     Cfigreader(theme, "buttonactive", ImVec4(0.15f, 0.15f, 0.15f, 1.0f));
        colors[ImGuiCol_FrameBg] =          Cfigreader(theme, "framebg", ImVec4(0.13f, 0.14f, 0.40f, 1.0f));
        colors[ImGuiCol_FrameBgHovered] =   Cfigreader(theme, "framebghovered", ImVec4(0.3f, 0.3f, 0.3f, 1.0f));
        colors[ImGuiCol_FrameBgActive] =    Cfigreader(theme, "framebgactive", ImVec4(0.15f, 0.15f, 0.15f, 1.0f));
        colors[ImGuiCol_Tab] =              Cfigreader(theme, "tab", ImVec4(0.13f, 0.14f, 0.40f, 1.0f));
        colors[ImGuiCol_TabHovered] =       Cfigreader(theme, "tabhovered", ImVec4(0.3f, 0.3f, 0.3f, 1.0f));
        colors[ImGuiCol_TabActive] =        Cfigreader(theme, "tabactive", ImVec4(0.15f, 0.15f, 0.15f, 1.0f));
        colors[ImGuiCol_TabUnfocused] =     Cfigreader(theme, "tabunfocused", ImVec4(0.13f, 0.14f, 0.40f, 1.0f));
        colors[ImGuiCol_TabUnfocusedActive] = Cfigreader(theme, "tabunfocusedhovered", ImVec4(0.3f, 0.3f, 0.3f, 1.0f));
        colors[ImGuiCol_TitleBg] =          Cfigreader(theme, "titlebg", ImVec4(0.13f, 0.14f, 0.40f, 1.0f));
        colors[ImGuiCol_TitleBgActive] =    Cfigreader(theme, "titlebgactive", ImVec4(0.15f, 0.15f, 0.15f, 1.0f));
        colors[ImGuiCol_TitleBgCollapsed] = Cfigreader(theme, "titlebgcollapsed", ImVec4(0.13f, 0.14f, 0.40f, 1.0f));
        colors[ImGuiCol_Text] =             Cfigreader(theme, "textcolor", ImVec4(0.95f, 0.995f, 0.99f, 1.0f));
        colors[ImGuiCol_ScrollbarBg] =      Cfigreader(theme, "scrollbarbg", ImVec4(0.13f, 0.14f, 0.26f, 1.0f));


        // customize style for special frame

        ImGuiStyle& style = ImGui::GetStyle();
        if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
        {
            style.WindowRounding = 12.0f;
            style.Colors[ImGuiCol_WindowBg].w = 1.0f;
            style.FrameRounding = 3.0f;
            style.GrabRounding = 3.0f;
            style.WindowRounding = 8.0f;
            style.ChildRounding = 3.0f;
            style.PopupRounding = 3.0f;
            style.ScrollbarRounding = 8.0f;
            style.TabRounding = 6.0f;
            style.Colors[ImGuiCol_FrameBg].w = 0.7f;
        }

        // Setup Platform/Renderer backends

        ImGui_ImplGlfw_InitForOpenGL((GLFWwindow*)mWindow->get_native_window(), true);
        ImGui_ImplOpenGL3_Init(glsl_version);
        return true;
    }
    
    ThemeColors nrender::UIContext::get_colortheme(std::string theme)
    {
        if (theme == "Dark") {
            ImGui::StyleColorsDark();
            ThemeColors newcolor;
            newcolor.winbg                 = ImVec4{ 0.13f, 0.14f, 0.26f, 1.0f };
            newcolor.menubarbg             = ImVec4{ 0.08f, 0.15f, 0.21f, 1.0f };
            newcolor.separator             = ImVec4{ 0.35f, 0.21f, 0.16f, 1.0f };
            newcolor.header                = ImVec4{ 0.32f, 0.2f, 0.4f, 1.0f };
            newcolor.headerhovered         = ImVec4{ 0.3f, 0.3f, 0.3f, 1.0f };
            newcolor.headeractive          = ImVec4{ 0.23f, 0.24f, 0.40f, 1.0f };
            newcolor.button                = ImVec4{ 0.13f, 0.14f, 0.40f, 1.0f };
            newcolor.buttonhovered         = ImVec4{ 0.3f, 0.3f, 0.3f, 1.0f };
            newcolor.buttonactive          = ImVec4{ 0.15f, 0.15f, 0.15f, 1.0f };
            newcolor.framebg               = ImVec4{ 0.13f, 0.14f, 0.40f, 1.0f };
            newcolor.framebghovered        = ImVec4{ 0.3f, 0.3f, 0.3f, 1.0f };
            newcolor.framebgactive         = ImVec4{ 0.15f, 0.15f, 0.15f, 1.0f };
            newcolor.tab                   = ImVec4{ 0.13f, 0.14f, 0.40f, 1.0f };
            newcolor.tabhovered            = ImVec4{ 0.3f, 0.3f, 0.3f, 1.0f };
            newcolor.tabactive             = ImVec4{ 0.15f, 0.15f, 0.15f, 1.0f };
            newcolor.tabunfocused          = ImVec4{ 0.13f, 0.14f, 0.40f, 1.0f };
            newcolor.tabunfocusedhovered   = ImVec4{ 0.3f, 0.3f, 0.3f, 1.0f };
            newcolor.titlebg               = ImVec4{ 0.13f, 0.14f, 0.40f, 1.0f };
            newcolor.titlebgactive         = ImVec4{ 0.15f, 0.15f, 0.15f, 1.0f };
            newcolor.titlebgcollapsed      = ImVec4{ 0.13f, 0.14f, 0.40f, 1.0f };
            newcolor.textcolor             = ImVec4{ 0.95f, 0.995f, 0.99f, 1.0f };
            newcolor.scrollbarbg           = ImVec4{ 0.13f, 0.14f, 0.26f, 1.0f };
            return newcolor;
        }
        else if (theme == "Light") {
            ImGui::StyleColorsLight();
            ThemeColors newcolor;
            newcolor.winbg                 = ImVec4{ 0.95f, 0.95f, 0.95f, 1.0f };
            newcolor.menubarbg             = ImVec4{ 0.85f, 0.85f, 0.85f, 1.0f };
            newcolor.separator             = ImVec4{ 0.7f, 0.7f, 0.7f, 1.0f };
            newcolor.header                = ImVec4{ 0.7f, 0.7f, 0.7f, 1.0f };
            newcolor.headerhovered         = ImVec4{ 0.8f, 0.8f, 0.8f, 1.0f };
            newcolor.headeractive          = ImVec4{ 0.6f, 0.6f, 0.6f, 1.0f };
            newcolor.button                = ImVec4{ 0.8f, 0.8f, 0.8f, 1.0f };
            newcolor.buttonhovered         = ImVec4{ 0.9f, 0.9f, 0.9f, 1.0f };
            newcolor.buttonactive          = ImVec4{ 0.7f, 0.7f, 0.7f, 1.0f };
            newcolor.framebg               = ImVec4{ 0.85f, 0.85f, 0.85f, 1.0f };
            newcolor.framebghovered        = ImVec4{ 0.9f, 0.9f, 0.9f, 1.0f };
            newcolor.framebgactive         = ImVec4{ 0.85f, 0.85f, 0.85f, 1.0f };
            newcolor.tab                   = ImVec4{ 0.8f, 0.8f, 0.8f, 1.0f };
            newcolor.tabhovered            = ImVec4{ 0.7f, 0.7f, 0.7f, 1.0f };
            newcolor.tabactive             = ImVec4{ 0.7f, 0.7f, 0.7f, 1.0f };
            newcolor.tabunfocused          = ImVec4{ 0.8f, 0.8f, 0.8f, 1.0f };
            newcolor.tabunfocusedhovered   = ImVec4{ 0.8f, 0.8f, 0.8f, 1.0f };
            newcolor.titlebg               = ImVec4{ 0.85f, 0.85f, 0.85f, 1.0f };
            newcolor.titlebgactive         = ImVec4{ 0.85f, 0.85f, 0.85f, 1.0f };
            newcolor.titlebgcollapsed      = ImVec4{ 0.85f, 0.85f, 0.85f, 1.0f };
            newcolor.textcolor             = ImVec4{ 0.01f, 0.1f, 0.01f, 1.0f };
            newcolor.scrollbarbg           = ImVec4{ 0.95f, 0.95f, 0.95f, 1.0f };
            return newcolor;
        }
        else if (theme == "DarkGreen") {
            ImGui::StyleColorsDark();
            ThemeColors newcolor;
            newcolor.winbg                 = ImVec4{ 0.0f, 0.15f, 0.08f, 1.0f };
            newcolor.menubarbg             = ImVec4{ 0.0f, 0.2f, 0.0f, 1.0f };
            newcolor.separator             = ImVec4{ 0.0f, 0.5f, 0.3f, 1.0f };
            newcolor.header                = ImVec4{ 0.0f, 0.4f, 0.0f, 1.0f };
            newcolor.headerhovered         = ImVec4{ 0.0f, 0.45f, 0.09f, 1.0f };
            newcolor.headeractive          = ImVec4{ 0.09f, 0.45f, 0.10f, 1.0f };
            newcolor.button                = ImVec4{ 0.0f, 0.3f, 0.0f, 1.0f };
            newcolor.buttonhovered         = ImVec4{ 0.07f, 0.45f, 0.20f, 1.0f };
            newcolor.buttonactive          = ImVec4{ 0.0f, 0.25f, 0.0f, 1.0f };
            newcolor.framebg               = ImVec4{ 0.0f, 0.2f, 0.0f, 1.0f };
            newcolor.framebghovered        = ImVec4{ 0.0f, 0.25f, 0.0f, 1.0f };
            newcolor.framebgactive         = ImVec4{ 0.0f, 0.15f, 0.0f, 1.0f };
            newcolor.tab                   = ImVec4{ 0.0f, 0.2f, 0.0f, 1.0f };
            newcolor.tabhovered            = ImVec4{ 0.0f, 0.25f, 0.0f, 1.0f };
            newcolor.tabactive             = ImVec4{ 0.0f, 0.15f, 0.0f, 1.0f };
            newcolor.tabunfocused          = ImVec4{ 0.0f, 0.2f, 0.0f, 1.0f };
            newcolor.tabunfocusedhovered   = ImVec4{ 0.0f, 0.25f, 0.0f, 1.0f };
            newcolor.titlebg               = ImVec4{ 0.0f, 0.2f, 0.0f, 1.0f };
            newcolor.titlebgactive         = ImVec4{ 0.0f, 0.15f, 0.0f, 1.0f };
            newcolor.titlebgcollapsed      = ImVec4{ 0.0f, 0.2f, 0.0f, 1.0f };
            newcolor.textcolor             = ImVec4{ 0.99f, 0.99f, 0.99f, 1.0f };
            newcolor.scrollbarbg           = ImVec4{ 0.0f, 0.15f, 0.0f, 1.0f };
            return newcolor;
        }
    }

    void UIContext::pre_render()
    {

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Create the docking environment
        ImGuiWindowFlags windowFlags = ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoTitleBar |
            ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus |
            ImGuiWindowFlags_NoBackground;

        ImGuiViewport* viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(ImVec2(viewport->Pos.x, viewport->Pos.y + 22));
        ImGui::SetNextWindowSize(viewport->Size);
        ImGui::SetNextWindowViewport(viewport->ID);

        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
        ImGui::Begin("InvisibleWindow", nullptr, windowFlags);
        ImGui::PopStyleVar(3);

        ImGuiID dockSpaceId = ImGui::GetID("InvisibleWindowDockSpace");

        ImGui::DockSpace(dockSpaceId, ImVec2(0.0f, 0.0f), ImGuiDockNodeFlags_PassthruCentralNode);
        ImGui::End();
    }

    void UIContext::post_render()
    {
        // Rendering
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        ImGuiIO& io = ImGui::GetIO();

        if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
        {
            GLFWwindow* backup_current_context = glfwGetCurrentContext();
            ImGui::UpdatePlatformWindows();
            ImGui::RenderPlatformWindowsDefault();
            glfwMakeContextCurrent(backup_current_context);
        }
    }

    void UIContext::end()
    {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
    }

}