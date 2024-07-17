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

namespace nrender
{
    std::string UIContext::theme = "light";
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
    robinit->get_settings("theme", theme);
    if (theme == "Dark")
    {
        ImGui::StyleColorsDark();
        colors[ImGuiCol_WindowBg] = ImVec4{ 0.13f, 0.14f, 0.26f, 1.0f };

        colors[ImGuiCol_MenuBarBg] = ImVec4{ 0.08f, 0.15f, 0.21f, 1.0f };
        colors[ImGuiCol_Separator] = ImVec4{ 0.35f, 0.21f, 0.16f, 1.0f };


        colors[ImGuiCol_Header] = ImVec4{ 0.32f, 0.2f, 0.4f, 1.0f };
        colors[ImGuiCol_HeaderHovered] = ImVec4{ 0.3f, 0.3f, 0.3f, 1.0f };
        colors[ImGuiCol_HeaderActive] = ImVec4{ 0.15f, 0.15f, 0.15f, 1.0f };

        colors[ImGuiCol_Button] = ImVec4{ 0.2f, 0.2f, 0.2f, 1.0f };
        colors[ImGuiCol_ButtonHovered] = ImVec4{ 0.3f, 0.3f, 0.3f, 1.0f };
        colors[ImGuiCol_ButtonActive] = ImVec4{ 0.15f, 0.15f, 0.15f, 1.0f };

        colors[ImGuiCol_FrameBg] = ImVec4{ 0.18f, 0.2f, 0.35f, 1.0f };
        colors[ImGuiCol_FrameBgHovered] = ImVec4{ 0.3f, 0.3f, 0.3f, 1.0f };
        colors[ImGuiCol_FrameBgActive] = ImVec4{ 0.18f, 0.2f, 0.35f, 1.0f };

        colors[ImGuiCol_Tab] = ImVec4{ 0.15f, 0.15f, 0.15f, 1.0f };
        colors[ImGuiCol_TabHovered] = ImVec4{ 0.23f, 0.41f, 0.23f, 1.0f };
        colors[ImGuiCol_TabActive] = ImVec4{ 0.23f, 0.41f, 0.23f, 1.0f };
        colors[ImGuiCol_TabUnfocused] = ImVec4{ 0.15f, 0.15f, 0.15f, 1.0f };
        colors[ImGuiCol_TabUnfocusedActive] = ImVec4{ 0.2f, 0.2f, 0.2f, 1.0f };

        colors[ImGuiCol_TitleBg] = ImVec4{ 0.18f, 0.2f, 0.35f, 1.0f };
        colors[ImGuiCol_TitleBgActive] = ImVec4{ 0.18f, 0.2f, 0.35f, 1.0f };
        colors[ImGuiCol_TitleBgCollapsed] = ImVec4{ 0.18f, 0.2f, 0.35f, 1.0f };
    }
    else if (theme == "Light")
    {
        ImGui::StyleColorsLight();
        colors[ImGuiCol_Text] = ImVec4{ 0.0f, 0.0f, 0.0f, 1.0f };
        colors[ImGuiCol_WindowBg] = ImVec4{ 0.95f, 0.95f, 0.95f, 1.0f };

        colors[ImGuiCol_MenuBarBg] = ImVec4{ 0.85f, 0.85f, 0.85f, 1.0f };
        colors[ImGuiCol_Separator] = ImVec4{ 0.7f, 0.7f, 0.7f, 1.0f };

        colors[ImGuiCol_Header] = ImVec4{ 0.7f, 0.7f, 0.7f, 1.0f };
        colors[ImGuiCol_HeaderHovered] = ImVec4{ 0.8f, 0.8f, 0.8f, 1.0f };
        colors[ImGuiCol_HeaderActive] = ImVec4{ 0.6f, 0.6f, 0.6f, 1.0f };

        colors[ImGuiCol_Button] = ImVec4{ 0.8f, 0.8f, 0.8f, 1.0f };
        colors[ImGuiCol_ButtonHovered] = ImVec4{ 0.9f, 0.9f, 0.9f, 1.0f };
        colors[ImGuiCol_ButtonActive] = ImVec4{ 0.7f, 0.7f, 0.7f, 1.0f };

        colors[ImGuiCol_FrameBg] = ImVec4{ 0.85f, 0.85f, 0.85f, 1.0f };
        colors[ImGuiCol_FrameBgHovered] = ImVec4{ 0.9f, 0.9f, 0.9f, 1.0f };
        colors[ImGuiCol_FrameBgActive] = ImVec4{ 0.85f, 0.85f, 0.85f, 1.0f };

        colors[ImGuiCol_Tab] = ImVec4{ 0.8f, 0.8f, 0.8f, 1.0f };
        colors[ImGuiCol_TabHovered] = ImVec4{ 0.7f, 0.7f, 0.7f, 1.0f };
        colors[ImGuiCol_TabActive] = ImVec4{ 0.7f, 0.7f, 0.7f, 1.0f };
        colors[ImGuiCol_TabUnfocused] = ImVec4{ 0.8f, 0.8f, 0.8f, 1.0f };
        colors[ImGuiCol_TabUnfocusedActive] = ImVec4{ 0.8f, 0.8f, 0.8f, 1.0f };

        colors[ImGuiCol_TitleBg] = ImVec4{ 0.85f, 0.85f, 0.85f, 1.0f };
        colors[ImGuiCol_TitleBgActive] = ImVec4{ 0.85f, 0.85f, 0.85f, 1.0f };
        colors[ImGuiCol_TitleBgCollapsed] = ImVec4{ 0.85f, 0.85f, 0.85f, 1.0f };

        //colors[ImGuiCol_ChildBg] = ImVec4{ 0.85f, 0.85f, 0.85f, 1.0f };
        
        colors[ImGuiCol_NavHighlight] = ImVec4{ 0.8f, 0.8f, 0.8f, 1.0f };
        colors[ImGuiCol_PopupBg] = ImVec4{ 0.85f, 0.85f, 0.85f, 1.0f };
    }
    // customize style for special frame
    
    ImGuiStyle& style = ImGui::GetStyle();
    if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
    {
      style.WindowRounding = 0.0f;
      style.Colors[ImGuiCol_WindowBg].w = 1.0f;
    }

    // Setup Platform/Renderer backends

    ImGui_ImplGlfw_InitForOpenGL((GLFWwindow*)mWindow->get_native_window(), true);
    ImGui_ImplOpenGL3_Init(glsl_version);
    return true;
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