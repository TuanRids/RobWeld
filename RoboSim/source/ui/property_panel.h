#pragma once

#include "elems/light.h"
#include "ui/scene_view.h"

#include "imgui.h"
#include "utils/imgui_widgets.h"
#include <ImFileBrowser.h>
#include <Windows.h>
#include <Commdlg.h>
namespace nui
{
  class Property_Panel
  {
  public:

    Property_Panel()
    {
      mCurrentFile = "< ... >";

      mFileDialog.SetTitle("Import Mesh");
      mFileDialog.SetFileFilters({ ".fbx", ".obj",".stl"});

      ImGuiIO& io = ImGui::GetIO();
      io.Fonts->AddFontFromFileTTF("C:/Windows/Fonts/Arial.ttf", 16.0f);

      showMenuBar = true;
    }
       void MenuBar();
    void render(nui::SceneView* mScene);

    void set_mesh_load_callback(const std::function<void(const std::string&)>& callback)
    {
      mMeshLoadCallback = callback;
    }
    void OpenFileDialog();
  private:
    bool showMenuBar;
    // create a file browser instance
    ImGui::FileBrowser mFileDialog;

    std::function<void(const std::string&)> mMeshLoadCallback;

    std::string mCurrentFile;


  };
}


