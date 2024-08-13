#pragma once

#include "render_base.h"
#include "Filemgr/RobInitFile.h"
struct ThemeColors {
    ImVec4 winbg;
    ImVec4 menubarbg;
    ImVec4 separator;
    ImVec4 header;
    ImVec4 headerhovered;
    ImVec4 headeractive;
    ImVec4 button;
    ImVec4 buttonhovered;
    ImVec4 buttonactive;
    ImVec4 framebg;
    ImVec4 framebghovered;
    ImVec4 framebgactive;
    ImVec4 tab;
    ImVec4 tabhovered;
    ImVec4 tabactive;
    ImVec4 tabunfocused;
    ImVec4 tabunfocusedhovered;
    ImVec4 titlebg;
    ImVec4 titlebgactive;
    ImVec4 titlebgcollapsed;
    ImVec4 textcolor;
    ImVec4 scrollbarbg;
};

namespace nrender
{
  class UIContext : public RenderContext
  {
  private:
      static std::string theme;
      RobInitFile* robinit;
      ThemeColors get_colortheme(std::string theme);
  public:

    bool init(nwindow::IWindow* window) override;

    void pre_render() override;
    static void get_theme(std::string& gtheme) { gtheme = theme; }
    void post_render() override;

    void end() override;
    // hello world
  };
}
