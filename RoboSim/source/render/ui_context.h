#pragma once

#include "render_base.h"
#include "Filemgr/RobInitFile.h"

namespace nrender
{
  class UIContext : public RenderContext
  {
  private:
      static std::string theme;
      RobInitFile* robinit;
  public:

    bool init(nwindow::IWindow* window) override;

    void pre_render() override;
    static void get_theme(std::string& gtheme) { gtheme = theme; }
    void post_render() override;

    void end() override;
    // hello world
  };
}
