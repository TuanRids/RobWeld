#pragma once

#include "render_base.h"

namespace nrender
{
  class UIContext : public RenderContext
  {
      static std::string theme;
  public:

    bool init(nwindow::IWindow* window) override;
    std::string ReadThemeFromFile();
    void pre_render() override;
    static std::string get_theme(){return theme;}
    void post_render() override;

    void end() override;

  };
}
