#pragma once

#include "render_base.h"
#include "GLFW/glfw3.h"
#include "ui_context.h"
namespace nrender
{
  class OpenGL_Context : public RenderContext
  {
  public:

    bool init(nwindow::IWindow* window) override;

    void pre_render() override;

    void post_render() override;


    void end() override;
  };
}
