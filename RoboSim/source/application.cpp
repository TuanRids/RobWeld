#include "pch.h"

#include "application.h"

#include "window/jgl_window.h"

Application::Application(const std::string& app_name)
{
  mWindow = std::make_unique<nwindow::GLWindow>();
  mWindow->init(app_name);
  mWindow->set_icon("assets/Robo.png");

}

void Application::loop()
{
  
  while (mWindow->is_running())
  {
    mWindow->render();

  }
}