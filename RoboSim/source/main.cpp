#include "pch.h"
#include "application.h"

int main(void)
{

  //FreeConsole();

  auto app = std::make_unique<Application>("RobWeld");
  app->loop();

  return 0;
}
