#include "pch.h"
#include "application.h"

int main(void)
{
 
#ifdef _DEBUG
    AllocConsole();
    freopen("CONOUT$", "w", stdout);
    freopen("CONOUT$", "w", stderr);
    std::cout << "Debug mode: Console is visible." << std::endl;
#else
    FreeConsole();
#endif
  auto app = std::make_unique<Application>("RoboSim");
  app->loop();

  return 0;
}
