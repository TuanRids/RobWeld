#pragma once
#include <memory>
#include <string>
#include <unordered_map>

namespace nui {
    /*
    FrameManage is a singleton class that manages the active GUI frame
    CAN BE USED TO MANAGE OTHER FRAME PROPERTIES IN FUTURE
    Current: crActiveGui
    */
    class FrameManage {
    public:
        // Getter for crActiveGui variable
        static bool getCrActiveGui(const std::string& frameName);

        // Setter for crActiveGui variable
        static void setCrActiveGui(const std::string& frameName, bool isActive);

        //getter for viewport position
        static void getViewportSize(float& x, float& y) {x = viewport_x;y = viewport_y;}
        //setter for viewport position
        static void setViewportSize(float& x, float& y) 
        {
            viewport_x = x;
            viewport_y = y;
        }

    private:
        // Private constructor to ensure no other FrameManage objects can be created outside the singleton
        FrameManage() {}

        // Static method to return the unique instance of FrameManage
        static FrameManage& getInstance();

        // crActiveGui variable stored in FrameManage and shared via singleton
        std::unordered_map<std::string, bool> crActiveGui;

        // viewport position
        static float viewport_x, viewport_y;
    };

}
