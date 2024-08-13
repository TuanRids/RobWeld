#pragma once
#include <memory>
#include <string>
#include <unordered_map>
#include "IUIComponent.h"

namespace nui {
    /// <summary>
    /// Class use for calling the function of IUIComponent
    /// Use to communicate with other classes outside of ui
    /// </summary>
    class FrameManage : public IUIComponent {
        FrameManage() {};
    public:
        static FrameManage& getInstance() { static FrameManage instance; return instance; }
        ~FrameManage() = default;


        static void get3DSize(float& x, float& y){
            x = vdsize_x;
            y = vdsize_y;
        }
        static void GetViewPos(float& x, float& y){
            x = viewport_x;
            y = viewport_y;
        }
        static bool getCrActiveGui(const std::string& frameName) {
            return frameName == crActiveGui ? true : false;
        }
        
        static void update_chart_dat(const std::map<std::string, float>& newdata){Chartdata = newdata;}

    };

}
