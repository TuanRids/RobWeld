#include "pch.h"
#include "FrameManage.h"

namespace nui {
    float FrameManage::viewport_x = 10.0f;
    float FrameManage::viewport_y = 10.0f;
    float FrameManage::vdsize_x = 10.0f;
    float FrameManage::vdsize_y = 10.0f;
    bool FrameManage::getCrActiveGui(const std::string& frameName) {
        return getInstance().crActiveGui[frameName];
    }

    void FrameManage::setCrActiveGui(const std::string& frameName, bool isActive) {
        getInstance().crActiveGui[frameName] = isActive;
    }

    FrameManage& FrameManage::getInstance() {

        static FrameManage instance;
        return instance;
    }
}
