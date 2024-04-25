#include "pch.h"
#include "FrameManage.h"

namespace nui {
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
