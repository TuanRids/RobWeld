#include "pch.h"
#include "IUIComponent.h"

namespace nui
{

    std::string IUIComponent::crActiveGui;
    float IUIComponent::viewport_x = 10.0f;
    float IUIComponent::viewport_y = 10.0f;
    float IUIComponent::vdsize_x = 10.0f;
    float IUIComponent::vdsize_y = 10.0f;

    RobInitFile* IUIComponent::robinit = &RobInitFile::getinstance();
    nelems::mMesh* IUIComponent::rdMesh = &nelems::mMesh::getInstance();
    std::unique_ptr<nelems::Camera> IUIComponent::mCamera = std::make_unique<nelems::Camera>();
    std::unique_ptr<nelems::Light> IUIComponent::mLight = std::make_unique<nelems::Light>();
    std::unique_ptr<nshaders::Shader> IUIComponent::mShader = std::make_unique<nshaders::Shader>();
    std::map<std::string, float> IUIComponent::Chartdata = { {"Weld A", 60.2f },{"Weld B", 75.5f },{"Weld C", 85.4f },{"Weld D", 46.0f } };

    void IUIComponent::render()
    {
        // Implement render logic
    }

    void IUIComponent::resize(int32_t width, int32_t height)
    {
        // Implement resize logic
    }

    bool IUIComponent::getCrActiveGui(const std::string& frameName) {
        return frameName == crActiveGui ? true : false;
    }

    void IUIComponent::setCrActiveGui(const std::string& frameName, bool isActive) {
        if (isActive) { crActiveGui = frameName; }
    }

    void IUIComponent::set3DSize(float& x, float& y) {
        vdsize_x = x;
        vdsize_y = y;
    }

    void IUIComponent::get3DSize(float& x, float& y) {
        x = vdsize_x;
        y = vdsize_y;
    }

    void IUIComponent::GetViewPos(float& x, float& y) {
        x = viewport_x;
        y = viewport_y;
    }

    void IUIComponent::SetViewPos(float& x, float& y) {
        viewport_x = x;
        viewport_y = y;
    }
}
