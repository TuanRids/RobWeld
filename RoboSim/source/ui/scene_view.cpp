#include "pch.h"
#include "scene_view.h"
#include <Windows.h>
#include <opencv2/opencv.hpp>
namespace nui
{
    std::string SceneView::arg_render_mode = "Surface";
    SceneView::SceneView():zoom(1), mSize(800, 600)
    {
        rdMesh = &nelems::mMesh::getInstance();
        robinit = &RobInitFile::getinstance();
        mFrameBuffer = std::make_unique<nrender::OpenGL_FrameBuffer>();

        float SXAA = 2.5;
        std::string temptsxaa;
        robinit->get_settings("SSXA_Ratio", temptsxaa);
        if (!temptsxaa.empty()) {
            SXAA = std::stof(temptsxaa);
        }

        mFrameBuffer->create_buffers(800 * SXAA, 600 * SXAA);
        mShader = std::make_unique<nshaders::Shader>();
        mShader->load("shaders/vs.vert", "shaders/fs_pbr.frag");
        mLight = std::make_unique<nelems::Light>();
        mCamera = std::make_unique<nelems::Camera>(45.0f, 1.3f, 0.1f, 100.0f);
    }

    void SceneView::resize(int32_t width, int32_t height)
    {
        mSize.x = float(width);
        mSize.y = float(height);
        mFrameBuffer->create_buffers((int32_t)mSize.x, (int32_t)mSize.y);
    }

    void SceneView::on_mouse_move(double x, double y, nelems::EInputButton button)
    {
        mCamera->on_mouse_move(x, y, button);
    }

    void SceneView::on_mouse_wheel(double delta)
    {
        mCamera->on_mouse_wheel(delta * double(zoom));
    }

    void SceneView::set_rotation_center()
    {
        std::shared_ptr<nelems::oMesh> selMesh;
        for (auto it = rdMesh->getMesh()->begin(); it != rdMesh->getMesh()->end(); it++)
        {
            auto mesh = *it;
            if (selMesh->selected)
            {
                mCamera->set_rotation_center(selMesh->oMaterial.position);
                return;
            }
        }
        mCamera->set_rotation_center({0,0,0 });
    }
    void SceneView::setZoom(int newZoom)
    {
        zoom = newZoom;
    }
    
    void SceneView::load_mesh(const std::string& filepath, bool robot)
    {
        if (!rdMesh)
        {
            rdMesh = &nelems::mMesh::getInstance();
        }
        try
        {
            rdMesh->load(filepath, robot);
        }
        catch (const std::exception& e)
        {
            std::cerr << e.what() << std::endl;
        }
    }
    
    cv::Mat framebufferToMat(uint64_t ogl_texture_id, int width, int height) {
        glBindTexture(GL_TEXTURE_2D, ogl_texture_id);
        GLint gl_texture_width, gl_texture_height;

        // Retrieve texture dimensions
        glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &gl_texture_width);
        glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_HEIGHT, &gl_texture_height);

        // Allocate memory for texture data
        std::vector<unsigned char> gl_texture_bytes(gl_texture_width * gl_texture_height * 3);

        // Get the texture image
        glGetTexImage(GL_TEXTURE_2D, 0 /* mipmap level */, GL_BGR, GL_UNSIGNED_BYTE, gl_texture_bytes.data());

        // Create a cv::Mat object using the data
        cv::Mat texture_image = cv::Mat(gl_texture_height, gl_texture_width, CV_8UC3, gl_texture_bytes.data()).clone();

        // Returning a clone of the matrix to ensure that the data persists beyond the function scope
        return texture_image;
    }


    void matToTexture(const cv::Mat& mat, GLuint & textureID) {
        //glGenTextures(1, &textureID);
        glBindTexture(GL_TEXTURE_2D, textureID);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, mat.cols, mat.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, mat.data);
        glBindTexture(GL_TEXTURE_2D, 0);        
    }

    void SceneView::render()
    {
        GLuint processedTextureID;
        mShader->use();
        mLight->update(mShader.get());

        mFrameBuffer->bind();
        if (!rdMesh) { rdMesh = &nelems::mMesh::getInstance(); }
        else { render_mode(); }
        mFrameBuffer->unbind();

        ImGui::Begin("ViewPort");
        {
            nui::FrameManage::setCrActiveGui("ViewPort", ImGui::IsWindowFocused() || ImGui::IsWindowHovered());
            ImVec2 viewportPanelSize = ImGui::GetContentRegionAvail();
            mSize = { viewportPanelSize.x, viewportPanelSize.y };
            mCamera->set_aspect(mSize.x / mSize.y);
            mCamera->update(mShader.get());
            ImVec2 viewportWindowPos = ImGui::GetWindowPos();
            nui::FrameManage::setViewportSize(viewportWindowPos.x, viewportWindowPos.y);
            nui::FrameManage::set3DSize(mSize.x, mSize.y);

            // Add rendered texture to ImGUI scene window
            uint64_t textureID = mFrameBuffer->get_texture();
            //convert texture frambuffer to opencv mat
            cv::Mat image = framebufferToMat(textureID, viewportPanelSize.x, viewportPanelSize.y);
            cv::GaussianBlur(image, image, cv::Size(3, 3), 0);
            cv::Mat sharpImage;
            cv::Mat kernel = (cv::Mat_<float>(3, 3) <<
                0, -1, 0,
                -1, 5, -1,
                0, -1, 0);
            cv::filter2D(image, sharpImage, image.depth(), kernel);

            matToTexture(image, processedTextureID);
            ImGui::Image(reinterpret_cast<void*>(processedTextureID), ImVec2{ mSize.x, mSize.y }, ImVec2{ 0, 1 }, ImVec2{ 1, 0 });

        }
        ImGui::End();
        //glDeleteTextures(1, &processedTextureID);
    }

    void SceneView::render_mode()
    {
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable(GL_BLEND);
        glEnable(GL_MULTISAMPLE);
        glHint(GL_LINE_SMOOTH_HINT, GL_DONT_CARE);
        glEnable(GL_LINE_SMOOTH);

        glPointSize(2.0f);
        if (arg_render_mode == "Points") {
            glPolygonMode(GL_FRONT_AND_BACK, GL_POINT);
            rdMesh->update(mShader.get(), mLight->lightmode);
        }
        else if (arg_render_mode == "WireFrame") {
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            rdMesh->update(mShader.get(), mLight->lightmode);
        }
        else if (arg_render_mode == "Surface") {
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            rdMesh->update(mShader.get(), mLight->lightmode);
        }
    }
}
