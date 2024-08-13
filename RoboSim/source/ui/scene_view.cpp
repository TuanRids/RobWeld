#include "pch.h"
#include "scene_view.h"
#include <Windows.h>

namespace nui
{
    std::string SceneView::arg_render_mode = "Surface";
    SceneView::SceneView():zoom(1), mSize(800, 600)
    {
        float SXAA = 2.5;
        robinit->get_settings("SSXA_Ratio", SXAA);

        mFrameBuffer = std::make_shared<nrender::OpenGL_FrameBuffer>();
        mFrameBuffer->create_buffers(static_cast<int>(800 * SXAA), static_cast<int>(600 * SXAA));
        // mShader = std::make_shared<nshaders::Shader>();
        mShader->load("shaders/vs.vert", "shaders/fs_pbr.frag");
    }

    void SceneView::resize(int32_t width, int32_t height)
    {
        mSize.x = float(width);
        mSize.y = float(height);
        mFrameBuffer->create_buffers(static_cast<int32_t>(mSize.x), static_cast<int32_t>(mSize.y));
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
            if (mesh->selected)
            {
                mCamera->set_rotation_center(mesh->oMaterial.position);
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
#pragma optimize("", off)

    void SceneView::render()
    {
        GLuint processedTextureID;
        mShader->use();
        mLight->update(mShader.get());

        mFrameBuffer->bind();
        render_mode(); 
        mFrameBuffer->unbind();
        GLenum error = glGetError();
        if (error != GL_NO_ERROR) {
            std::cerr << "OpenGL error after rendering to framebuffer: " << error << std::endl;
        }

        ImGui::Begin("ViewPort");
        {
            setCrActiveGui("ViewPort", ImGui::IsWindowHovered());
            ImVec2 viewportPanelSize = ImGui::GetContentRegionAvail();
            mSize = { viewportPanelSize.x, viewportPanelSize.y };

            mCamera->set_aspect(mSize.x / mSize.y);
            mCamera->update(mShader.get());
            ImVec2 viewportWindowPos = ImGui::GetWindowPos();
            SetViewPos(viewportWindowPos.x, viewportWindowPos.y);
            set3DSize(mSize.x, mSize.y);

            // Add rendered texture to ImGUI scene window
            uint64_t textureID = mFrameBuffer->get_texture();

            // Ensure the texture ID is valid before using it
            if (textureID != 0) {
                cv::Mat image = framebufferToMat(textureID, static_cast<int>(viewportPanelSize.x), static_cast<int>(viewportPanelSize.y));
                cv::GaussianBlur(image, image, cv::Size(3, 3), 0);
                cv::Mat sharpImage;
                cv::Mat kernel = (cv::Mat_<float>(3, 3) <<
                    0, -1, 0,
                    -1, 5, -1,
                    0, -1, 0);
                cv::filter2D(image, sharpImage, image.depth(), kernel);

                matToTexture(image, processedTextureID);
                ImGui::Image(reinterpret_cast<void*>(processedTextureID), ImVec2{ mSize.x, mSize.y }, ImVec2{ 0, 1 }, ImVec2{ 1, 0 });
            } else {
                ImGui::Text("Error: FrameBuffer texture is invalid.");
            }
        }
        ImGui::End();
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
#pragma optimize("", on)

}
