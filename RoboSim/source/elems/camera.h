#pragma once

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>

#include "input.h"
#include "element.h"
#include "shader/shader_util.h"


namespace nelems
{
	
	class Camera : public Element
	{
	public:

		Camera( float fov, float aspect, float near, float far)
		{
			mAspect = aspect;
			mNear = near;
			mFar = far;
			mFOV = fov;

			set_aspect(mAspect);
			update_view_matrix();
		}

		void update(nshaders::Shader* shader) override
		{
			glm::mat4 model{ 1.0f };
			shader->set_mat4(model, "model");
			shader->set_mat4(mViewMatrix, "view");
			shader->set_mat4(get_projection(), "projection");
			shader->set_vec3(mPosition, "camPos");
		}

    void set_aspect(float aspect)
    {
     mProjection = glm::perspective(mFOV, aspect, mNear, mFar);
    }

	void set_distance(float offset)
	{
		mDistance += offset;
		update_view_matrix();
	}

	const glm::mat4& get_projection() const
	{
		return mProjection;
	}

    glm::mat4 get_view_projection() const
    {
      return mProjection * get_view_matrix();
    }

	glm::vec3 get_up() const
	{
		return glm::rotate(get_direction(), cUp);
	}

	glm::vec3 get_right() const
	{
		return glm::rotate(get_direction(), cRight);
	}

	glm::vec3 get_forward() const
	{
		return glm::rotate(get_direction(), cForward);
	}

	glm::quat get_direction() const
	{
		return glm::quat(glm::vec3(-mPitch, -mYaw, 0.0f));
	}

	glm::mat4 get_view_matrix() const
	{
		return mViewMatrix;
	}

	void on_mouse_wheel(double delta)
	{
		set_distance(float(delta) * 0.5f);

		update_view_matrix();
	}
	void set_rotation_center(const glm::vec3& position)
	{
		// Update the focus to the new position
		mFocus = {position.x,position.z,-position.y};

		// Set the distance to 1 to zoom in to the new position
		mDistance = 900.0f;

		// Calculate the direction vector from the current position to the new position
		glm::vec3 direction = mFocus - mPosition;

		// Update the position of the camera along the direction vector
		mPosition += direction;

		// Update the view matrix to look at the new position
		glm::quat orientation = get_direction();
		mViewMatrix = glm::translate(glm::mat4(1.0f), mPosition) * glm::toMat4(orientation);
		mViewMatrix = glm::inverse(mViewMatrix);
	}



	void reset()
	{
		mFocus = { 0.0f, 0.0f, 0.0f };
		//mDistance = 5.0f;
		update_view_matrix();
	}
	void set_fov(float fov)
	{
		mFOV = fov;
		update_projection_matrix();
	}

	void set_near(float near)
	{
		mNear = near;
		update_projection_matrix();
	}

	void set_far(float far)
	{
		mFar = far;
		update_projection_matrix();
	}

	void on_mouse_move(double x, double y, EInputButton button)
	{
		glm::vec2 pos2d{ x, y };

		if (button == EInputButton::Right)
		{
			glm::vec2 delta = (pos2d - mCurrentPos2d) * 0.004f;

			float sign = get_up().y < 0 ? -1.0f : 1.0f;
			mYaw += sign * delta.x * cRotationSpeed;
			mPitch += delta.y * cRotationSpeed;

			update_view_matrix();
		}	
		else if (button == EInputButton::Middle)
		{
			// TODO: Adjust pan speed for distance
			glm::vec2 delta = (pos2d - mCurrentPos2d) * 0.003f;

			mFocus += -get_right() * delta.x * mDistance;
			mFocus += get_up() * delta.y * mDistance;

			update_view_matrix();
		}
		else if (button == EInputButton::key_A)
		{
			glm::vec2 delta{x * 0.004f,0 };
			mFocus += -get_right() * delta.x ;
			update_view_matrix();
		}
		else if (button == EInputButton::key_D)
		{
			glm::vec2 delta{ x * 0.004f,0 };
			mFocus += get_right() * delta.x;
			update_view_matrix();
		}
		else if (button == EInputButton::key_Q)
		{
			glm::vec2 delta{ 0,y * 0.004f };
			mFocus += -get_up() * delta.y;
			update_view_matrix();
		}
		else if (button == EInputButton::key_E)
		{
			glm::vec2 delta{ 0,y / 1000 };
			mFocus += get_up() * delta.y;
			update_view_matrix();
		}


		mCurrentPos2d = pos2d;
	}
	void update_projection_matrix()
	{
		// this will be called when the aspect ratio changes
		//mProjection = glm::perspective(mFOV, mAspect, mNear, mFar);
	}

	void update_view_matrix()
	{
		mPosition =  mFocus - get_forward() * mDistance;

		glm::quat orientation = get_direction();
		mViewMatrix = glm::translate(glm::mat4(1.0f), mPosition) * glm::toMat4(orientation);
		mViewMatrix = glm::inverse(mViewMatrix);
	}
	glm::vec3 get_position() const
	{
		return mPosition;
	}


	private:
		std::shared_ptr<std::string> crActiveGui = std::make_shared<std::string>("none"); // check current active gui

		glm::mat4 mViewMatrix;
		glm::mat4 mProjection = glm::mat4{ 1.0f };
		glm::vec3 mPosition = { 0.0f, 0.0f, 0.0f };

		glm::vec3 mFocus = { 400.0f, 0.0f, -100.0f };

		float mDistance = 955.0f;
		float mAspect;
		float mFOV;
		float mNear;
		float mFar;

		float mPitch = 0.43f;
		float mYaw = -3.0f;

		glm::vec2 mCurrentPos2d = { 0.0f,0.0f };

		const glm::vec3 cRight = { 1.0f, 0.0f, 0.0f };
		const glm::vec3 cUp = { 0.0f, 1.0f, 0.0f };
		const glm::vec3 cForward = { 0.0f, 0.0f, -1.0f };

		const float cRotationSpeed = 2.0f;

	};
}

