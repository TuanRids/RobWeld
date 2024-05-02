#pragma once

#include "pch.h"

namespace nelems
{
  enum class EInputButton
  {
    Left = 0,
    Right = 1,
    Middle = 2,
    key_A = 3,
    key_D = 4,
    key_Q = 5,
    key_E = 6,
    key_R = 7,
    None = 9
  };

  class Input
  {
  public:
    static EInputButton GetPressedButton(GLFWwindow* window)
    {
      EInputButton result = EInputButton::None;

      if (glfwGetMouseButton(window, 0) == GLFW_PRESS)
        return EInputButton::Left;
      else if (glfwGetMouseButton(window, 1) == GLFW_PRESS)
        return EInputButton::Right;
      else if (glfwGetMouseButton(window, 2) == GLFW_PRESS)
        return EInputButton::Middle;
      else if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        return EInputButton::key_A;
      else if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
          return EInputButton::key_D;
	  else if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS)
		return EInputButton::key_Q;
	  else if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS)
		return EInputButton::key_E;
	  else if (glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS)
		return EInputButton::key_R;

      return EInputButton::None;

    }
  };
}
