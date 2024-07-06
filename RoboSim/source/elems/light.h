#pragma once

#include "pch.h"
#include "element.h"
#include "shader/shader_util.h"

namespace nelems
{
  class Light : public Element
  {
  public:

    Light()
    {
      mColor = glm::vec3(1.0f, 1.0f, 1.0f);
      mPosition = { 1.5f, 3.5f, 3.0f };
      mStrength = 100.0;
      lightmode = 1;
    }

    ~Light() {}

    void update(nshaders::Shader* shader) override
    {
      shader->set_vec3(mPosition, "lightPosition");
      shader->set_vec3(mColor * static_cast<float>(1000 * mStrength), "lightColor");
      shader->set_i1(lightmode, "LightModes");

    }

    glm::vec3 mPosition;

    glm::vec3 mColor;

    float mStrength;
    int lightmode;
  };
}