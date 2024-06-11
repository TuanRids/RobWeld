#pragma once

#include <Eigen/Dense>
#include "mesh.h"

namespace nelems {

    class TranRBMatrix {
    public:
        static void applyTransformation(oMesh& mesh, const glm::vec3& center, float angleX, float angleY, float angleZ);

    };

}
