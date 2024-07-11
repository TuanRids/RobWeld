#pragma once

#include <vector>
#include <optional>
#include <glm/glm.hpp>
#include <cmath>
#include "rigging/point.h"
#include "elems/mesh.h"

bool pt_dis(nelems::oMesh& target, Point& point)
{
    // Calculate distance between target and point
    float distance = sqrt(pow(target.oMaterial.position.x - point.x, 2) + pow(target.oMaterial.position.y - point.y, 2) + pow(target.oMaterial.position.z - point.z, 2));
    return distance < 0.1;
}

class RiggedObject
{
private:
    std::vector<Point> points{};
    std::shared_ptr<nelems::oMesh> target;
    nelems::mMesh* proMesh;
    // Speed [0.01 to 0.99];
    float speed{ 0.1 };
    float delay{ 0.1 };
    float progress{ 0.0f };
    int currentPointIndex{ 0 };

public:
    RiggedObject() : proMesh(nullptr), target(nullptr)
    {
        points.push_back(Point{ 100.0f, 0.0f, 100.0f });
        points.push_back(Point{ 200.0f, 0.0f, 100.0f });
    }
    void add_point(Point& addpt) { points.push_back(addpt); }
    void rem_point(Point& rempt) { points.erase(std::remove(points.begin(), points.end(), rempt), points.end()); }
    ~RiggedObject() { points.clear(); }
    void set_speed(float spd)
    {
        if (spd <= 0.99 && spd >= 0.01)
            speed = spd;
        else
            throw std::invalid_argument("Speed must be between 0.01 and 0.99");
    }
    void set_target()
    {
        if (proMesh)
        {
            for (auto it = proMesh->getMesh()->begin(); it != proMesh->getMesh()->end(); it++)
            {
                auto target = *it;
                if (target->selected)
                {
                    return;
                }
            }
        }
        target = nullptr; // No target found
    }
    void move_to_pointA(int ptA)
    {
        if (target)
        {
            target->move(points[ptA].x, points[ptA].y, points[ptA].z);
        }
    }

    void update(float deltaTime, bool& move_trigger)
    {
        if (!proMesh)
        {
            proMesh = &nelems::mMesh::getInstance();
            set_target();
        }

        if (!target)
        {
            return; // No target to move
        }

        if (currentPointIndex < points.size())
        {
            const Point& targetPoint = points[currentPointIndex];

            glm::vec3 direction = glm::vec3(targetPoint.x - target->oMaterial.position.x,
                targetPoint.y - target->oMaterial.position.y,
                targetPoint.z - target->oMaterial.position.z);
            float distance = glm::length(direction);
            direction = glm::normalize(direction);

            float step = speed * deltaTime;
            if (distance < step)
            {
                target->move(targetPoint.x, targetPoint.y, targetPoint.z);
                currentPointIndex++;
                progress = 0.0f;
            }
            else
            {
                target->move(target->oMaterial.position.x + direction.x * step,
                    target->oMaterial.position.y + direction.y * step,
                    target->oMaterial.position.z + direction.z * step);
                progress += step;
            }
        }
        else
        {
            if (move_trigger)
            {
                currentPointIndex = 0;
                move_trigger = false;
            }
        }
    }
    void render_target()
    {
        if (target)
        {
            std::cout << "rendering"
                << target->oMaterial.position.x << ","
                << target->oMaterial.position.y << ","
                << target->oMaterial.position.z << std::endl;
        }
    }
};