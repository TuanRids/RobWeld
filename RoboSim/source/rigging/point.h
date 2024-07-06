#pragma once
#include "pch.h"
struct Point {
	float x, y, z;
	bool operator==(const Point& other) const {
		return x == other.x && y == other.y && z == other.z;
	}
	bool operator!=(const Point& other) const {
		return x != other.x || y != other.y || z != other.z;
	}
};

