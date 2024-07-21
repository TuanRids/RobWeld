#pragma once
#include "pch.h"
#include "../elems/mesh.h"

namespace nmesh_import {

	struct IMeshImporter
	{
    virtual bool from_file(const std::string& filepath, nelems::oMesh* pMesh) = 0;
	};
}