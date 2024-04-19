#pragma once

#include "import_base.h"
#include "../elems/mesh.h"
namespace nmesh_import {

	class ObjMeshImporter : public IMeshImporter
	{
	public:
		virtual bool from_file(const std::string& filepath, nelems::oMesh* pMesh) override;
	};
}
