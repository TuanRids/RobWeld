#pragma once


#include "elems/mesh.h"
#include "Command/ObHistory.h"
namespace nui
{
	/*
	Manage UI Actions. set ObHistory at jgl_window.cpp
	 */
	class uiAction
	{
	private:
		nelems::mMesh* proMesh;
		ncommand::ObHistory* obAction;

	public:
		uiAction(): proMesh(nullptr), obAction(nullptr) {
			if (!proMesh) {proMesh = &nelems::mMesh::getInstance();
			obAction = &ncommand::ObHistory::getInstance();
			}
		}
		~uiAction() { proMesh = nullptr; obAction = nullptr; }

		void MoveOb_uiAction()
		{
			if (obAction == nullptr) return;

			std::unique_ptr<ncommand::Command> moveCmd = std::make_unique<ncommand::MoveOb>(proMesh, 10.0f, 10.0f, 10.0f);
			obAction->execmd(std::move(moveCmd)); }
		void undocmd()
		{
			if (obAction == nullptr) return;
			obAction->undocmd();
		}
		void redocmd()
		{
			if (obAction == nullptr) return;
			obAction->redocmd();
		}
		void Command_Logs()
		{;
			obAction->Command_Logs();	
		}
	};
}