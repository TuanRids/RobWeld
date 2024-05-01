#pragma once


#include "elems/mesh.h"
#include "Command/ObHistory.h"
#include "utils/RobsFileIO.h"

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
		nutils::RobsFileIO robFileIO;
	public:
		uiAction(): proMesh(nullptr), obAction(nullptr), robFileIO() {
			if (!proMesh) {proMesh = &nelems::mMesh::getInstance();
			obAction = &ncommand::ObHistory::getInstance();
			}
		}
		~uiAction() { proMesh = nullptr; obAction = nullptr; }
		// 
		void getXYZ(const char* name,bool &pressOk, float& x, float& y, float& z)
		{
			ImGui::SetNextWindowSize(ImVec2(200, 200));
			ImGui::SetNextWindowPos(ImVec2(300, 300), ImGuiCond_Always);

			ImGui::Begin(name, NULL, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav);
			ImGui::Text("X"); ImGui::SameLine(); ImGui::InputFloat("X", &x);
			ImGui::Text("Y"); ImGui::SameLine(); ImGui::InputFloat("Y", &y);
			ImGui::Text("Z"); ImGui::SameLine(); ImGui::InputFloat("Z", &z);
			if (ImGui::Button("OK")) {
				pressOk = true;
				ImGui::CloseCurrentPopup();
			}
			ImGui::End(); 
		}

		void MoveOb_uiAction(bool &waitloop)
		{
			if (obAction == nullptr || proMesh->check_selected() < 1) { waitloop = false; return; }
			static float x = 0 , y = 0, z = 0;static bool pressOk = false;
			getXYZ("Move Object", pressOk, x,y,z);
			
			if ((x != 0.0f || y != 0.0f || z != 0.0f) && pressOk)
			{
				std::unique_ptr<ncommand::Command> moveCmd = std::make_unique<ncommand::MoveOb>(proMesh, x,y,z);
				obAction->execmd(std::move(moveCmd)); pressOk = false;
				waitloop = false;x=y=z=0;
			}
		}
		void RotateOb_uiAction()
		{
		if (obAction == nullptr) return;
		std::unique_ptr<ncommand::Command> moveCmd = std::make_unique<ncommand::RotateOb>(proMesh,0.0f,0.0f,0.0f);
		obAction->execmd(std::move(moveCmd));
		}
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
		void SaveToFile() {
			if (proMesh->size() == 0) {std::cout << "Error: Mesh not loaded" << std::endl;}
			else { robFileIO.SaveToFile(*proMesh);}}
		void LoadFromFile() { robFileIO.LoadFromFile(*proMesh);}

	};
}