#pragma once


#include "elems/mesh.h"
#include "Command/ObHistory.h"
#include "utils/RobsFileIO.h"
#include <random> // Include the random library
#include <glm/glm.hpp> // Include the glm library for vec3

namespace nui
{
	/*
	HAVETODO: Make render to jgl_window.cpp to for clearly management
	Manage UI Actions. set ObHistory at jgl_window.cpp
	 */
	class uiAction
	{
	private:
		nelems::mMesh* proMesh;
		std::shared_ptr<nelems::oMesh> mesh;
		ncommand::ObHistory* obAction;
		nutils::RobsFileIO robFileIO;
	public:
		uiAction(): proMesh(nullptr), obAction(nullptr), robFileIO(), mesh(nullptr){
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

			ImGui::Begin(name, NULL, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoMove |  ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav);
			ImGui::Text("X"); ImGui::SameLine(); ImGui::InputFloat("X", &x);
			ImGui::Text("Y"); ImGui::SameLine(); ImGui::InputFloat("Y", &y);
			ImGui::Text("Z"); ImGui::SameLine(); ImGui::InputFloat("Z", &z);
			if (ImGui::Button("OK")) {
				pressOk = true;
				ImGui::CloseCurrentPopup();
			}
			ImGui::End(); 
		}
		// ********** MoveObject **********
		void MoveOb_uiAction(float &x, float &y, float &z)
		{
			if (obAction == nullptr || proMesh->check_selected() < 1) { return; }
			std::unique_ptr<ncommand::Command> moveCmd = std::make_unique<ncommand::MoveOb>(proMesh, x, y, z);
			obAction->execmd(std::move(moveCmd));
		}
		void MoveOb_uiAction(bool &waitloop)
		{
			if (obAction == nullptr || proMesh->check_selected() < 1) { waitloop = false; return; }
			static float mx = 0 , my = 0, mz = 0;static bool pressOk = false;
			getXYZ("Move Object", pressOk, mx,my,mz);

			std::unique_ptr<ncommand::Command> moveCmd = std::make_unique<ncommand::MoveOb>(proMesh, mx,my,mz);
			obAction->execmd(std::move(moveCmd)); pressOk = false;
			waitloop = false; mx = my = mz = 0;
		}
		// ********** RotateObject **********
		void RotateOb_uiAction(float& x, float& y, float& z)
		{
			if (obAction == nullptr || proMesh->check_selected() < 1) { return; }
			std::unique_ptr<ncommand::Command> moveCmd = std::make_unique<ncommand::RotateOb>(proMesh, x, y, z);
			obAction->execmd(std::move(moveCmd));
		}
		void RotateOb_uiAction(bool& waitloop)
		{
			if (obAction == nullptr || proMesh->check_selected() < 1) { waitloop = false; return; }
			static float rx = 0, ry = 0, rz = 0; static bool pressOk = false;
			getXYZ("Rotate Object", pressOk, rx, ry, rz);

			std::unique_ptr<ncommand::Command> moveCmd = std::make_unique<ncommand::RotateOb>(proMesh,rx,ry,rz);
			obAction->execmd(std::move(moveCmd)); pressOk = false;
			waitloop = false; rx = ry = rz = 0;
		}
		void Random_Color()
		{
			if (obAction == nullptr || proMesh->check_selected() < 1) { return; }

			// Create a random device and a Mersenne Twister random engine
			std::random_device rd;
			std::mt19937 gen(rd());
			std::uniform_real_distribution<float> dis(0.3f, 0.7f); // Uniform distribution between 0 and 1

			for (auto it = proMesh->getMesh()->begin(); it != proMesh->getMesh()->end(); it++)
			{
				auto mesh = *it;
				if (mesh->selected)
				{
					// Set oMaterial.mColor to a random color using the uniform distribution
					mesh->oMaterial.mColor = glm::vec3(dis(gen), dis(gen), dis(gen));
					mesh->create_buffers();
				}
			}
		}

		// ********** Del selected Object **********
		void Del_selected_objects()
		{
			std::unique_ptr<ncommand::Command> moveCmd = std::make_unique<ncommand::delOb>(proMesh);
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
		void SaveToFile() {
			if (proMesh->size() == 0) {std::cout << "Error: Mesh not loaded" << std::endl;}
			else { robFileIO.SaveToFile(*proMesh);}}
		void LoadFromFile() { robFileIO.LoadFromFile(*proMesh);}

	};
}