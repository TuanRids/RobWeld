#pragma once
#include "pch.h"
#include "YMConnect/YMConnect.h"
#include "imgui.h"
#include "elems/mesh.h"
#include <iomanip> 
namespace nymrobot
{
	/// <summary>
	/// Singleton pattern to create only 1 instance of the class
	/// Connect the robot arm only 1 time.
	/// </summary>
	class ymconnect
	{
	private:
		bool call_move{ false }, call_read{ false };
		bool connect_trigger = true;
		StatusInfo status;
		MotomanController* controller;
		UINT32 restime = 10;
		
		// Singleton pattern to prevent instantiation
		ymconnect(const ymconnect&) = delete;
		ymconnect(ymconnect&&) = delete;
		ymconnect& operator=(const ymconnect&) = delete;
		ymconnect& operator=(ymconnect&&) = delete;
		float angle1{ 0 }, angle2{ 0 }, angle3{ 0 }, angle4{ 0 }, angle5{ 0 }, angle6{ 0 };
		// pointer to Robot's mesh
		nelems::mMesh* proMeshRb;
		std::stringstream resultmsg;
		// based ptr
		// Private constructor to prevent instantiation
		ymconnect() : controller(nullptr), proMeshRb(nullptr)
		{ 
			YMConnect::OpenConnection("192.0.0.0", status, restime); // Fake Login for destroy status
		}
	public:
		// singleton instance
		static ymconnect& getInstance() { 
			static ymconnect instance; return instance; }

		//deconstructor
		~ymconnect() 
		{ 
			if (status.StatusCode  == 0)
			{
				disconnect_robot(false);
			}
			delete controller;
		}

		//connect & disconnect to robot
		void connect_robot();

		void disconnect_robot(bool showmsg);
		void render();

		// setter connect trigger as true to show UI for connecting with the robot
		// static for simple access
		void set_connect_trigger(const bool& trigger) { connect_trigger = trigger; }

		// command robot
		void move_robot();
		void read_robot();

		void get_angle(float& g1,float &g2,float &g3,float &g4,float &g5,float &g6) {
			if (status.StatusCode != 0){ return; }
			if (status.StatusCode == 0)
			{
				g1 = angle1;
				g2 = angle2;
				g3 = angle3;
				g4 = angle4;
				g5 = angle5;
				g6 = angle6;
			}
		}

	};
}


