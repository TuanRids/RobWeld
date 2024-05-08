#pragma once
#include "pch.h"
#include "YMConnect/YMConnect.h"
#include "imgui.h"
namespace nymrobot
{
	/// <summary>
	/// Singleton pattern to create only 1 instance of the class
	/// Connect the robot arm only 1 time.
	/// </summary>
	class ymconnect
	{
	private:
		bool call_move, call_read;
		bool connect_trigger = false;
		StatusInfo status;
		std::shared_ptr<MotomanController> controller;
		UINT32 restime = 10;

		// Private constructor to prevent instantiation
		ymconnect() : controller(nullptr)
		{ 
			controller = std::make_shared<MotomanController>();
			YMConnect::OpenConnection("192.0.0.0", status, restime); // Fake Login for destroy status
		}
	public:
		// singleton instance
		static ymconnect& getInstance() { static ymconnect instance; return instance; }

		//deconstructor
		~ymconnect() { disconnect_robot(false); }

		//connect & disconnect to robot
		void connect_robot();
		void moreUIRB();

		void disconnect_robot(bool showmsg);
		void render();

		// setter connect trigger as true to show UI for connecting with the robot
		// static for simple access
		void set_connect_trigger(const bool& trigger) { connect_trigger = trigger; }

		// command robot
		void move_robot();
		void read_robot();


	};
}


