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
		static bool connect_trigger;
		StatusInfo status;
		std::shared_ptr<MotomanController> controller;

		// Private constructor to prevent instantiation
		ymconnect() : controller(nullptr)
				{ controller = std::make_shared<MotomanController>();}
	public:
		// singleton instance
		static ymconnect& getInstance() { static ymconnect instance; return instance; }

		//deconstructor
		~ymconnect() { disconnect_robot(false); }

		//connect & disconnect to robot
		void connect_robot();
		void disconnect_robot(bool showmsg);
		void render();

		// setter connect trigger as true to show UI for connecting with the robot
		// static for simple access
		static void set_connect_trigger(bool trigger) { connect_trigger = trigger; }

		// command robot
		void move_robot();
		void read_robot();
	};
}


