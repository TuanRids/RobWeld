#pragma once
#include "pch.h"
#include "YMConnect/YMConnect.h"
#include "imgui.h"
#include "elems/mesh.h"
#include <iomanip> 
#include <py3rdsrc/readpysrc.h>
#include <vector>
#include <sstream>

namespace nymrobot {

    class ymconnect {
    private:
        StatusInfo status;
        MotomanController* controller;
        UINT32 restime = 10;
        bool switchVisualizeMode = false;
        std::vector<std::vector<float>> limitangle;
        readpysrc readpysrc;
        float angle[6];
        nelems::mMesh* proMeshRb = nullptr;
        std::stringstream resultmsg;

        ymconnect();
        ~ymconnect();

        ymconnect(const ymconnect&) = delete;
        ymconnect(ymconnect&&) = delete;
        ymconnect& operator=(const ymconnect&) = delete;
        ymconnect& operator=(ymconnect&&) = delete;

    public:
        static ymconnect& getInstance() {
            static ymconnect instance;
            return instance;
        }

        void connect_robot();
        void disconnect_robot(bool showmsg);
        void render();
        void set_limitangle(const std::vector<std::vector<float>>& getlim) { limitangle = getlim; }
        int check_files_in_directory();
        void move_robot();
        void read_robot();

        bool getSwitchVisualize() const { return switchVisualizeMode; }
        void setSwitchVisualize() { switchVisualizeMode = false; }
        void get_angle(float& g1, float& g2, float& g3, float& g4, float& g5, float& g6);
    };

}
