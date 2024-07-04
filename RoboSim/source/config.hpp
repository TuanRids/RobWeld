// config.hpp
#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <string>

namespace Config {
    // == path to data ===
    // robot model
	constexpr const char* PATH_TO_ROBOTEMPLATE =  "RobotStandard//Yaskawa-gp8-113k.fbx";
    // 3D data from Hung
    static constexpr const char* PATH_TO_3D_HUNG =  "check_pcl//datascan_processed.txt";
	// 3D data from Sang
    constexpr const char* PATH_TO_3D_Sang =  "check_pcl//dataclean.txt";

    // == IMPORTANT NAME & ADDRESS ===
    // Address to Inte-Process Communication with Vision 
    constexpr const char* IPC_GET_IMG = "Local\\ImgPytoCPP";
    // Address to Inte-Process Communication with Vision 
    constexpr const char* IPC_GET_IMG_BELOW = "Local\\ImgPytoCPPBelow";
    // Address to Inte-Process Communication with Vision 
    constexpr const char* IPC_GET_STATUS = "Local\\StatusPytoCPP";
    // Address to Inte-Process Communication with Vision 
	constexpr const char* IPC_GET_DATA = "Local\\floatPytoCPP";
    // Get robot Position from Robot 
    constexpr const char* IPC_GET_POS = "Local\\PosPytoCPP";
    // Get adjusted 3D coordinator
    constexpr const char* IPC_GET_COOR3DMESH = "Local\\Coor3DMeshtoCPP";

    // Send mapping data with Vision 
	constexpr const char* IPC_SEND_MAPPING = "Local\\MapCPPtoDictPy";
    
    

}

#endif // CONFIG_HPP
