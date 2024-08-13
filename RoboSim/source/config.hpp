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
    constexpr const char* IPC_GET_IMG = "Local\\ImgAbove";
    // Address to Inte-Process Communication with Vision 
    constexpr const char* IPC_GET_IMG_BELOW = "Local\\ImgBelow";
    // Address to Inte-Process Communication with Vision 
    constexpr const char* IPC_GET_STATUS = "Local\\VisionStatus";
    // Get robot Position from Robot 
    constexpr const char* IPC_GET_POS = "Local\\Robpos";
    // Get adjusted 3D coordinator
    constexpr const char* IPC_GET_COOR3DMESH = "Local\\Coor3DMesh";
    // Get 3D chart 
    constexpr const char* IPC_GET_CHART = "Local\\ChartInspection";

    // Send mapping data with Vision 
	constexpr const char* IPC_SEND_TRIGGER = "Local\\TriggerVision";
    
    

}

#endif // CONFIG_HPP
