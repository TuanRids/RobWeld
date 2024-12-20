#pragma once
#pragma once
#include "pch.h"  // config in pch

#include "cfreader.h"
#include "utils/RobsFileIO.h"
#include "ui/statuslogs.h"
#include "ui/FrameManage.h"
#include "mesh_import/pcltomesh.h"
#include "elems/mesh.h"
#include "ui/scene_view.h"
class IPCtransfer {
public:
    static IPCtransfer& getInstance() { static IPCtransfer instance; return instance; }
    void IPCTransferRender();
    void getter_6pos(std::vector<std::vector<float>>& get6pos);
    void status_robot(const bool& cn_status){ robot_cnstt = cn_status; }
    void trigger_run();
    void trigger_re3D(){tgre3d = true;}
	void trigger_details(){tgdetails = true;}
	void trigger_sample(){tgsample = true;}
private:
    ~IPCtransfer();
    // singleton
    IPCtransfer();

    bool UnImgFrameTrigger = false;
    bool tgre3d, tgdetails, tgsample = 0;
    GLuint image_texture;
    GLuint image_texture_below;
    cv::Mat img;
    cv::Mat img_below;
    std::map<std::string, bool> TriggerToPy;

    bool SharedMemoryTrigger = false;
    static std::vector<std::vector<float>> shared_get6pos;
    // static std::vector<std::vector<float>> shared_3Ddata;
    std::unique_ptr<std::deque<cv::Mat>> history_img;
    unsigned int stt_id = 999, coord_id = 0, chart_id = 0, img_below_id = 0;

    cv::Mat img_selected;
    nui::StatusLogs* sttlogs;
    RobInitFile* robinit;
    nelems::mMesh* promesh;
    nui::SceneView* sceneview;
    std::unique_ptr<PclToMesh> pcl2m;
    static bool robot_cnstt;
    void send_datatoIPC();
    // void trigger_3DCreator();
    void clean_image();
    void reset_TriggerToPy();
    bool receive_data();


    void Display_info();
    GLuint matToTexture(const cv::Mat& mat);
    void displayControlButtons();
    void displayMainImage();
    void displaySecondaryImage();
    void displayLaserView();

    void handleDragAndCrop(ImVec2 window_pos, float zoomFactor, cv::Mat& img_selected, GLuint& texture);
};