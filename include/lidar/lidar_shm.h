#ifndef __LIDER_SHM_H__
#define __LIDER_SHM_H__

// data: 2020.12.21
// Version: 1.4.5

#ifdef __cplusplus
extern "C" {
#endif

struct SlamOutput {
    float i;
    float j;
    float forward;
    float valid_value;
    
    float forward_occ_dist;
    float left_occ_dist;
    float right_occ_dist;
    
    float status;
    
    float brp_x;
    float brp_y;
    float brp_yaw;
    
    float laser_state;
    // float imu_state;
    
    // float no_use1;
    // float no_use2;
    // float no_use3;
    // float no_use4;
    // float no_use5;
    // float no_use6;
    // float no_use7;
    // float no_use8;
    // float no_use9;
    // float no_use10;
    // float no_use11;
};

struct GridPose {
    float i;
    float j;
    float forward;
};

struct Pose2D {
	float x;
	float y;
	float yaw;
};

void InitReader();
struct SlamOutput GetSlamOutput();

int isValid();
int GetGridStatus(int i, int j);
struct GridPose GetCurGridPose();
float GetForwardOccDist();



float GetLeftOccDist();
float GetRightOccDist();
float GetLocStatus();
struct Pose2D GetPoseBefRelocalization();

int GetLaserState();
int GetImuState();

#ifdef __cplusplus
}
#endif
#endif
