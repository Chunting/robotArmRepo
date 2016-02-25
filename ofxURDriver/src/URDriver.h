//
//  ofxURDriver.hpp
//  urModernDriverTest
//
//  Created by dantheman on 2/20/16.
//
//

#pragma once
#include "ofMain.h"
#include "ur_driver.h"
#include "UR5KinematicModel.h"
#include "ofxTiming.h"
class ofxURDriver : public ofThread{
public:
    ofxURDriver();
    ~ofxURDriver();
    void setup(string ipAddress, double minPayload = 0.0, double maxPayload = 1.0);
    void start();
    void disconnect();
    void threadedFunction();
    bool isDataReady();
    float getThreadFPS();
    bool bDataReady;
    bool bStarted;
    
    // Robot Arm
    UrDriver* robot;
    std::condition_variable rt_msg_cond_;
    std::condition_variable msg_cond_;
    bool has_goal_;
    std::thread* rt_publish_thread_;
    std::thread* mb_publish_thread_;
    double io_flag_delay_;
    double max_velocity_;
    std::vector<double> joint_offsets_;
    std::string base_frame_;
    std::string tool_frame_;
    bool use_ros_control_;
    std::thread* ros_control_thread_;
    
    UR5KinematicModel model;
    RateTimer timer;
    float epslion = 0.00000000000000001;

};