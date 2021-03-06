//
//  RobotController.h
//  urModernDriverTest
//
//  Created by dantheman on 4/4/16.
//
//
#pragma once
#include "ofMain.h"
#include "RobotStateMachine.h"
#include "ofxURDriver.h"
#include "RobotParameters.h"
#include "PathRecorder.h"
class RobotController{
public:
    RobotController();
    ~RobotController();
    
    void setup(RobotParameters & params);
    void updateMovement();
    void updateData();
    void update();
    void moveArm();
    void draw();
    void toggleRecord();
    void close();
    ofNode getTCPNode();
    vector<double> getJointPosition();
    RobotStateMachine state;
    ofxURDriver robot;
    URMove movement;
    RobotParameters * robotParams;
    
    PathRecorder recorder;
    
    
    
    Joint workSurfaceTargetTCP;
};