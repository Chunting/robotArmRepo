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
class RobotController{
public:
    RobotController();
    ~RobotController();
    
    void setup(RobotParameters & params);
    void updateMovement();
    void updateData();
    void update(Joint targetPt);
    void moveArm();
    void draw();
    
    
    void updatePath(Joint targetPt);
    
    
    vector<double> getJointPosition();
    RobotStateMachine state;
    ofxURDriver robot;
    URMove movement;
    RobotParameters * robotParams;
    
    Joint workSurfaceTargetTCP;
};