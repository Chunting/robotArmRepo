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
class RobotController{
public:
    RobotController();
    ~RobotController();
    
    void setup();
    void update();
    void draw();
    RobotStateMachine state;
};