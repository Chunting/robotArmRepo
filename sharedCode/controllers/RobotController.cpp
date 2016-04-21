#include "RobotController.h"


RobotController::RobotController(){
    
}

RobotController::~RobotController(){
    
    
}


void RobotController::setup(RobotParameters & params){
    robot.setup("192.168.1.9",0, 1);
    robot.start();
    robotParams = &params;
    movement.setup();
}

vector<double> RobotController::getJointPosition(){
    return robotParams->currentJointPos;
}

void RobotController::update(){
    toggleRecord();
    updateData();
    updateMovement();
}

void RobotController::updateMovement(){
    movement.setCurrentJointPosition(robotParams->currentJointPos);

    
   
    
    // send the target TCP to the kinematic solver
    movement.addTargetPoint(robotParams->targetTCP);
    movement.update();

    
    // set the joint speeds
    vector<double> tempSpeeds;
    tempSpeeds.assign(6, 0);
    tempSpeeds = movement.getCurrentSpeed();

    // move the robot to the target TCP
    if(robotParams->bMove){
        robot.setSpeed(tempSpeeds, movement.getAcceleration());
    }
    
    for(int i = 0; i < tempSpeeds.size(); i++){
        robotParams->jointVelocities[i] = (float)tempSpeeds[i];
    }
    
}

void RobotController::updateData(){
    // pass the current joints from the robot to the kinematic solver
    robotParams->currentJointPos = robot.getJointPositions();

    for(int i = 0; i < robotParams->currentJointPos.size(); i++){
        robotParams->jointPos[i] = (float)robotParams->currentJointPos[i];
    }
    robotParams->actualTCP = robot.getToolPose();
    robotParams->tcpPosition = robotParams->actualTCP.position;
    ofQuaternion tcpO = robotParams->actualTCP.rotation;
    robotParams->tcpOrientation = ofVec4f(tcpO.x(), tcpO.y(), tcpO.z(), tcpO.w());
    if(robotParams->bRecord){
        recorder.addPose(robotParams->currentJointPos, ofGetElapsedTimef());
    }
    // update GUI params
    for(int i = 0; i < robotParams->currentJointPos.size(); i++){
        robotParams->jointPos[i] = ofRadToDeg((float)robotParams->currentJointPos[i]);
    }
    
}

void RobotController::toggleRecord(){
    if(robotParams->bRecord){
        recorder.startRecording();
    }else{
        recorder.endRecording();
    }
}

void RobotController::draw(){
    
}