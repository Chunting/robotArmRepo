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
    // update GUI params
    for(int i = 0; i < robotParams->currentJointPos.size(); i++){
        robotParams->jointPos[i] = ofRadToDeg((float)robotParams->currentJointPos[i]);
    }
    
    moveArm();
    
    robotParams->targetTCP.position+=robotParams->tcpOffset;
    
    // send the target TCP to the kinematic solver
    movement.addTargetPoint(robotParams->targetTCP);
    movement.update();
    vector<double> rawIK = movement.getRawJointPos();
    for(int i = 0; i < rawIK.size(); i++){
        robotParams->jointPosIKRaw[i] = ofRadToDeg((float)rawIK[i]);
    }
    
    // get back the target joint trajectories
    vector<double> target = movement.getTargetJointPos();
    for(int i = 0; i < target.size(); i++){
        robotParams->targetJointPos[i] = ofRadToDeg((float)target[i]);
    }
    
    // set the joint speeds
    vector<double> tempSpeeds;
    tempSpeeds.assign(6, 0);
    tempSpeeds = movement.getCurrentSpeed();
    for(int i = 0; i < tempSpeeds.size(); i++){
        robotParams->jointVelocities[i] = (float)tempSpeeds[i];
    }
    // move the robot to the target TCP
    if(robotParams->bMove){
        robot.setSpeed(tempSpeeds, movement.getAcceleration());
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
}

void RobotController::updatePose(Joint targetTCP){
    workSurfaceTargetTCP = targetTCP;
}

void RobotController::moveArm(){
    
    // assign the target pose to the current robot pose
    if(robotParams->bCopy){
        robotParams->bCopy = false;
        
        // get the robot's position
        robotParams->targetTCP = robotParams->actualTCP;
        
        
        // update GUI params
        robotParams->targetTCPPosition = robotParams->targetTCP.position;
        robotParams->targetTCPOrientation = ofVec4f(robotParams->targetTCP.rotation.x(), robotParams->targetTCP.rotation.y(), robotParams->targetTCP.rotation.z(), robotParams->targetTCP.rotation.w());
        
    }
    // follow a user-defined position and orientation
    if(robotParams->bFollow){
        

        robotParams->targetTCP.position.interpolate(robotParams->targetTCPPosition.get(), movement.followLerp);
        robotParams->targetTCP.rotation = ofQuaternion(robotParams->targetTCPOrientation);
        //        }
        
        // update GUI params
        robotParams->bTrace = false;
        robotParams->targetTCPPosition = robotParams->targetTCP.position;
        
    }else if(robotParams->bTrace || robotParams->b3DPath){
        // set target TCP to a default orientation, then modify
        robotParams->targetTCP.rotation = ofQuaternion(90, ofVec3f(0, 0, 1));
        robotParams->targetTCP.rotation*=ofQuaternion(90, ofVec3f(1, 0, 0));
        robotParams->targetTCP.rotation*=ofQuaternion(0, ofVec3f(0,1, 0));
        
        robotParams->targetTCP.position = workSurfaceTargetTCP.position;
        robotParams->targetTCP.rotation *= workSurfaceTargetTCP.rotation;
        
        // update GUI params
        robotParams->targetTCPPosition = robotParams->targetTCP.position;
        robotParams->targetTCPOrientation = ofVec4f(robotParams->targetTCP.rotation.x(), robotParams->targetTCP.rotation.y(), robotParams->targetTCP.rotation.z(), robotParams->targetTCP.rotation.w());
        
    }else if(robotParams->bFigure8){
        
        // use a preset orientation
        robotParams->targetTCP.rotation = ofQuaternion(90, ofVec3f(0, 0, 1));
        robotParams->targetTCP.rotation*=ofQuaternion(90, ofVec3f(1, 0, 0));
        
        // update the target position
        robotParams->targetTCP.position.interpolate(robotParams->targetTCPPosition.get()+ofVec3f(cos(ofGetElapsedTimef()*0.25)*0.2, 0, sin(ofGetElapsedTimef()*0.25*2)*0.2), 0.5);
        
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