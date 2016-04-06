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
    
    // send the target TCP to the kinematic solver
    movement.addTargetPoint(robotParams->targetTCP);
    movement.update();
    
    
    // get back the target joint trajectories
    vector<double> target = movement.getTargetJointPos();
    for(int i = 0; i < target.size(); i++){
        robotParams->targetJointPos[i] = (float)target[i];
    }
    
    // set the joint speeds
    vector<double> tempSpeeds;
    tempSpeeds.assign(6, 0);
    tempSpeeds = movement.getCurrentSpeed();
    for(int i = 0; i < tempSpeeds.size(); i++){
        robotParams->jointVelocities[i] = (float)tempSpeeds[i];
    }
    // move the robot to the target TCP
    robotParams->avgAccel = movement.getAcceleration();
    if(robotParams->bMove){
        robot.setSpeed(tempSpeeds, robotParams->avgAccel);
    }
    
}

void RobotController::updateData(){
    // pass the current joints from the robot to the kinematic solver
    robotParams->currentJointPos = robot.getJointPositions();
    
    
    // update GUI params
    for(int i = 0; i < robotParams->currentJointPos.size(); i++){
        robotParams->jointPos[i] = (float)robotParams->currentJointPos[i];
    }
    robotParams->tcpPosition = robot.getToolPoint();
}

void RobotController::updatePath(Joint targetTCP){
    workSurfaceTargetTCP = targetTCP;
}

void RobotController::moveArm(){
    // set target TCP to a default orientation, then modify
    robotParams->targetTCP.rotation = ofQuaternion(90, ofVec3f(0, 0, 1));
    robotParams->targetTCP.rotation*=ofQuaternion(90, ofVec3f(1, 0, 0));
    robotParams->targetTCP.rotation*=ofQuaternion(0, ofVec3f(0,1, 0));
    
    // assign the target pose to the current robot pose
    if(robotParams->bCopy){
        robotParams->bCopy = false;
        
        // get the robot's position
        robotParams->targetTCP.position = robot.getToolPoint();
        // get the robot's orientation
        // targetTCP.rotation = .... <-- why is this working without grabbing the current orientation?
        
        // update GUI params
        robotParams->targetTCPPosition = robotParams->targetTCP.position;
        robotParams->targetTCPOrientation = ofVec4f(robotParams->targetTCP.rotation.x(), robotParams->targetTCP.rotation.y(), robotParams->targetTCP.rotation.z(), robotParams->targetTCP.rotation.w());
        
    }
    // follow a user-defined position and orientation
    if(robotParams->bFollow){
        
        //        // follow mocap rigid body
        //        if (natNet.recordedPath.size() > 1){
        //
        //            //            auto &rb = recordedPath[0];
        //            //            targetTCP.position = rb.matrix.getTranslation()/1000;
        //            //            targetTCP.rotation = rb.matrix.getRotate();
        //
        //            updateWorksurface(natNet.getCurrentRigidBody());
        //
        //        }else{
        //            // go from current to next position
        robotParams->targetTCP.position.interpolate(robotParams->targetTCPPosition.get(), 0.1);
        //            // go from current orientation to next orientation (???)
        robotParams->targetTCP.rotation = ofQuaternion(robotParams->targetTCPOrientation);
        //        }
        
        // update GUI params
        robotParams->targetTCPPosition = robotParams->targetTCP.position;
        robotParams->tcpOrientation =robotParams->targetTCP.rotation.getEuler();
    }
    // follow a pre-defined path
    if(robotParams->bTrace){
        
        
        robotParams->targetTCP.position = workSurfaceTargetTCP.position;
        robotParams->targetTCP.rotation *= workSurfaceTargetTCP.rotation;
        
        // update GUI params
        robotParams->targetTCPPosition = robotParams->targetTCP.position;
        robotParams->targetTCPOrientation = ofVec4f(robotParams->targetTCP.rotation.x(), robotParams->targetTCP.rotation.y(), robotParams->targetTCP.rotation.z(), robotParams->targetTCP.rotation.w());
        
    }
    // draw out a figure 8 in mid-air
    if(robotParams->bFigure8){
        
        // use a preset orientation
        robotParams->targetTCP.rotation = ofQuaternion(90, ofVec3f(0, 0, 1));
        robotParams->targetTCP.rotation*=ofQuaternion(90, ofVec3f(1, 0, 0));
        
        // update the target position
        robotParams->targetTCP.position.interpolate(robotParams->targetTCPPosition.get()+ofVec3f(cos(ofGetElapsedTimef()*0.25)*0.2, 0, sin(ofGetElapsedTimef()*0.25*2)*0.2), 0.5);
        
        // update GUI params
        robotParams->tcpOrientation = robotParams->targetTCP.rotation.getEuler();
    }
    
}

void RobotController::draw(){
    
}