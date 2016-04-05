#include "RobotController.h"


RobotController::RobotController(){
    
}

RobotController::~RobotController(){
    
    
}


void RobotController::setup(RobotParameters & params){
    robot.setup("192.168.1.9",0, 1);
    robot.start();
    robotParams = &params;
}

vector<double> RobotController::getJointPosition(){
    return robotParams->currentJointPos;
}

void RobotController::update(){
    // pass the current joints from the robot to the kinematic solver
     robotParams->currentJointPos = robot.getJointPositions();
  
    
    // update GUI params
    for(int i = 0; i < robotParams->currentJointPos.size(); i++){
        robotParams->jointPos[i] = (float)robotParams->currentJointPos[i];
    }
    robotParams->tcpPosition = robot.getToolPoint();
    
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
//            targetTCP.position.interpolate(robotParams->targetTCPPosition.get(), 0.1);
//            // go from current orientation to next orientation (???)
//            targetTCP.rotation *= ofQuaternion(robotParams->targetTCPOrientation);
//        }
        
        // update GUI params
        robotParams->targetTCPPosition = robotParams->targetTCP.position;
        robotParams->tcpOrientation =robotParams->targetTCP.rotation.getEuler();
    }
    // follow a pre-defined path
    if(robotParams->bTrace){
        
//        // update the worksurface
//        workSurface.update();
//        
//        // get the target point on the worksurface
//        Joint workSrfTarget = workSurface.getTargetPoint(ofGetElapsedTimef()-tagStartTime);
//        robotParams->targetTCP.position = workSrfTarget.position;
//        robotParams->targetTCP.rotation *= workSrfTarget.rotation;
//        
//        // update GUI params
//        robotParams->targetTCPPosition = robotParams->targetTCP.position;
//        robotParams->targetTCPOrientation = ofVec4f(robotParams->targetTCP.rotation.x(), robotParams->targetTCP.rotation.y(), robotParams->targetTCP.rotation.z(), robotParams->targetTCP.rotation.w());
        
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