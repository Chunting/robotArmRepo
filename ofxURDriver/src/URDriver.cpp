//
//  ofxURDriver.cpp
//  urModernDriverTest
//
//  Created by dantheman on 2/20/16.
//
//

#include "URDriver.h"

ofxURDriver::ofxURDriver(){
}

ofxURDriver::~ofxURDriver(){
    
}
void ofxURDriver::setup(string ipAddress, double minPayload, double maxPayload){
    robot = new UrDriver(rt_msg_cond_,
                         msg_cond_, ipAddress);
    
    char buf[256];
    vector<string> foo = robot->getJointNames();
    std::string joint_prefix = "";
    std::vector<std::string> joint_names;
    joint_prefix = "ofxURDriver-";
    joint_names.push_back(joint_prefix + "shoulder_pan_joint");
    joint_names.push_back(joint_prefix + "shoulder_lift_joint");
    joint_names.push_back(joint_prefix + "elbow_joint");
    joint_names.push_back(joint_prefix + "wrist_1_joint");
    joint_names.push_back(joint_prefix + "wrist_2_joint");
    joint_names.push_back(joint_prefix + "wrist_3_joint");
    robot->setJointNames(joint_names);
    
    //Bounds for SetPayload service
    //Using a very conservative value as it should be set through the parameter server
    double min_payload = minPayload;
    double max_payload = maxPayload;
    robot->setMinPayload(min_payload);
    robot->setMaxPayload(max_payload);
    sprintf(buf, "Bounds for set_payload service calls: [%f, %f]",
            min_payload, max_payload);
    ofLog(OF_LOG_NOTICE)<<buf;
    model.setup();
    bStarted = false;
}
void ofxURDriver::start(){
    ofLog(OF_LOG_NOTICE)<<"Starting ofxURDriver Controller"<<endl;
    startThread();
}

void ofxURDriver::disconnect(){
    robot->halt();
//    stopThread();
}

bool ofxURDriver::isDataReady(){
    if(bDataReady){
        bDataReady = false;
        return true;
    }else{
        return false;
    }
}

float ofxURDriver::getThreadFPS(){
    float fps = 0;
    if(lock()){
        fps = timer.getFrameRate();
        unlock();
    }
    return fps;
}

void ofxURDriver::threadedFunction(){
    while(isThreadRunning()){
        timer.tick();
        if(!bStarted){
            bStarted = robot->start();
            if(bStarted){
                ofLog(OF_LOG_NOTICE)<<"Robot Started"<<endl;
            }else{
                ofLog(OF_LOG_ERROR)<<"Rboto Not Started"<<endl;
                stopThread();
            }
        }
        std::mutex msg_lock;
        std::unique_lock<std::mutex> locker(msg_lock);
        while (!robot->rt_interface_->robot_state_->getControllerUpdated()) {
            rt_msg_cond_.wait(locker);
        }
        bDataReady = true;
        model.jointsRaw = robot->rt_interface_->robot_state_->getQActual();
        
        for(int i = 0; i < model.jointsNode.size(); i++){
            model.jointsRaw[i] = ofRadToDeg(model.jointsRaw[i]);
            if(i == 1 || i == 3){
                model.jointsRaw[i]+=90;
            }
            
            model.jointsQ[i].makeRotate(model.jointsRaw[i], model.angles[i]);
            model.jointsNode[i].setOrientation(model.jointsQ[i]);
        }
        
        
        vector<double> foo = robot->rt_interface_->robot_state_->getToolVectorActual();
        float angle = sqrt(pow(foo[3], 2)+pow(foo[4], 2)+pow(foo[5], 2));
        if( angle < epslion){
            model.tool.setOrientation(ofQuaternion(0, 0, 0, 1));
        }else{
            model.tool.setOrientation(ofQuaternion(angle, ofVec3f(foo[3]/angle, foo[4]/angle, foo[5]/angle)));
        }
        model.tool.setPosition(ofVec3f(foo[0], foo[1], foo[2])*1000);
        
        foo = robot->rt_interface_->robot_state_->getToolVectorTarget();
        
        angle = sqrt(pow(foo[3], 2)+pow(foo[4], 2)+pow(foo[5], 2));
        if( angle < epslion){
            model.targetPoint.setOrientation(ofQuaternion(0, 0, 0, 1));
        }else{
            model.targetPoint.setOrientation(ofQuaternion(angle, ofVec3f(foo[3]/angle, foo[4]/angle, foo[5]/angle)));
        }
        model.targetPoint.setPosition(ofVec3f(foo[0], foo[1], foo[2])*1000);

        
        model.jointsTargetRaw = robot->rt_interface_->robot_state_->getQTarget();
        for(int i = 0; i < model.jointsTargetNode.size(); i++){
            model.jointsTargetRaw[i] = ofRadToDeg(model.jointsTargetRaw[i]);
            if(i == 1 || i == 3){
                model.jointsTargetRaw[i]+=90;
            }
            
            model.jointTargetQ[i].makeRotate(model.jointsTargetRaw[i], model.angles[i]);
            model.jointsTargetNode[i].setOrientation(model.jointTargetQ[i]);
        }
        
        robot->rt_interface_->robot_state_->setControllerUpdated();
    }
}