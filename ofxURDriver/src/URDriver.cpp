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
    stopThread();
}

bool ofxURDriver::isDataReady(){
    if(bDataReady){
        bDataReady = false;
        return true;
    }else{
        return false;
    }
}
vector<double> ofxURDriver::getToolPointRaw(){
    lock();
    vector<double> fooR = model.toolPointRaw;
    unlock();
    return fooR;
}

vector<double> ofxURDriver::getJointPositions(){
    lock();
    vector<double> fooR = model.jointsRaw;
    unlock();
    return fooR;
}
vector<double> ofxURDriver::getJointAngles(){
    lock();
    vector<double> fooR = model.jointsRaw;
    unlock();
    return fooR;
}

float ofxURDriver::getThreadFPS(){
    float fps = 0;
    if(lock()){
        fps = timer.getFrameRate();
        unlock();
    }
    return fps;
}

void ofxURDriver::moveJoints(vector<double> pos){
    posBuffer.push_back(pos);
}

void ofxURDriver::setSpeed(vector<double> speeds){
    speedBuffers.push_back(speeds);
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
        }else{
            std::mutex msg_lock;
            std::unique_lock<std::mutex> locker(msg_lock);
            while (!robot->rt_interface_->robot_state_->getControllerUpdated()) {
                rt_msg_cond_.wait(locker);
            }
            bDataReady = true;
            model.jointsRaw = robot->rt_interface_->robot_state_->getQActual();
            model.jointsProcessed = model.jointsRaw;
            model.dtoolPoint.rotation = ofQuaternion();
            for(int i = 0; i < model.joints.size(); i++){
                model.jointsProcessed[i] = ofRadToDeg(model.jointsRaw[i]);
                if(i == 1 || i == 3){
                    model.jointsProcessed[i]+=90;
                }
                
                model.joints[i].rotation.makeRotate(model.jointsProcessed[i], model.joints[i].axis);
                model.dtoolPoint.rotation*=model.joints[i].rotation;
            }
            
            
            model.toolPointRaw = robot->rt_interface_->robot_state_->getToolVectorActual();
            float angle = sqrt(pow(model.toolPointRaw[3], 2)+pow(model.toolPointRaw[4], 2)+pow(model.toolPointRaw[5], 2));
            if( angle < epslion){
                model.tool.rotation = ofQuaternion(0, 0, 0, 0);
            }else{
                model.tool.rotation = ofQuaternion(angle, ofVec3f(model.toolPointRaw[3]/angle, model.toolPointRaw[4]/angle, model.toolPointRaw[5]/angle));
            }
            model.tool.position = ofVec3f(model.toolPointRaw[0], model.toolPointRaw[1], model.toolPointRaw[2]);

            robot->rt_interface_->robot_state_->setControllerUpdated();
            

            if(speedBuffers.size() > 0){
                robot->setSpeed(speedBuffers.front()[0], speedBuffers.front()[1], speedBuffers.front()[2], speedBuffers.front()[3], speedBuffers.front()[4], speedBuffers.front()[5]);
                speedBuffers.pop_front();
            }
        }
    }
}