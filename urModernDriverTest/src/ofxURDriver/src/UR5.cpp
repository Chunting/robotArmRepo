//
//  UR5.cpp
//  urModernDriverTest
//
//  Created by dantheman on 2/20/16.
//
//

#include "UR5.h"

UR5::UR5(){
}

UR5::~UR5(){
    
}
void UR5::setup(){
    robot = new UrDriver(rt_msg_cond_,
                         msg_cond_, "192.168.1.9");
    
    char buf[256];
    vector<string> foo = robot->getJointNames();
    std::string joint_prefix = "";
    std::vector<std::string> joint_names;
    joint_prefix = "ur5-";
    joint_names.push_back(joint_prefix + "shoulder_pan_joint");
    joint_names.push_back(joint_prefix + "shoulder_lift_joint");
    joint_names.push_back(joint_prefix + "elbow_joint");
    joint_names.push_back(joint_prefix + "wrist_1_joint");
    joint_names.push_back(joint_prefix + "wrist_2_joint");
    joint_names.push_back(joint_prefix + "wrist_3_joint");
    robot->setJointNames(joint_names);
    
    //Bounds for SetPayload service
    //Using a very conservative value as it should be set through the parameter server
    double min_payload = 0.;
    double max_payload = 1.;
    robot->setMinPayload(min_payload);
    robot->setMaxPayload(max_payload);
    sprintf(buf, "Bounds for set_payload service calls: [%f, %f]",
            min_payload, max_payload);
    ofLog()<<buf;
    
    if( robot-> start()){
        ofLog()<<"SUCCESSSS"<<endl;
    }
    
    model.setup();
}
void UR5::update(){
    std::mutex msg_lock; // The values are locked for reading in the class, so just use a dummy mutex
    std::unique_lock<std::mutex> locker(msg_lock);
    while (!robot->rt_interface_->robot_state_->getControllerUpdated()) {
        rt_msg_cond_.wait(locker);
    }
    
    //    clock_gettime(CLOCK_MONOTONIC, &current_time);
    //    elapsed_time = ofGetElapsedTimef();
    //    last_time = ofGetElapsedTimef();
    model.jointsRaw = robot->rt_interface_->robot_state_->getQActual();
    
    for(int i = 0; i < model.joints.size(); i++){
        model.jointsRaw[i] = ofRadToDeg(model.jointsRaw[i]);
        if(i == 1 || i == 3){
            model.jointsRaw[i]+=90;
        }
        model.jointsQ[i].makeRotate(model.jointsRaw[i], model.angles[i]);
        model.joints[i].setOrientation(model.jointsQ[i]);
    }
    
    
    vector<double> foo = robot->rt_interface_->robot_state_->getToolVectorActual();
    model.tool.setPosition(ofVec3f(foo[0], foo[1], foo[2])*1000);
    model.tool.setOrientation(ofVec3f(ofRadToDeg(foo[3]), ofRadToDeg(foo[4]), ofRadToDeg(foo[5])));
    
    robot->rt_interface_->robot_state_->setControllerUpdated();
    
}
void UR5::draw(){
    cam.begin(mViewPort);
    ofEnableDepthTest();
    model.draw();
    ofDisableDepthTest();
    cam.end();
    
    
    
}