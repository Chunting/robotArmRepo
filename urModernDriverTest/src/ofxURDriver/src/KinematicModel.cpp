//
//  KinectModel.cpp
//  urModernDriverTest
//
//  Created by dantheman on 2/20/16.
//
//

#include "KinematicModel.h"
KinematicModel::KinematicModel(){
    
}
KinematicModel::~KinematicModel(){
    
}
void KinematicModel::setup(){
    
    for(int i = 0; i < 6; i++){
        angles.push_back(ofVec3f());
        jointLength.push_back(ofVec3f());
        jointsQ.push_back(ofQuaternion());
        joints.push_back(ofNode());
        
    }
    
    joints[1].setParent(joints[0]);
    joints[2].setParent(joints[1]);
    joints[3].setParent(joints[2]);
    joints[4].setParent(joints[3]);
    joints[5].setParent(joints[4]);
    
    joints[0].setPosition(0, 0, 0);
    joints[1].setPosition(ofVec3f(0, -109.3, 89.2));
    joints[2].setPosition(ofVec3f(0, 0, 425));
    joints[3].setPosition(ofVec3f(0, 109.3, 392));
    joints[4].setPosition(ofVec3f(0, -109.3, 0));
    joints[5].setPosition(ofVec3f(0, -82.5, 94.75));
    
    
    
    angles[0].set(0, 0, 1);
    angles[1].set(1, 0, 0);
    angles[2].set(1, 0, 0);
    angles[3].set(1, 0, 0);
    angles[4].set(0, 0, 1);
    angles[5].set(0, 1, 0);
    
    jointLength[0].set(0, 0, 0);
    jointLength[1].set(10, 0, 0);
    jointLength[2].set(425, 0, 0);
    jointLength[3].set(392, 0, 0);
    jointLength[4].set(0, 109, 0);
    jointLength[5].set(0, 82, 0);
}
void KinematicModel::update(){
    
}
void KinematicModel::draw(){
    ofSetColor(255, 255, 255);
    tool.draw();
    joints[0].draw();
    for(int i = 1; i < joints.size(); i++){
        if(i == 0){
            ofSetColor(255, 0, 0);
        }
        if(i == 1){
            ofSetColor(0, 255, 255);
        }
        if(i == 2){
            ofSetColor(0, 0, 255);
        }
        if(i == 3){
            ofSetColor(0, 255, 0);
        }
        if(i == 4){
            ofSetColor(255, 255, 0);
        }
        if(i == 5){
            ofSetColor(255, 0, 255);
        }
        ofDrawLine(joints[i].getGlobalPosition(), joints[i-1].getGlobalPosition());
        joints[i].draw();
    }
}