//
//  RobotParameters.h
//  urModernDriverTest
//
//  Created by dantheman on 4/4/16.
//
//

#pragma once
#include "ofMain.h"
class RobotParameters{
public :
    void setup(){
        robotArmParams.add(targetTCPPosition.set("Set TCP POS", ofVec3f(0, 0, 0), ofVec3f(-1, -1, -1), ofVec3f(1, 1, 1)));
        robotArmParams.add(targetTCPOrientation.set("Set TCP ORIENT",ofVec4f(0,0,0,1), ofVec4f(-1,-1,-1,-1), ofVec4f(1,1,1,1)));
        robotArmParams.add(tcpPosition.set("Robot TCP POS", ofVec3f(0, 0, 0), ofVec3f(-1, -1, -1), ofVec3f(1, 1, 1)));
        robotArmParams.add(tcpOrientation.set("Robot TCP ORIENT", ofVec3f(0, 0, 0), ofVec3f(-TWO_PI, -TWO_PI, -TWO_PI), ofVec3f(TWO_PI, TWO_PI, TWO_PI)));
        
        robotArmParams.add(bFollow.set("set TCP", false));
        robotArmParams.add(bTrace.set("bTrace GML", false));
        robotArmParams.add(bCopy.set("get TCP", false));
        
        joints.setName("Joints");
        joints.add(bMove.set("Move", false));
        joints.add(avgAccel.set("avgAccel", 0, 0, 200));
        joints.add(bFigure8.set("bFigure8", false));
        for(int i = 0; i < 6; i++){
            jointPos.push_back(ofParameter<float>());
            targetJointPos.push_back(ofParameter<float>());
            jointVelocities.push_back(ofParameter<float>());
            joints.add(jointPos.back().set("joint "+ofToString(i), 0, -TWO_PI, TWO_PI));
            joints.add(targetJointPos.back().set("target joint "+ofToString(i), 0, -TWO_PI, TWO_PI));
            joints.add(jointVelocities.back().set("Joint Speed"+ofToString(i), 0, -100, 100));
        }


    };
    ofParameterGroup robotArmParams;
    ofParameter<ofVec3f> targetTCPPosition;
    ofParameter<ofVec4f> targetTCPOrientation;
    ofParameter<ofVec3f> tcpOrientation;
    ofParameter<ofVec3f> tcpPosition;
    
    ofParameterGroup joints;
    ofParameter<float> avgAccel;
    vector<ofParameter<float> > jointPos;
    vector<ofParameter<float> > targetJointPos;
    vector<ofParameter<float> > jointVelocities;
    
    ofParameter<bool> bMove;
    ofParameter<bool> bFigure8;
    ofParameter<bool> bTrace;
    ofParameter<bool> bFollow;
    ofParameter<bool> bCopy;
    ofParameter<bool> bStop;
    

};

