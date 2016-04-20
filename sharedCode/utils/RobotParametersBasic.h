//
//  RobotParametersBasic.h
//  example-move-basic
//
//  Created by mad on 4/20/16.
//
//


#pragma once
#include "ofMain.h"
#include "RobotParameters.h"
class RobotParametersBasic : public RobotParameters{
    public :
    void setup(){
        
        robotArmParams.add(tcpPosition.set("Actual Robot TCP POS", ofVec3f(0, 0, 0), ofVec3f(-1, -1, -1), ofVec3f(1, 1, 1)));
        robotArmParams.add(tcpOrientation.set("Actual Robot TCP ORIENT", ofVec3f(0, 0, 0), ofVec3f(-TWO_PI, -TWO_PI, -TWO_PI), ofVec3f(TWO_PI, TWO_PI, TWO_PI)));
        robotArmParams.add(targetTCPPosition.set("Set TCP POS", ofVec3f(0, 0, 0), ofVec3f(-1, -1, -1), ofVec3f(1, 1, 1)));
        robotArmParams.add(targetTCPOrientation.set("Set TCP ORIENT",ofVec4f(0,0,0,1), ofVec4f(-1,-1,-1,-1), ofVec4f(1,1,1,1)));
        
//        robotArmParams.add(tcpOffset.set("tcpOffset", ofVec3f(0, 0, 0), ofVec3f(-0.2, -0.2, -0.2), ofVec3f(0.2, 0.2, 0.2)));
        
        

        
       
//        robotArmParams.add(bTrace.set("bTrace GML", false));
        robotArmParams.add(bCopy.set("get TCP", false));
        robotArmParams.add(bFollow.set("set TCP", false));
//        robotArmParams.add(b3DPath.set("3DPath", false));
        robotArmParams.add(bMove.set("Move", false));
//        robotArmParams.add(bFigure8.set("bFigure8", false));
        
        joints.setName("Joints");
        targetJoints.setName("Target Joints");
        jointSpeeds.setName("Joint Speeds");
        
        
        for(int i = 0; i < 6; i++){
            jointPos.push_back(ofParameter<float>());
            joints.add(jointPos.back().set("joint "+ofToString(i), 0, -360, 360));
        }
        
        for(int i = 0; i < 6; i++){
            targetJointPos.push_back(ofParameter<float>());
            targetJoints.add(targetJointPos.back().set("target joint "+ofToString(i), 0, -360, 360));
            jointPosIKRaw.push_back(ofParameter<float>());
            jointsIK.add(jointPosIKRaw.back().set("ik joint "+ofToString(i), 0, -360, 360));
        }
        
        for(int i = 0; i < 6; i++){
            jointVelocities.push_back(ofParameter<float>());
            jointSpeeds.add(jointVelocities.back().set("Joint Speed"+ofToString(i), 0, -100, 100));
        }
        
    };
    ofParameterGroup robotArmParams;
    ofParameter<ofVec3f> targetTCPPosition;
    ofParameter<ofVec4f> targetTCPOrientation;
    ofParameter<ofVec3f> tcpOrientation;
    ofParameter<ofVec3f> tcpPosition;
    
    ofParameter<ofVec3f> tcpOffset;
    
    ofParameterGroup joints;
    ofParameterGroup targetJoints;
    ofParameterGroup jointSpeeds;
    ofParameterGroup jointsIK;
    
    
    
    ofParameter<float> followLerp;
    ofParameter<float> avgAccel;
    vector<ofParameter<float> > jointPosIKRaw;
    vector<ofParameter<float> > jointPos;
    vector<ofParameter<float> > targetJointPos;
    vector<ofParameter<float> > jointVelocities;
    
    ofParameter<bool> bMove;
    ofParameter<bool> bFigure8;
    ofParameter<bool> bTrace;
    ofParameter<bool> bFollow;
    ofParameter<bool> bCopy;
    ofParameter<bool> bStop;
    ofParameter<bool> b3DPath;
    
    vector<double> currentJointPos;
    
    Joint targetTCP;
    
    
    
};
