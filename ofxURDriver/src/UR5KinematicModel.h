//
//  KinectModel.h
//  urModernDriverTest
//
//  Created by dantheman on 2/20/16.
//
//

#pragma once
#include "ofMain.h"
#include "ofxAssimpModelLoader.h"
class UR5KinematicModel{
public:
    UR5KinematicModel();
    ~UR5KinematicModel();
    void setup();
    void update();
    void draw();
    void setToolMesh(ofMesh mesh);
    
    
    ofxAssimpModelLoader loader;
    vector<ofMesh> meshs;
    ofMesh toolMesh;
    
    ofShader shader;
    
    float elapsed_time, last_time;
    ofVec3f pt;
    vector<ofNode> jointsNode;
    vector<ofNode> jointsTargetNode;
    vector<double> jointsTargetRaw;
    vector<double> jointsRaw;
    vector<ofQuaternion> jointTargetQ;
    vector<ofQuaternion> jointsQ;
    vector<ofVec3f> angles;
    ofNode tool;
    ofNode targetPoint;
    ofNode toolD;
    ofEasyCam cam;
    
    ofParameter<bool> bDrawModel;
    ofParameter<bool> bDrawTargetModel;
};
