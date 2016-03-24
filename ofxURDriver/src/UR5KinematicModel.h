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

struct Joint{
    ofVec3f offset;
    ofVec3f axis;
    ofVec3f position;
    ofQuaternion rotation;
};

class UR5KinematicModel{
public:
    UR5KinematicModel();
    ~UR5KinematicModel();
    void setup();
    void update();
    void draw();
    void setToolMesh(ofMesh mesh);
    ofQuaternion getToolPointMatrix();
    
    ofxAssimpModelLoader loader;
    vector<ofMesh> meshs;
    ofMesh toolMesh;
    
    ofShader shader;
    
    float elapsed_time, last_time;
    ofVec3f pt;
    vector<Joint> joints;
    vector<double> jointsProcessed;
    vector<double> jointsRaw;
    vector<double> toolPointRaw;
    Joint tool;

    Joint dtoolPoint;
    
    ofEasyCam cam;
    
    ofParameter<bool> bDrawModel;
    ofParameter<bool> bDrawTargetModel;
};
