//
//  KinectModel.cpp
//  urModernDriverTest
//
//  Created by dantheman on 2/20/16.
// Copyright (c) 2016, Daniel Moore, Madaline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
//

#include "UR5KinematicModel.h"
UR5KinematicModel::UR5KinematicModel(){
    
}
UR5KinematicModel::~UR5KinematicModel(){
    
}
void UR5KinematicModel::setup(){
    
    ofDirectory dir;
    dir.listDir(ofToDataPath("models"));
    dir.sort();
    dir.allowExt("dae");
    //
//    for(int i = 0; i < dir.size(); i++){
//        loader.loadModel(dir.getPath(i));
//        meshs.push_back(loader.getMesh(0));
//    }
    
    // load robot mesh
    loader.loadModel(ofToDataPath("models/ur5.dae"));
    for(int i = 0; i < loader.getNumMeshes(); i++){
        meshs.push_back(loader.getMesh(i));
    }
    
    // load default tool ... right now exporting with 34.5 offset
    loader.loadModel(ofToDataPath("models/myTool.dae"));
    setToolMesh(loader.getMesh(loader.getNumMeshes()-1));
    tool.offset = ofVec3f(0,-135,0);    // is this doing anything?
    
    jointsRaw.assign(6, 0.0);
    jointsRaw[1] = -PI/2.0;
    jointsRaw[3] = -PI/2.0;
    
    joints.resize(6);
    
    joints[0].position.set(0, 0, 0);
    joints[1].position.set(0, -72.238, 83.204);
    joints[2].position.set(0,-77.537,511.41);
    joints[3].position.set(0, -70.608, 903.192);
    joints[4].position.set(0, -117.242, 950.973);
    joints[5].position.set(0, -164.751, 996.802);
    tool.position.set(joints[5].position + ofVec3f(0,-135,0)); // tool tip position
    
    for(int i = 1; i < joints.size(); i++){
        joints[i].offset = joints[i].position-joints[i-1].position;
    }
    tool.offset = joints[5].offset;
    
    
    
    joints[0].axis.set(0, 0, 1);
    joints[1].axis.set(0, -1, 0);
    joints[2].axis.set(0, -1, 0);
    joints[3].axis.set(0, -1, 0);
    joints[4].axis.set(0, 0, 1);
    joints[5].axis.set(0, 1, 0);
    tool.axis.set(joints[5].axis);
    
    joints[0].rotation.makeRotate(0, ofVec3f(1, 0, 0), 0, ofVec3f(0, 1, 0), 0, ofVec3f(0, 0, 1));
    joints[1].rotation.makeRotate(0, ofVec3f(1, 0, 0), 0, ofVec3f(0, 1, 0), 0, ofVec3f(0, 0, 1));
    joints[2].rotation.makeRotate(0, ofVec3f(1, 0, 0), 0, ofVec3f(0, 1, 0), 0, ofVec3f(0, 0, 1));
    joints[3].rotation.makeRotate(0, ofVec3f(1, 0, 0), 0, ofVec3f(0, 1, 0), 0, ofVec3f(0, 0, 1));
    joints[4].rotation.makeRotate(0, ofVec3f(1, 0, 0), 0, ofVec3f(0, 1, 0), 0, ofVec3f(0, 0, 1));
    joints[5].rotation.makeRotate(0, ofVec3f(1, 0, 0), 0, ofVec3f(0, 1, 0), 0, ofVec3f(0, 0, 1));
    tool.rotation = joints[5].rotation;
    
    shader.load("shaders/model");
    
    bDrawModel.set("Draw Model", true);
    bDrawTargetModel.set("Draw Target Model", false);
    bDrawModel = true;
}

ofQuaternion UR5KinematicModel::getToolPointMatrix(){

    return joints[0].rotation*joints[1].rotation*joints[2].rotation*joints[3].rotation*joints[4].rotation;
}

void UR5KinematicModel::setToolMesh(ofMesh mesh){
    
//    // add an initial 34.5 offset to place on the end of joint 5
//    for (auto &v : mesh.getVertices())
//        v.y -= 34.5;
    
    toolMesh = mesh;
}
void UR5KinematicModel::update(){
    
}
void UR5KinematicModel::draw(){
    ofDrawAxis(100);
    ofEnableDepthTest();
    ofSetColor(255, 255, 0);
    ofDrawSphere(tool.position*ofVec3f(1000, 1000, 1000), 10);
    
    
    ofSetColor(255, 0, 255);
    //    targetPoint.draw();
    ofDisableDepthTest();
    
    //
    //    for(int i = 0; i < joints.size(); i++){
    //        jointsNode[i].draw();
    //    }
    
    
    
    if(bDrawModel){
        ofEnableDepthTest();
        shader.begin();
        shader.setUniform1f("elapsedTime", abs(sin(ofGetElapsedTimef()*0.01)));
        shader.setUniform1f("stage", 3.0);
        shader.setUniform1f("alpha", 1.0);
        float x;
        ofVec3f axis;
        ofQuaternion q;
        ofVec3f offset;
        ofPushMatrix();
        {
            for(int i = 0; i < joints.size(); i++)
            {
                float x;
                ofVec3f axis;
                q = joints[i].rotation;
                q.getRotate(x, axis);
                ofTranslate(joints[i].offset);
                ofDrawAxis(10);
                if(i >= 3){
                    ofPushMatrix();
                    ofRotateZ(-180);
                    ofRotateX(-180);
                    ofScale(100, 100, 100);
                    meshs[i].draw();
                    ofPopMatrix();
                }
                ofRotate(x, axis.x, axis.y, axis.z);
                ofDrawAxis(100);
                if(i < 3){
                    ofPushMatrix();
                    ofRotateZ(-180);
                    ofRotateX(-180);
                    ofScale(100, 100, 100);
                    meshs[i].draw();
                    ofPopMatrix();
                }
            }
            toolMesh.draw();
        }
        ofPopMatrix();
            
        shader.end();
        ofDisableDepthTest();
        
        ofPushMatrix();
        {
            for(int i = 0; i < joints.size(); i++)
            {
                float x;
                ofVec3f axis;
                q = joints[i].rotation;
                q.getRotate(x, axis);
                ofTranslate(joints[i].offset);
                ofDrawAxis(10);
                ofRotate(x, axis.x, axis.y, axis.z);
                ofDrawAxis(100);

            }
        }
        ofPopMatrix();

    }
}