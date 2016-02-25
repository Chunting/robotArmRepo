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
    
    ofDirectory dir;
    dir.listDir(ofToDataPath("models"));
    dir.sort();
    dir.allowExt("dae");
    
    for(int i = 0; i < dir.size(); i++){
        loader.loadModel(dir.getPath(i));
        meshs.push_back(loader.getMesh(0));
    }
    
    
    
    
    //    -86.20,70.50,0
    //    -425.00,0,0
    //    -346.93,-54.50,0
    //    45.50,-47.50,0
    //    47.50,-45.50,0
    //    45.50,-47.50,0
    
    joints[0].setPosition(-0.525,0,6.386);
    joints[1].setPosition(-85.795,0,-77.537);
    joints[2].setPosition(-425.09,0,0);
    joints[3].setPosition(-391.782,0,6.929);
    joints[4].setPosition(-47.781,0,-46.634);
    //    joints[5].setPosition(47.50,-45.50,0);
    joints[5].setPosition(-45.829,0,-47.509);
    
    joints[1].setParent(joints[0]);
    joints[2].setParent(joints[1]);
    joints[3].setParent(joints[2]);
    joints[4].setParent(joints[3]);
    joints[5].setParent(joints[4]);
    
    
    angles[0].set(-1, 0, 0);
    angles[1].set(0, 0, -1);
    angles[2].set(0, 0, -1);
    angles[3].set(0, 0, -1);
    angles[4].set(-1, 0, 0);
    angles[5].set(0, 0, 1);
    
    //    jointLength[0].set(0, 0, 0);
    //    jointLength[1].set(10, 0, 0);
    //    jointLength[2].set(425, 0, 0);
    //    jointLength[3].set(392, 0, 0);
    //    jointLength[4].set(0, 109, 0);
    //    jointLength[5].set(0, 82, 0);
    
    shader.load("shaders/model");
}
void KinematicModel::setToolMesh(ofMesh mesh){
    toolMesh = mesh;
}
void KinematicModel::update(){
    
}
void KinematicModel::draw(){
    ofSetColor(255, 255, 255);
    tool.draw();
    
    shader.begin();
    shader.setUniform1f("elapsedTime", ofGetElapsedTimef());
    shader.setUniform1f("stage", abs(sin(ofGetElapsedTimef()*0.001)*9.0));
    ofPushMatrix();
    {
        ofRotateX(-90);
        ofRotateZ(90);
        
        ofTranslate(joints[0].getPosition());
        float x;
        ofVec3f axis;
        ofQuaternion q = joints[0].getGlobalOrientation();
        q.getRotate(x, axis);
        ofRotate(x, axis.x, axis.y, axis.z);
        ofSetColor(255, 0, 0);
        meshs[0].draw();
        ofPushMatrix();
        {
            ofSetColor(0, 255, 0);
            ofTranslate(joints[1].getPosition());
            q = joints[1].getOrientationQuat();
            q.getRotate(x, axis);
            ofRotate(x, axis.x, axis.y, axis.z);
            meshs[1].draw();
            ofPushMatrix();
            {
                
                ofSetColor(0, 0, 255);
                ofTranslate(joints[2].getPosition());
                q = joints[2].getOrientationQuat();
                q.getRotate(x, axis);
                ofRotate(x, axis.x, axis.y, axis.z);
                meshs[2].draw();
                ofPushMatrix();
                {
                    ofTranslate(joints[3].getPosition());
                    ofSetColor(255, 0, 255);
                    q = joints[3].getOrientationQuat();
                    q.getRotate(x, axis);
                    ofRotate(x, axis.x, axis.y, axis.z);
                    ofPushMatrix();
                    {
                        ofTranslate(joints[4].getPosition());
                        meshs[3].draw();
                        q = joints[4].getOrientationQuat();
                        q.getRotate(x, axis);
                        ofRotate(x, axis.x, axis.y, axis.z);
                        
                        ofPushMatrix();
                        {
                            ofSetColor(255, 255, 0);
                            ofTranslate(joints[5].getPosition());
                            meshs[4].draw();
                            q = joints[5].getOrientationQuat();
                            q.getRotate(x, axis);
                            ofRotate(x, axis.x, axis.y, axis.z);
                            toolMesh.draw();
                            
                        }
                        ofPopMatrix();
                    }
                    ofPopMatrix();
                }
                ofPopMatrix();
            }
            ofPopMatrix();
        }
        ofPopMatrix();
    }
    ofPopMatrix();
    shader.end();
}