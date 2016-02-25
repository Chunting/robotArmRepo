//
//  KinectModel.cpp
//  urModernDriverTest
//
//  Created by dantheman on 2/20/16.
//
//

#include "UR5KinematicModel.h"
UR5KinematicModel::UR5KinematicModel(){
    
}
UR5KinematicModel::~UR5KinematicModel(){
    
}
void UR5KinematicModel::setup(){
    
    for(int i = 0; i < 6; i++){
        angles.push_back(ofVec3f());
        jointsQ.push_back(ofQuaternion());
        jointsNode.push_back(ofNode());
        
        jointsTargetNode.push_back(ofNode());
        jointTargetQ.push_back(ofQuaternion());
    }
    
    ofDirectory dir;
    dir.listDir(ofToDataPath("models"));
    dir.sort();
    dir.allowExt("dae");
    
    for(int i = 0; i < dir.size(); i++){
        loader.loadModel(dir.getPath(i));
        meshs.push_back(loader.getMesh(0));
    }
    
    
    jointsNode[0].setPosition(-0.525,0,6.386);
    jointsNode[1].setPosition(-85.795,0,-77.537);
    jointsNode[2].setPosition(-425.09,0,0);
    jointsNode[3].setPosition(-391.782,0,6.929);
    jointsNode[4].setPosition(-47.781,0,-46.634);
    jointsNode[5].setPosition(-45.829,0,-47.509);
    
    jointsTargetNode[0].setPosition(-0.525,0,6.386);
    jointsTargetNode[1].setPosition(-85.795,0,-77.537);
    jointsTargetNode[2].setPosition(-425.09,0,0);
    jointsTargetNode[3].setPosition(-391.782,0,6.929);
    jointsTargetNode[4].setPosition(-47.781,0,-46.634);
    jointsTargetNode[5].setPosition(-45.829,0,-47.509);
    
    jointsNode[1].setParent(jointsNode[0]);
    jointsNode[2].setParent(jointsNode[1]);
    jointsNode[3].setParent(jointsNode[2]);
    jointsNode[4].setParent(jointsNode[3]);
    jointsNode[5].setParent(jointsNode[4]);
    
    
    jointsTargetNode[1].setParent(jointsTargetNode[0]);
    jointsTargetNode[2].setParent(jointsTargetNode[1]);
    jointsTargetNode[3].setParent(jointsTargetNode[2]);
    jointsTargetNode[4].setParent(jointsTargetNode[3]);
    jointsTargetNode[5].setParent(jointsTargetNode[4]);
    
    
    angles[0].set(-1, 0, 0);
    angles[1].set(0, 0, -1);
    angles[2].set(0, 0, -1);
    angles[3].set(0, 0, -1);
    angles[4].set(-1, 0, 0);
    angles[5].set(0, 0, 1);
    
    shader.load("shaders/model");
    
    bDrawModel.set("Draw Model", true);
    bDrawTargetModel.set("Draw Target Model", false);
}
void UR5KinematicModel::setToolMesh(ofMesh mesh){
    toolMesh = mesh;
}
void UR5KinematicModel::update(){
    
}
void UR5KinematicModel::draw(){
    ofEnableDepthTest();
    ofSetColor(255, 255, 0);
    tool.draw();
    
    ofSetColor(255, 0, 255);
    targetPoint.draw();
    ofDisableDepthTest();
    
    if(bDrawModel){
        ofEnableDepthTest();
        shader.begin();
        shader.setUniform1f("elapsedTime", ofGetElapsedTimef());
        shader.setUniform1f("stage", 3.0);
        shader.setUniform1f("alpha", 1.0);
        ofPushMatrix();
        {
            ofRotateX(-90);
            ofRotateZ(90);
            
            ofTranslate(jointsNode[0].getPosition());
            float x;
            ofVec3f axis;
            ofQuaternion q = jointsNode[0].getGlobalOrientation();
            q.getRotate(x, axis);
            ofRotate(x, axis.x, axis.y, axis.z);
            meshs[0].draw();
            ofPushMatrix();
            {
                ofTranslate(jointsNode[1].getPosition());
                q = jointsNode[1].getOrientationQuat();
                q.getRotate(x, axis);
                ofRotate(x, axis.x, axis.y, axis.z);
                meshs[1].draw();
                ofPushMatrix();
                {
                    ofTranslate(jointsNode[2].getPosition());
                    q = jointsNode[2].getOrientationQuat();
                    q.getRotate(x, axis);
                    ofRotate(x, axis.x, axis.y, axis.z);
                    meshs[2].draw();
                    ofPushMatrix();
                    {
                        ofTranslate(jointsNode[3].getPosition());
                        q = jointsNode[3].getOrientationQuat();
                        q.getRotate(x, axis);
                        ofRotate(x, axis.x, axis.y, axis.z);
                        ofPushMatrix();
                        {
                            ofTranslate(jointsNode[4].getPosition());
                            meshs[3].draw();
                            q = jointsNode[4].getOrientationQuat();
                            q.getRotate(x, axis);
                            ofRotate(x, axis.x, axis.y, axis.z);
                            
                            ofPushMatrix();
                            {
                                ofTranslate(jointsNode[5].getPosition());
                                meshs[4].draw();
                                q = jointsNode[5].getOrientationQuat();
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
        ofDisableDepthTest();
    }
    
    if(bDrawTargetModel){
        ofEnableDepthTest();
        shader.begin();
        shader.setUniform1f("elapsedTime", ofGetElapsedTimef());
        shader.setUniform1f("stage", 3.);
        shader.setUniform1f("alpha", 0.9);
        ofPushMatrix();
        {
            ofRotateX(-90);
            ofRotateZ(90);
            
            ofTranslate(jointsTargetNode[0].getPosition());
            float x;
            ofVec3f axis;
            ofQuaternion q = jointsTargetNode[0].getGlobalOrientation();
            q.getRotate(x, axis);
            ofRotate(x, axis.x, axis.y, axis.z);
            ofSetColor(255, 0, 0);
            meshs[0].draw();
            ofPushMatrix();
            {
                ofTranslate(jointsTargetNode[1].getPosition());
                q = jointsTargetNode[1].getOrientationQuat();
                q.getRotate(x, axis);
                ofRotate(x, axis.x, axis.y, axis.z);
                meshs[1].draw();
                ofPushMatrix();
                {
                    
                    ofTranslate(jointsTargetNode[2].getPosition());
                    q = jointsTargetNode[2].getOrientationQuat();
                    q.getRotate(x, axis);
                    ofRotate(x, axis.x, axis.y, axis.z);
                    meshs[2].draw();
                    ofPushMatrix();
                    {
                        ofTranslate(jointsTargetNode[3].getPosition());
                        q = jointsTargetNode[3].getOrientationQuat();
                        q.getRotate(x, axis);
                        ofRotate(x, axis.x, axis.y, axis.z);
                        ofPushMatrix();
                        {
                            ofTranslate(jointsTargetNode[4].getPosition());
                            meshs[3].draw();
                            q = jointsTargetNode[4].getOrientationQuat();
                            q.getRotate(x, axis);
                            ofRotate(x, axis.x, axis.y, axis.z);
                            
                            ofPushMatrix();
                            {
                                ofTranslate(jointsTargetNode[5].getPosition());
                                meshs[4].draw();
                                q = jointsTargetNode[5].getOrientationQuat();
                                q.getRotate(x, axis);
                                ofRotate(x, axis.x, axis.y, axis.z);
                                targetPoint.draw();
                                
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
        ofDisableDepthTest();
    }
}