//Copyright (c) 2016, Daniel Moore, Madaline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.

//--------------------------------------------------------------
//
//
// Robot following targets on a surface EXAMPLE
//
//
//--------------------------------------------------------------

//
// This example shows you how to:
//
// 1. Load a mesh surface.
// 2. Make paths on the surface for the robot to follow.
// 3. Use the surface normals as the orientation plane for the robot.


#include "ofApp.h"
#include "ofxAssimpModelLoader.h"

//--------------------------------------------------------------
void ofApp::setup(){
    ofSetFrameRate(120);
    ofSetVerticalSync(true);
    ofBackground(0);
    ofSetLogLevel(OF_LOG_SILENT);
    
    ofxAssimpModelLoader loader;
    loader.loadModel(ofToDataPath("mesh_srf.stl"));
    srf = loader.getMesh(0);
    
    // scale surface from to meters
    for (auto &v : srf.getVertices())
        v /= 100;
    

    buildToolpath(toolpath2D);
    projectToolpath(srf,toolpath2D,toolpath);
   
    cam.rotate(90, ofVec3f(1,0,0)); // make Z up
}



//--------------------------------------------------------------
void ofApp::update(){
    
   

}

//--------------------------------------------------------------
void ofApp::draw(){
    ofBackground(0);
    ofEnableAlphaBlending();
    
    
    cam.begin();
    ofPushMatrix();
    ofScale(1000);
    ofDrawAxis(.05);
    
    
    ofSetColor(100);
    srf.draw();
    ofSetColor(250,100);
    srf.drawWireframe();
    
    ofSetColor(ofColor::aqua);
    for (auto &face : srf.getUniqueFaces()){
        ofVec3f n = face.getFaceNormal();
        n /= -100; // scale to meters & flip
        ofVec3f pos = (face.getVertex(0) + face.getVertex(1) + face.getVertex(2)) / 3;
        
        ofDrawLine(pos, pos+n);
    }
    
    ofSetColor(200, 0, 0, 100);
    toolpath2D.draw();
    for (auto &v : toolpath2D)
        ofDrawLine(v.x,v.y,v.z,v.x,v.y,-.01);

    
    for (auto &face : testFaces){
        ofSetColor(200, 0, 0, 100);
        ofSetLineWidth(1);
        ofDrawLine(face.getVertex(0),face.getVertex(1));
        ofDrawLine(face.getVertex(0),face.getVertex(2));
        ofDrawLine(face.getVertex(2),face.getVertex(1));
        
        ofVec3f n = face.getFaceNormal();
        n /= -100; // scale to meters & flip
        ofVec3f pos = (face.getVertex(0) + face.getVertex(1) + face.getVertex(2)) / 3;
        
        ofSetColor(200, 0, 0);
        ofSetLineWidth(3);
        ofDrawLine(pos, pos+n);
    }
    ofSetLineWidth(1);
    
    ofSetColor(ofColor::yellow);
    for (auto &p : testPts)
        ofDrawSphere(p, .001);
    
    
    ofPopMatrix();
    cam.end();
}

//--------------------------------------------------------------
void ofApp::buildToolpath(ofPolyline &path){
    
    path.clear();
    
    // just make a XY circle for now ...
    
    float res = 60;
    float radius = .05;
    float theta = 360/res;
    
    for (int i=0; i<res; i++){
        ofPoint p = ofPoint(radius,0,.15);
        path.addVertex(p.rotate(theta*i, ofVec3f(0,0,1)));
    }
    path.close();
    
}

//--------------------------------------------------------------
void ofApp::projectToolpath(ofMesh mesh, ofPolyline &path2D, ofPolyline &path){
    
    path.clear();
    
    for (auto &v : path2D.getVertices()){
        // project along the Z axis
        // look for the distance between only the X,Y coords
        ofVec3f p0  = ofVec3f(v.x,v.y,0);
        
        // find the closest face to the 2D path point
        for (int i=0; i<mesh.getUniqueFaces().size(); i++){
            
            // hacky ... but works!
            ofPolyline face;
            face.addVertex(mesh.getFace(i).getVertex(0));
            face.addVertex(mesh.getFace(i).getVertex(1));
            face.addVertex(mesh.getFace(i).getVertex(2));
            face.close();
            face.getVertices()[0].z = 0;
            face.getVertices()[1].z = 0;
            face.getVertices()[2].z = 0;
            
            if (face.inside(p0)){
                testFaces.push_back(mesh.getFace(i));
                
                ofVec3f projectedPt;
                ofVec3f p = (mesh.getFace(i).getVertex(0)+mesh.getFace(i).getVertex(1)+mesh.getFace(i).getVertex(2))/3;
                ofVec3f n = mesh.getFace(i).getFaceNormal();
                n.normalize();
 
                // formula from: http://stackoverflow.com/questions/8942950/how-do-i-find-the-orthogonal-projection-of-a-point-onto-a-plane
                projectedPt = p0 - (p0-p).dot(n) * n;
                
                testPts.push_back(projectedPt);
                testPtNormals.push_back(n);
  
            }
        }
        
       
    }
    
    
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
