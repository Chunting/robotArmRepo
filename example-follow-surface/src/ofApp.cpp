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
    
    // scale surface to meters
    for (auto &v : srf.getVertices()){
        v /= 100;
    }
    

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
    ofScale(1000);      // draw in mm
    ofDrawAxis(.05);
    
    // show the surface
    ofSetColor(100);
    srf.draw();
    ofSetColor(250,100);
    srf.drawWireframe();
    
    // show surface normals
    ofSetColor(ofColor::aqua,100);
    for (auto &face : srf.getUniqueFaces()){
        ofVec3f n = face.getFaceNormal();
        n /= -100; // scale to meters & flip
        ofVec3f pos = (face.getVertex(0) + face.getVertex(1) + face.getVertex(2)) / 3;
        ofDrawLine(pos, pos+n);
    }
    
    // draw 2D & 3D toolpaths
    ofSetColor(200, 0, 0);
    toolpath2D.draw();
    ofSetColor(ofColor::chartreuse);
    ofSetLineWidth(3);
    toolpath.draw();
    
    // show toolpath normals
    ofSetColor(ofColor::blue);
    for (int i=0; i<toolpathOrients.size(); i++){
        
        ofQuaternion q = toolpathOrients[i];
        ofMatrix4x4 m44 = ofMatrix4x4(q);
        m44.setTranslation(toolpath[i]);
        ofPushMatrix();
        glMultMatrixf(m44.getPtr());
        ofDrawAxis(.01);
        ofPopMatrix();
        
    }
    ofSetLineWidth(1);
    
    ofPopMatrix();
    cam.end();
}

//--------------------------------------------------------------
void ofApp::buildToolpath(ofPolyline &path){
    
    path.clear();
    
    // make an XY circle as a toolpath ...    
    float res = 60;
    float radius = .05;
    float theta = 360/res;
    
    for (int i=0; i<res; i++){
        ofPoint p = ofPoint(radius,0,0);
        p.rotate(theta*i, ofVec3f(0,0,1));
        p.x += .0;
        path.addVertex(p);
    }
    path.close();
    
}

//--------------------------------------------------------------
void ofApp::projectToolpath(ofMesh mesh, ofPolyline &path2D, ofPolyline &path){
    
    path.clear();
    toolpath.clear();
    toolpathOrients.clear();
    
    for (auto &v : path2D.getVertices()){
        
        // find the closest face to the 2D path point
        for (int i=0; i<mesh.getUniqueFaces().size(); i++){
            
            // re-make the face as a 2D polyline so we can check
            // if the toolpath point is inside ... hacky, but it works!
            ofPolyline f;
            f.addVertex(mesh.getFace(i).getVertex(0));
            f.addVertex(mesh.getFace(i).getVertex(1));
            f.addVertex(mesh.getFace(i).getVertex(2));
            f.close();
            f.getVertices()[0].z = 0;
            f.getVertices()[1].z = 0;
            f.getVertices()[2].z = 0;
            
            // project the 2D point onto the mesh face
            if (f.inside(v)){
                auto face = mesh.getFace(i);
                
                // find the distance between our toolpath point and the mesh face
                ofVec3f facePos = (mesh.getFace(i).getVertex(0)+mesh.getFace(i).getVertex(1)+mesh.getFace(i).getVertex(2))/3;
                ofVec3f face2toolPt = v - facePos;
                float projectedDist = face2toolPt.dot(face.getFaceNormal().getNormalized());
               
                // use the distance as the length of a vertical projection vector
                ofVec3f length = ofVec3f(0,0,-projectedDist);
                ofVec3f projectedPt = v-length;
                
                // save the projected point and face normal
                toolpath.addVertex(projectedPt);
                ofQuaternion q;
                q.makeRotate(ofVec3f(0,0,-1), face.getFaceNormal().getNormalized());
                toolpathOrients.push_back(q);
            }
            
        }
    }
    
    toolpath.close();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    float offset = .005;

    
     if (key == OF_KEY_RIGHT){
        for (auto &p : toolpath2D)
            p.x += offset;
        projectToolpath(srf,toolpath2D,toolpath);
    }
    else if (key == OF_KEY_LEFT){
        for (auto &p : toolpath2D)
            p.x -= offset;
        projectToolpath(srf,toolpath2D,toolpath);
    }
    else if (key == OF_KEY_UP){
        for (auto &p : toolpath2D)
            p.y += offset;
        projectToolpath(srf,toolpath2D,toolpath);
    }
    else if (key == OF_KEY_DOWN){
        for (auto &p : toolpath2D)
            p.y -= offset;
        projectToolpath(srf,toolpath2D,toolpath);
    }
    
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
