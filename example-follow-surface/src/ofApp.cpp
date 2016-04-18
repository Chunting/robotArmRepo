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
    ofSetFrameRate(60);
    ofSetVerticalSync(true);
    ofBackground(0);
    ofSetLogLevel(OF_LOG_SILENT);
    
    setupUserPanel();
    setupDebugPanel();
    setupCameras();
    setupGeometry();
    
    // setup robot
    robot.setup(parameters);
    panel.add(robot.movement.movementParams);
    
    speeds.assign(6, 0);
    parameters.bMove = false;
    parameters.bCopy = true;
    
    panel.loadFromFile("settings.xml");
    
    ofEnableAlphaBlending();
}



//--------------------------------------------------------------
void ofApp::update(){
    
    robot.update();
    
    if(parameters.bTrace){
        Joint pose;
        pose.position = toolpath.getVertices()[pathIndex];
        pose.rotation = toolpathOrients[pathIndex];
        robot.updatePath(pose);
        
        pathIndex = (pathIndex+1)%toolpath.getVertices().size();
    }
   
    updateActiveCamera();
}

//--------------------------------------------------------------
void ofApp::draw(){
    ofBackground(0);
    
    ofSetColor(255,160);
    ofDrawBitmapString("OF FPS "+ofToString(ofGetFrameRate()), 30, ofGetWindowHeight()-50);
    ofDrawBitmapString("Robot FPS "+ofToString(robot.robot.getThreadFPS()), 30, ofGetWindowHeight()-65);
    
    // show realtime robot
    cams[0].begin(ofRectangle(0, 0, ofGetWindowWidth()/2, ofGetWindowHeight()));
    robot.robot.model.draw();
    drawGeometry();
    cams[0].end();
    
    // show simulated robot
    cams[1].begin(ofRectangle(ofGetWindowWidth()/2, 0, ofGetWindowWidth()/2, ofGetWindowHeight()));
    robot.movement.draw();
    drawGeometry();
    cams[1].end();
    
    
    drawGUI();
    hightlightViewports();
}

//--------------------------------------------------------------
void ofApp::setupUserPanel(){
    parameters.setup();
    
    panel.setup(parameters.robotArmParams);
    panel.setPosition(10, 10);
}

//--------------------------------------------------------------
void ofApp::setupDebugPanel(){
    panelJoints.setup(parameters.joints);
    panelTargetJoints.setup(parameters.targetJoints);
    panelJointsSpeed.setup(parameters.jointSpeeds);
    panelJointsIK.setup(parameters.jointsIK);
    
    panelJoints.setPosition(ofGetWindowWidth()-panelJoints.getWidth()-10, 10);
    panelJointsIK.setPosition(panelJoints.getPosition().x-panelJoints.getWidth(), 10);
    panelTargetJoints.setPosition(panelJointsIK.getPosition().x-panelJoints.getWidth(), 10);
    panelJointsSpeed.setPosition(panelTargetJoints.getPosition().x-panelJoints.getWidth(), 10);
}

void ofApp::setupCameras(){
    
    for(int i = 0; i < N_CAMERAS; i++){
        cams[i].setup();
        cams[i].autosavePosition = true;
        cams[i].usemouse = false;
        cams[i].cameraPositionFile = "cam_"+ofToString(i)+".xml";
        cams[i].viewport = ofRectangle(ofGetWindowWidth()/2*i, 0, ofGetWindowWidth()/2, ofGetWindowHeight());
        cams[i].loadCameraPosition();
    }
}

//--------------------------------------------------------------
void ofApp::setupGeometry(){
    ofxAssimpModelLoader loader;
    loader.loadModel(ofToDataPath("mesh_srf.stl"));
    srf = loader.getMesh(0);
    
    // scale surface to meters
    for (auto &v : srf.getVertices()){
        v /= 100;
    }
    
    buildToolpath(toolpath2D);
    
    // move toolpath surface to be in a more reachable position (temp fix)
    ofVec3f offset = ofVec3f(0,.5,.35);
    for (auto &v : srf.getVertices()){
        v += offset;
    }
    for (auto &v : toolpath2D.getVertices()){
        v += offset;
    }
    
    projectToolpath(srf,toolpath2D,toolpath);
    
    
}

//--------------------------------------------------------------
void ofApp::drawGeometry(){
    ofPushMatrix();
    ofPopStyle();
    
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
    
    ofPushStyle();
    ofPopMatrix();
}

//--------------------------------------------------------------
void ofApp::drawGUI(){
    panel.draw();
    panelJoints.draw();
    panelJointsIK.draw();
    panelJointsSpeed.draw();
    panelTargetJoints.draw();
}


//--------------------------------------------------------------
void ofApp::updateActiveCamera(){
    
    if (ofGetMouseX() < ofGetWindowWidth()/N_CAMERAS)
    {
        activeCam = 0;
    }
    else
    {
        activeCam = 1;
    }
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
void ofApp::handleViewportPresets(int key){
    
    float dist = 2000;
    float zOffset = 450;
    
    // TOP VIEW
    if (key == '1'){
        cams[activeCam].reset();
        cams[activeCam].setPosition(0, 0, dist);
        cams[activeCam].lookAt(ofVec3f(0, 0, 0), ofVec3f(0, 0, 1));
        cams[activeCam].movedManually();
        viewportLabels[activeCam] = "TOP VIEW";
    }
    // LEFT VIEW
    else if (key == '2'){
        cams[activeCam].reset();
        cams[activeCam].setPosition(dist, 0, 0);
        cams[activeCam].lookAt(ofVec3f(0, 0, 0), ofVec3f(0, 0, 1));
        cams[activeCam].movedManually();
        viewportLabels[activeCam] = "LEFT VIEW";
    }
    // FRONT VIEW
    else if (key == '3'){
        cams[activeCam].reset();
        cams[activeCam].setPosition(0, dist, 0);
        cams[activeCam].lookAt(ofVec3f(0, 0, 0), ofVec3f(0, 0, 1));
        cams[activeCam].movedManually();
        viewportLabels[activeCam] = "FRONT VIEW";
    }
    // PERSPECTIVE VIEW
    else if (key == '4'){
        cams[activeCam].reset();
        cams[activeCam].setPosition(dist, dist, dist/4);
        cams[activeCam].lookAt(ofVec3f(0, 0, 0), ofVec3f(0, 0, 1));
        cams[activeCam].movedManually();
        viewportLabels[activeCam] = "PERSPECTIVE VIEW";
    }
    if(key == '5'){
        cams[activeCam].usemouse = true;
    }

}

//--------------------------------------------------------------
void ofApp::hightlightViewports(){
    ofPushStyle();
    
    float w = 6;
    ofSetLineWidth(w);
    
    // highlight right viewport
    if (activeCam == 1){
        ofSetColor(ofColor::white,80);
        ofDrawLine(ofGetWindowWidth()/2, 0, ofGetWindowWidth()/2, ofGetWindowHeight());
        ofDrawLine(ofGetWindowWidth()/2, w/2, ofGetWindowWidth(), w/2);
        ofDrawLine(ofGetWindowWidth()-w/2, 0, ofGetWindowWidth()-w/2, ofGetWindowHeight());
        ofDrawLine(ofGetWindowWidth()/2, ofGetWindowHeight()-w/2, ofGetWindowWidth(), ofGetWindowHeight()-w/2);
        ofSetColor(ofColor::white,40);
        ofDrawLine(0, w/2, ofGetWindowWidth()/2, w/2);
        ofDrawLine(w/2, 0, w/2, ofGetWindowHeight());
        ofDrawLine(0, ofGetWindowHeight()-w/2, ofGetWindowWidth()/2, ofGetWindowHeight()-w/2);
    }
    // hightligh left viewport
    else{
        ofSetLineWidth(w);
        ofSetColor(ofColor::white,80);
        ofDrawLine(ofGetWindowWidth()/2, 0, ofGetWindowWidth()/2, ofGetWindowHeight());
        ofDrawLine(0, w/2, ofGetWindowWidth()/2, w/2);
        ofDrawLine(w/2, 0, w/2, ofGetWindowHeight());
        ofDrawLine(0, ofGetWindowHeight()-w/2, ofGetWindowWidth()/2, ofGetWindowHeight()-w/2);
        ofSetColor(ofColor::white,40);
        ofDrawLine(ofGetWindowWidth()/2, w/2, ofGetWindowWidth(), w/2);
        ofDrawLine(ofGetWindowWidth()-w/2, 0, ofGetWindowWidth()-w/2, ofGetWindowHeight());
        ofDrawLine(ofGetWindowWidth()/2, ofGetWindowHeight()-w/2, ofGetWindowWidth(), ofGetWindowHeight()-w/2);
    }
    
    // show Viewport info
    
    
    ofSetColor(ofColor::white,200);
    ofDrawBitmapString(viewportLabels[0], 30, ofGetWindowHeight()-30);
    ofDrawBitmapString("REALTIME", ofGetWindowWidth()/2 - 90, ofGetWindowHeight()-30);
    ofDrawBitmapString(viewportLabels[1], ofGetWindowWidth()/2+30, ofGetWindowHeight()-30);
    ofDrawBitmapString("SIMULATED", ofGetWindowWidth() - 100, ofGetWindowHeight()-30);
    
    ofPopStyle();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    float offset = .005;

    if(key == '5'){
        cams[activeCam].usemouse = true;
    }
    
    handleViewportPresets(key);
    
//     if (key == OF_KEY_RIGHT){
//        for (auto &p : toolpath2D)
//            p.x += offset;
//        projectToolpath(srf,toolpath2D,toolpath);
//    }
//    else if (key == OF_KEY_LEFT){
//        for (auto &p : toolpath2D)
//            p.x -= offset;
//        projectToolpath(srf,toolpath2D,toolpath);
//    }
//    else if (key == OF_KEY_UP){
//        for (auto &p : toolpath2D)
//            p.y += offset;
//        projectToolpath(srf,toolpath2D,toolpath);
//    }
//    else if (key == OF_KEY_DOWN){
//        for (auto &p : toolpath2D)
//            p.y -= offset;
//        projectToolpath(srf,toolpath2D,toolpath);
//    }
    
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
