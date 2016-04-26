#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){

    ofSetFrameRate(60);
    ofSetVerticalSync(true);
    ofBackground(0);
    ofSetLogLevel(OF_LOG_SILENT);
    
    setupUserPanel();
    setupDebugPanel();

    // setup robot
    robot.setup(parameters);
    panel.add(robot.movement.movementParams);
    
    // setup geometry
    workSrf.setup("mesh_srf.stl");
    
    setupCameras();
}

//--------------------------------------------------------------
void ofApp::update(){
    moveArm();
    robot.update();
    
    updateActiveCamera();
}

//--------------------------------------------------------------
void ofApp::draw(){
    ofSetColor(255,160);
    ofDrawBitmapString("OF FPS "+ofToString(ofGetFrameRate()), 30, ofGetWindowHeight()-50);
    ofDrawBitmapString("Robot FPS "+ofToString(robot.robot.getThreadFPS()), 30, ofGetWindowHeight()-65);
    
    // show realtime robot
    cams[0].begin(ofRectangle(0, 0, ofGetWindowWidth()/2, ofGetWindowHeight()));
    workSrf.draw();
    robot.robot.model.draw();
    cams[0].end();
    
    // show simulation robot
    cams[1].begin(ofRectangle(ofGetWindowWidth()/2, 0, ofGetWindowWidth()/2, ofGetWindowHeight()));
    workSrf.draw();
    robot.movement.draw(robot.movement.selectedSolution);
    cams[1].end();
    
    drawGUI();
    hightlightViewports();
}

void ofApp::moveArm(){
    
    // get the current robot pose
    if(parameters.bCopy){
        parameters.bCopy = false;
        parameters.targetTCP.rotation = ofQuaternion(90, ofVec3f(0, 0, 1));
        parameters.targetTCP.rotation*=ofQuaternion(90, ofVec3f(1, 0, 0));
        
        // get the robot's pose
        parameters.targetTCP.position = parameters.actualTCP.position;
        parameters.targetTCP.rotation*=parameters.actualTCP.rotation;
        
        // update GUI params
        parameters.targetTCPPosition = parameters.targetTCP.position;
        parameters.targetTCPOrientation = ofVec4f(parameters.targetTCP.rotation.x(), parameters.targetTCP.rotation.y(), parameters.targetTCP.rotation.z(), parameters.targetTCP.rotation.w());
        
    }
    // get the current robot pose
    if(parameters.bFollow){
        parameters.targetTCP.position.interpolate(parameters.targetTCPPosition.get(), parameters.followLerp);
        parameters.targetTCP.rotation.slerp(parameters.followLerp, parameters.targetTCP.rotation, ofQuaternion(parameters.targetTCPOrientation));
        parameters.targetTCPOrientation = ofVec4f(parameters.targetTCP.rotation.x(), parameters.targetTCP.rotation.y(), parameters.targetTCP.rotation.z(), parameters.targetTCP.rotation.w());
    }
    
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
   
    parameters.bCopy = true;
    
    for (int i=0; i<N_CAMERAS; i++){
        activeCam = i;
        handleViewportPresets('4');
    }
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
    
    if (ofGetMouseX() < ofGetWindowWidth()/N_CAMERAS){
        activeCam = 0;
    }
    else{
        activeCam = 1;
    }
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
        viewportLabels[activeCam] = "TOP VIEW";
    }
    // LEFT VIEW
    else if (key == '2'){
        cams[activeCam].reset();
        cams[activeCam].setPosition(dist, 0, 0);
        cams[activeCam].lookAt(ofVec3f(0, 0, 0), ofVec3f(0, 0, 1));
        viewportLabels[activeCam] = "LEFT VIEW";
    }
    // FRONT VIEW
    else if (key == '3'){
        cams[activeCam].reset();
        cams[activeCam].setPosition(0, dist, 0);
        cams[activeCam].lookAt(ofVec3f(0, 0, 0), ofVec3f(0, 0, 1));
        viewportLabels[activeCam] = "FRONT VIEW";
    }
    // PERSPECTIVE VIEW
    else if (key == '4'){
        cams[activeCam].reset();
        cams[activeCam].setPosition(dist, dist, dist/4);
        cams[activeCam].lookAt(ofVec3f(0, 0, 0), ofVec3f(0, 0, 1));
        viewportLabels[activeCam] = "PERSPECTIVE VIEW";
    }
    else if (key == '5'){
        cams[activeCam].reset();
        cams[activeCam].setPosition(0, 0, -dist);
        cams[activeCam].lookAt(ofVec3f(0, 0, 0), ofVec3f(0, 0, 1));
        viewportLabels[activeCam] = "PERSPECTIVE VIEW";
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

    if(key == 'm'){
        parameters.bMove = !parameters.bMove;
    }

    handleViewportPresets(key);
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
