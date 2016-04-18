//Copyright (c) 2016, Daniel Moore, Madaline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.

//--------------------------------------------------------------
//
//
// Robot following targets on a path EXAMPLE
//
//
//--------------------------------------------------------------

//
// This example shows you how to:
//
// 1. Create a 3D path & orientation planes for the robot to follow.
// 2. Calculate a target TCP from point on path.
// 3. Move the robot based on a target TCP.
// 4. Move the path while moving the robot using keyPressed.


#include "ofApp.h"
#include "URUtils.h"



//--------------------------------------------------------------
void ofApp::setup(){
    ofSetFrameRate(120);
    ofSetVerticalSync(true);
    ofBackground(0);
    ofSetLogLevel(OF_LOG_SILENT);
    
    // set up the GUI
    parameters.setup();
    panel.setup(parameters.robotArmParams);
    panel.setPosition(10, 10);
    panel.loadFromFile("settings.xml");
    
    // connect to the robot
    robot.setup("192.168.1.9",0, 1); // use the IP address of your robot here
    robot.start();
    
    // set up kinematic model
    movement.setup();
    panel.add(movement.movementParams);
    panelJoints.setup(parameters.joints);
    panelJoints.setPosition(ofGetWindowWidth()-panelJoints.getWidth()-10, 10);
    
    // assign speeds and disable movement
    speeds.assign(6, 0);
    parameters.bMove = false;
    
    // get the current pose on start up
    parameters.bCopy = true;
    path.setup();
    
}

//--------------------------------------------------------------
void ofApp::update(){
    
    // pass the current joints from the robot to the kinematic solver
    vector<double> currentJointPos = robot.getJointPositions();
    movement.setCurrentJointPosition(currentJointPos);
    
    // update GUI params
    for(int i = 0; i < currentJointPos.size(); i++){
        parameters.jointPos[i] = (float)currentJointPos[i];
    }
    parameters.tcpPosition = robot.getToolPoint();
    
    // set target TCP to a default orientation, then modify <-- I don't think this is doing anything.
    targetTCP.rotation = ofQuaternion(90, ofVec3f(0, 0, 1));
    targetTCP.rotation*= ofQuaternion(90, ofVec3f(1, 0, 0));
    targetTCP.rotation*= ofQuaternion( 0, ofVec3f(0, 1, 0));
    
    // assign the target pose to the current robot pose
    if(parameters.bCopy){
        parameters.bCopy = false;
        
        // get the robot's position
        targetTCP.position = robot.getToolPoint();
        // get the robot's orientation
        // targetTCP.rotation = .... <-- why is this working without grabbing the current orientation?
        
        // update GUI params
        parameters.targetTCPPosition = targetTCP.position;
        parameters.targetTCPOrientation = ofVec4f(targetTCP.rotation.x(), targetTCP.rotation.y(), targetTCP.rotation.z(), targetTCP.rotation.w());
        
    }
    
    // find the current point on the path
    if (!pause){
        
        // update the target TCP <-- from "bTrace" example
        ofMatrix4x4 orientation = path.getNextPose();
        targetTCP.position = orientation.getTranslation();
        targetTCP.rotation *= orientation.getRotate().conj();

        // send the target TCP to the kinematic solver
        movement.addTargetPoint(targetTCP);
        movement.update();
    }
    
    // get back the target joint trajectories
    vector<double> target = movement.getTargetJointPos();
    for(int i = 0; i < target.size(); i++){
        parameters.targetJointPos[i] = (float)target[i];
    }
    
    // set the joint speeds
    vector<double> tempSpeeds;
    tempSpeeds.assign(6, 0);
    tempSpeeds = movement.getCurrentSpeed();
    for(int i = 0; i < tempSpeeds.size(); i++){
        parameters.jointVelocities[i] = (float)tempSpeeds[i];
    }
    // move the robot to the target TCP
    parameters.avgAccel = movement.getAcceleration();
    if(parameters.bMove){
        robot.setSpeed(tempSpeeds, parameters.avgAccel);
    }
    
    
    // update which easyCam is active
    if (ofGetMouseX() < ofGetWindowWidth()/2)
        activeCam = 0;
    else
        activeCam = 1;
}

//--------------------------------------------------------------
void ofApp::draw(){
    ofBackground(0);
    ofEnableAlphaBlending();
    
    ofPushStyle();
    ofSetColor(255,160);
    ofDrawBitmapString("OF FPS "+ofToString(ofGetFrameRate()), 30, ofGetWindowHeight()-50);
    ofDrawBitmapString("Robot FPS "+ofToString(robot.getThreadFPS()), 30, ofGetWindowHeight()-65);
    ofPopStyle();
    
    cam.begin(ofRectangle(0, 0, ofGetWindowWidth()/2, ofGetWindowHeight()));
    
    // show the realtime robot
    robot.model.draw();
    
    
    // show the 3D path
    ofPushMatrix();
    ofPushStyle();
    ofScale(1000); // scale from meter to millimeters for visualizing
    
    
    
    path.draw();
    
    ofPopStyle();
    ofPopMatrix();
    cam.end();
    
    cam.begin(ofRectangle(ofGetWindowWidth()/2, 0, ofGetWindowWidth()/2, ofGetWindowHeight()));
    // draw simulated robot
    movement.draw();
    cam.end();
    // draw the GUI
    panel.draw();
    panelJoints.draw();
    
    hightlightViewports();
}


//--------------------------------------------------------------

void ofApp::keyPressed(int key){
    if(key == 'm'){
        parameters.bMove = !parameters.bMove;
    }else if(key == ' '){
        pause = !pause;
    }
    path.keyPressed(key);
    handleViewportPresets(key);
    
}

//--------------------------------------------------------------
void ofApp::handleViewportPresets(int key){
    
    float dist = 2000;
    float zOffset = 450;
    
    // TOP VIEW
    if (key == 't'){
        cam.reset();
        cam.setDistance(dist);
        viewportLabel = "TOP VIEW";
    }
    // LEFT VIEW
    else if (key == 'l'){
        cam.reset();
        cam.rotate(90, ofVec3f(0,1,0));
        cam.rotate(90, ofVec3f(1,0,0));
        cam.setDistance(dist);
        cam.boom(zOffset);
        viewportLabel = "LEFT VIEW";
    }
    // FRONT VIEW
    else if (key == 'f'){
        cam.reset();
        cam.rotate(90, ofVec3f(1,0,0));
        cam.setDistance(dist);
        cam.boom(zOffset);
        viewportLabel = "FRONT VIEW";
    }
    // PERSPECTIVE VIEW
    else if (key == 'p'){
        // hardcoded perspective camera
        ofMatrix4x4 perspective = ofMatrix4x4(0.721792, 0.689126, -0.0641996, 0,
                                              -0.0677436, 0.162658, 0.984354, 0,
                                              0.688787, -0.70615, 0.164089, 0,
                                              777.021, -719.789,  366.449, 1);
        cam.reset();
        cam.setGlobalPosition(perspective.getTranslation());
        cam.setGlobalOrientation(perspective.getRotate());
        viewportLabel = "PERSPECTIVE VIEW";
    }
    // CUSTOM  VIEW
    else if (key == 'c'){
        cam.reset();
        cam.setGlobalPosition(savedCamMat.getTranslation());
        cam.setGlobalOrientation(savedCamMat.getRotate());
        viewportLabel = "SAVED VIEW";
    }
    // Record custom view port
    if (key == 's'){
        savedCamMat = cam.getGlobalTransformMatrix();
        viewportLabel= "New Viewport Saved!";
        cout << ofToString(savedCamMat) << endl;
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
    ofDrawBitmapString(viewportLabel, 30, ofGetWindowHeight()-30);
    ofDrawBitmapString("REALTIME", ofGetWindowWidth()/2 - 90, ofGetWindowHeight()-30);
    ofDrawBitmapString(viewportLabel, ofGetWindowWidth()/2+30, ofGetWindowHeight()-30);
    ofDrawBitmapString("SIMULATED", ofGetWindowWidth() - 100, ofGetWindowHeight()-30);
    
    ofPopStyle();
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
