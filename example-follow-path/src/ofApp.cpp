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
    // 1. Load and interpolate a path for the robot.
    // 2. Calculate a target TCP from point on path.
    // 3. Move the robot based on a target TCP.
    // 4. Move the path while moving the robot.


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

    
    // build 2D perp plane
    float w = .035;
    plane2D.addVertex(-w/2, w/2);
    plane2D.addVertex( w/2, w/2);
    plane2D.addVertex( w/2, -w/2);
    plane2D.addVertex(-w/2, -w/2);
    plane2D.close();
    
    plane3D = plane2D;
    
    // build path
    pathIndex = 0;
    centroid = ofPoint(.4,.1,.25); // position in meters
    path = buildPath();  

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
    if (path.getVertices().size() > 2){
        
        pathIndex = (pathIndex + 1) % path.getVertices().size();
        
        // find the orientation
        ofVec3f curr = path.getVertices()[pathIndex];
        ofVec3f next;
        if (pathIndex == path.getVertices().size()-1){
            next = path.getVertices()[0];
        }else{
            next = path.getVertices()[pathIndex+1];
        }
        
        // set the plane normal
        norm = next - curr;
        norm.normalize();
    
 
        // align orientation plane with normal
        for (int i=0; i<4; i++){
           ofVec3f p = plane2D.getVertices()[i];
            ofQuaternion q;
            q.makeRotate(ofVec3f(0,0,1), norm);
            
            p = p * q;
            p += path.getVertices()[pathIndex];
            
            plane3D.getVertices()[i].set(p);
        }
        
        // calculate local axis of plane3D (for visualizing)
        u = plane3D.getVertices()[3] - plane3D.getVertices()[2];
        v = plane3D.getVertices()[3] - plane3D.getVertices()[0];
        u.normalize();
        v.normalize();

        // assign position and orientation
        targetTCP.position = plane3D.getVertices()[0].getMiddle(plane3D.getVertices()[2]);
        targetTCP.rotation.makeRotate(ofVec3f(1,0,0), norm);
    }
        
    // send the target TCP to the kinematic solver
    movement.addTargetPoint(targetTCP);
    movement.update();
    
    
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
    
    // show the 3D path
    ofSetColor(ofColor::aqua);
    path.draw();
    
    
    // show the target point
    ofSetColor(ofColor::yellow, 100);
    if (path.size() > 0)
        ofDrawSphere(path.getVertices()[pathIndex], .005);

    // show the local axes of the 3D plane
    ofQuaternion q;
    q.makeRotate(ofVec3f(0,0,1),norm);
    ofMatrix4x4 m44 = ofMatrix4x4(q);
    m44.setTranslation(path.getVertices()[pathIndex]);
    ofPushMatrix();
    glMultMatrixf(m44.getPtr());
    ofDrawAxis(.03);
    ofPopMatrix();
   
    // draw the perp plane
    ofSetLineWidth(.03);
    ofSetColor(ofColor::aliceBlue);
    plane3D.draw();
 
    
    ofPopStyle();
    ofPopMatrix();
    cam.end();
    
    
    // draw simulated robot
    movement.draw();
    
    // draw the GUI
    panel.draw();
    panelJoints.draw();
    
    hightlightViewports();
}

//--------------------------------------------------------------
ofPolyline ofApp::buildPath(){

    ofPolyline temp;
    float freq = .150;  // robot coordinates are in meters
    float amp  = .035;
    
    ofNode n0;
    ofNode n1;
    ofNode n2;
    
    n0.setPosition(centroid);
    n1.setParent(n0);
    n1.setPosition(0, 0, freq);
    n2.setParent(n1);
    n2.setPosition(0, amp, 0);
    
    float step = .5;
    float totalRotation = 0;
    while (totalRotation < 360){
        
        totalRotation += step;
        n0.pan(step);
        n1.tilt(2);
        n2.roll(1);
        
        temp.addVertex(n2.getGlobalPosition().rotate(90, ofVec3f(1,0,0)));
    }
    
    temp.close();
    return temp;
}

//--------------------------------------------------------------

void ofApp::keyPressed(int key){
    float step = .01;   // 10 millimeters
    
    if(key == 'm'){
        parameters.bMove = !parameters.bMove;
    }
    
    else if (key == OF_KEY_UP){
        centroid.y += step;
    }else if(key == OF_KEY_DOWN){
        centroid.y -= step;
    }else if(key == OF_KEY_RIGHT){
        centroid.x += step;
    }else if(key == OF_KEY_LEFT){
        centroid.x -= step;
    }else if(key == ' '){
        centroid = ofPoint(0,0,0);
    }
    
    path = buildPath();
    
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
