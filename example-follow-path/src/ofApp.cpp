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
    // 1. Create a 3D path & orientation planes for the robot.
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

    
    
    // build path
    ptIndex = 0;
    centroid = ofPoint(.5,.25,.25); // position in meters
    
    // create a 3D path & profile
    path = buildPath();
    profile = buildProfile(.025,4);
    
    // set the Z axis as the forward axis by default
    makeZForward = true;
    
    parsePts("path_XZ.txt", path_XZ);
    path = path_XZ;
}

void ofApp::parsePts(string filename, ofPolyline &polyline){
    ofFile file = ofFile(ofToDataPath(filename));
    
    ptf.clear();
   
    if(!file.exists()){
        ofLogError("The file " + filename + " is missing");
    }
    ofBuffer buffer(file);
    
    //Read file line by line
    for (ofBuffer::Line it = buffer.getLines().begin(), end = buffer.getLines().end(); it != end; ++it) {
        string line = *it;
        
        float scalar = 10;
        ofVec3f offset = ofVec3f(0, .25, 0);
        
        line = line.substr(1,line.length()-2);              // remove end { }
        vector<string> coords = ofSplitString(line, ", ");  // get x y z coordinates
        
        ofVec3f p = ofVec3f(ofToFloat(coords[0])*scalar,ofToFloat(coords[1])*scalar,ofToFloat(coords[2])*scalar);
        p += offset;
        
        polyline.addVertex(p);
        ptf.addPoint(p);
    }
    
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
    if (!pause && ptf.framesSize()>0){
        ptIndex = (ptIndex +1) % ptf.framesSize();
        
        orientation = ptf.frameAt(ptIndex);
        
        if (makeZForward)
            orientation = zForward(orientation);
        else if (makeZOut)
            orientation = zOut(orientation);
        
        // update the target TCP
        targetTCP.position = orientation.getTranslation();
        targetTCP.rotation *= orientation.getRotate();
    
    
    
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

    
   
    if (pause){
        // draw all the perp frames if we are paused
        for (int i=0; i<ptf.framesSize(); i++){
            ofMatrix4x4 m44 = ptf.frameAt(i);
            
            if (makeZForward)
                m44 = zForward(m44);
            else if (makeZOut)
                m44 = zOut(m44);
            
            ofSetColor(ofColor::aqua);
            ofPushMatrix();
            ofMultMatrix(m44);
            profile.draw();
            ofPopMatrix();
        }
    }
   

    // show the current orientation plane
    ofSetColor(ofColor::lightYellow);
    ofSetLineWidth(3);
    ofPushMatrix();
    ofMultMatrix(orientation);
    profile.draw();
    ofDrawAxis(.010);
    ofPopMatrix();
    
    // show the target point
    ofSetColor(ofColor::yellow);
    if (path.size() > 0)
        ofDrawSphere(path.getVertices()[ptIndex], .003);
    
   
    
    // show the 3D path
    ofSetLineWidth(.01);
    ofSetColor(ofColor::aqua);
    path.draw();
    
    // show debugging paths
    path_XZ.draw();


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
    ptf.clear();
    ofPolyline temp;
    
    ofNode n0;
    ofNode n1;
    ofNode n2;
    
    n0.setPosition(centroid.x,centroid.y,centroid.z);
    n1.setParent(n0);
    n1.setPosition(0,0,.2);
    n2.setParent(n1);
    n2.setPosition(0,.015,0);
    
    float totalRotation = 0;
    float step = .5;
    while (totalRotation < 360){
        
        n0.pan(step);
        n1.tilt(2);
        n2.roll(1);
        
        ofPoint p = n2.getGlobalPosition().rotate(90, ofVec3f(1,0,0));
        
        // and point to path
        temp.addVertex(p);
        
        // add point to perp frames
        ptf.addPoint(p);
        
        totalRotation += step;
    }
    
    temp.close();
    return temp;
}

//--------------------------------------------------------------
ofPolyline ofApp::buildProfile(float radius, int res){
    ofPolyline temp;
    
    // make a plane
    if (res == 4){
        temp.addVertex(ofVec3f(-radius/2, radius/2,0));
        temp.addVertex(ofVec3f( radius/2, radius/2,0));
        temp.addVertex(ofVec3f( radius/2,-radius/2,0));
        temp.addVertex(ofVec3f(-radius/2,-radius/2,0));
    }
    // make a polygon
    else{
        float theta = 360/res;
        for (int i=0; i<res; i++){
            ofPoint p = ofPoint(0,0,radius);
            temp.addVertex(p.rotate(theta*i, ofVec3f(1,0,0)));
        }
    }
    
    temp.close();
    return temp;
}


//--------------------------------------------------------------
ofMatrix4x4 ofApp::zForward(ofMatrix4x4 originalMat){
    
    ofVec3f pos  = originalMat.getTranslation();
    ofVec3f y = originalMat.getRowAsVec3f(1);   // local y-axis
    
    originalMat.setTranslation(0,0,0);
    originalMat.rotate(-90, y.x, y.y, y.z);     // rotate about the y
    originalMat.setTranslation(pos);
    
    return originalMat;
}


//--------------------------------------------------------------
ofMatrix4x4 ofApp::zOut(ofMatrix4x4 originalMat){
    
    ofVec3f pos  = originalMat.getTranslation();
    ofVec3f x = originalMat.getRowAsVec3f(0);   // local x-axis
    
    originalMat.setTranslation(0,0,0);
    originalMat.rotate(90, x.x, x.y, x.z);      // rotate about the y
    originalMat.setTranslation(pos);
    
    return originalMat;
}

//--------------------------------------------------------------

void ofApp::keyPressed(int key){
    float step = .01;   // 10 millimeters
    
    if(key == 'm'){
        parameters.bMove = !parameters.bMove;
    }
    
    else if (key == OF_KEY_UP){
        centroid.y += step;
        path = buildPath();
    }else if(key == OF_KEY_DOWN){
        centroid.y -= step;
        path = buildPath();
    }else if(key == OF_KEY_RIGHT){
        centroid.x += step;
        path = buildPath();
    }else if(key == OF_KEY_LEFT){
        centroid.x -= step;
        path = buildPath();
    }else if(key == ' '){
        centroid = ofPoint(0,0,0);
        path = buildPath();
    }
    
    else if (key == OF_KEY_SHIFT)
        pause = !pause;
    
    else if (key == '1'){
        makeZOut = false;
        makeZForward = true;
    }
    else if (key == '2'){
        makeZForward = false;
        makeZOut = true;
    }
    else if (key == '3'){
        makeZForward = false;
        makeZOut = false;
    }

    
   
    
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
