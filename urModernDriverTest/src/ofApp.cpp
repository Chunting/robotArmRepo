//Copyright (c) 2016, Daniel Moore, Madaline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.

#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    ofSetFrameRate(120);
    ofSetVerticalSync(true);
    ofBackground(0);
    ofSetLogLevel(OF_LOG_SILENT);
    
    string interface_name = "en0"; // or network interface name
    
    //    tcp.setPosition(0, 0, 0);
    parent.setPosition(0, 0, 0);
    //    tcp.setParent(parent);
    robotArmParams.add(targetTCP_POS.set("Set TCP POS", ofVec3f(0, 0, 0), ofVec3f(-1, -1, -1), ofVec3f(1, 1, 1)));
    robotArmParams.add(targetTCP_ORIENT.set("Set TCP ORIENT",ofVec4f(0,0,0,1), ofVec4f(-1,-1,-1,-1), ofVec4f(1,1,1,1)));
    robotArmParams.add(TCP_POS.set("Robot TCP POS", ofVec3f(0, 0, 0), ofVec3f(-1, -1, -1), ofVec3f(1, 1, 1)));
    robotArmParams.add(TCP_ORIENT_XYZ.set("Robot TCP ORIENT", ofVec3f(0, 0, 0), ofVec3f(-TWO_PI, -TWO_PI, -TWO_PI), ofVec3f(TWO_PI, TWO_PI, TWO_PI)));
    
    panel.setup(robotArmParams);
    panel.add(bFollow.set("set TCP", false));
    panel.add(bTrace.set("bTrace GML", false));
    panel.add(bCopy.set("get TCP", false));

    panel.setPosition(10, 10);
    
    workSurface.setup();
    
    joints.setName("Joints");
    joints.add(bMove.set("Move", false));
    joints.add(avgAccel.set("avgAccel", 0, 0, 200));
    joints.add(bFigure8.set("bFigure8", false));
    for(int i = 0; i < 6; i++){
        jointPos.push_back(ofParameter<float>());
        targetJointPos.push_back(ofParameter<float>());
        jointVelocities.push_back(ofParameter<float>());
        joints.add(jointPos.back().set("joint "+ofToString(i), 0, -TWO_PI, TWO_PI));
        joints.add(targetJointPos.back().set("target joint "+ofToString(i), 0, -TWO_PI, TWO_PI));
        joints.add(jointVelocities.back().set("Joint Speed"+ofToString(i), 0, -100, 100));
    }
    
    panelJoints.setup(joints);
    panelJoints.setPosition(ofGetWindowWidth()-panelJoints.getWidth()-10, 10);
    panelWorkSurface.setup(workSurface.workSurfaceParams);
    panelWorkSurface.setPosition(panel.getWidth()+10, 10);
    panelWorkSurface.loadFromFile("worksurface.xml");
    
#ifdef ENABLE_NATNET
    setupNatNet();
#endif
    robot.setup("192.168.1.9",0, 1);
    robot.start();
    movement.setup();
    panel.add(movement.movementParams);
    speeds.assign(6, 0);
    bMove = false;
    // get the current pose on start up
    bCopy = true;
    panel.loadFromFile("settings.xml");
    
    gml.loadFile("gml/53520.gml", 0, 0, 640, 480);
    
    /* 3D Navigation */
//    cams[1] = &movement.cam;
    // need to move URMove camera to ofApp

    handleViewportPresets('p');
    
    
    // temp fix ... set up rigid body worksurface for drawing/following mocap
    float w = 400;
    float h = 300;
    float offset = 00;//400;

    rbWorksrf.addVertex(ofVec3f(-w/2,  h/2 + offset, 0)); // UL
    rbWorksrf.addVertex(ofVec3f( w/2,  h/2 + offset, 0)); // LL
    rbWorksrf.addVertex(ofVec3f( w/2, -h/2 + offset, 0)); // LR
    rbWorksrf.addVertex(ofVec3f(-w/2, -h/2 + offset, 0)); // LL
    rbWorksrf.close();
}

//--------------------------------------------------------------
void ofApp::update(){
#ifdef ENABLE_NATNET
    updateNatNet();
#endif
    
  
    // pass the current joints from the robot to the kinematic solver
    vector<double> currentJointPos = robot.getJointPositions();
    movement.setCurrentJointPosition(currentJointPos);
    
    // update GUI params
    for(int i = 0; i < currentJointPos.size(); i++)
        jointPos[i] = (float)currentJointPos[i];
    TCP_POS = robot.getToolPoint();
    
    // set target TCP to a default orientation, then modify
    targetTCP.rotation = ofQuaternion(90, ofVec3f(0, 0, 1));
    targetTCP.rotation*=ofQuaternion(90, ofVec3f(1, 0, 0));
    targetTCP.rotation*=ofQuaternion(0, ofVec3f(0,1, 0));
  
    // assign the target pose to the current robot pose
    if(bCopy){
        bCopy = false;
        
        // get the robot's position
        targetTCP.position = robot.getToolPoint();
        // get the robot's orientation
        // targetTCP.rotation = .... <-- why is this working without grabbing the current orientation?
        
        // update GUI params
        targetTCP_POS = targetTCP.position;
        targetTCP_ORIENT = ofVec4f(targetTCP.rotation.x(), targetTCP.rotation.y(), targetTCP.rotation.z(), targetTCP.rotation.w());
        
    }
    // follow a user-defined position and orientation
    else if(bFollow){
        
        // follow mocap rigid body
        if (recordedPath.size() > 1){
            
//            auto &rb = recordedPath[0];
//            targetTCP.position = rb.matrix.getTranslation()/1000;
//            targetTCP.rotation = rb.matrix.getRotate();
            
            updateWorksurface(recordedPath[0]);
            
        }
        
        else{
            // go from current to next position
            targetTCP.position.interpolate(targetTCP_POS.get(), 0.1);
            // go from current orientation to next orientation (???)
            targetTCP.rotation *= ofQuaternion(targetTCP_ORIENT);
        }
        
        // update GUI params
        targetTCP_POS = targetTCP.position;
        TCP_ORIENT_XYZ = targetTCP.rotation.getEuler();
    }
    // follow a pre-defined path
    else if(bTrace){
        
        // update the worksurface
        workSurface.update();
        
        // get the target point on the worksurface
        Joint workSrfTarget = workSurface.getTargetPoint(ofGetElapsedTimef()-tagStartTime);
        targetTCP.position = workSrfTarget.position;
        targetTCP.rotation *= workSrfTarget.rotation;
        
        // update GUI params
        targetTCP_POS = targetTCP.position;
        targetTCP_ORIENT = ofVec4f(targetTCP.rotation.x(), targetTCP.rotation.y(), targetTCP.rotation.z(), targetTCP.rotation.w());

    }
    // draw out a figure 8 in mid-air
    else if(bFigure8){
        
        // use a preset orientation
        targetTCP.rotation = ofQuaternion(90, ofVec3f(0, 0, 1));
        targetTCP.rotation*=ofQuaternion(90, ofVec3f(1, 0, 0));
        
        // update the target position
        targetTCP.position.interpolate(targetTCP_POS.get()+ofVec3f(cos(ofGetElapsedTimef()*0.25)*0.2, 0, sin(ofGetElapsedTimef()*0.25*2)*0.2), 0.5);
        
        // update GUI params
        TCP_ORIENT_XYZ = targetTCP.rotation.getEuler();
    }
    
    
    // send the target TCP to the kinematic solver
    movement.addTargetPoint(targetTCP);
    movement.update();
    
    
    // get back the target joint trajectories
    vector<double> target = movement.getTargetJointPos();
    for(int i = 0; i < target.size(); i++)
        targetJointPos[i] = (float)target[i];
   
    // set the joint speeds
    vector<double> tempSpeeds;
    tempSpeeds.assign(6, 0);
    tempSpeeds = movement.getCurrentSpeed();
    for(int i = 0; i < tempSpeeds.size(); i++)
        jointVelocities[i] = (float)tempSpeeds[i];
   
    // move the robot to the target TCP
    avgAccel = movement.getAcceleration();
    if(bMove)
        robot.setSpeed(tempSpeeds, avgAccel);
    
    
    
    /* 3D Navigation */
    
    // update which easyCam is active
    if (ofGetMouseX() < ofGetWindowWidth()/N_CAMERAS)
        activeCam = 0;
    else
        activeCam = 1;
    
}


void ofApp::testMotors(){
    vector<double> foo;
    foo.assign(6, 0.0);
    if(!stop)
        foo[0] = 1.0;
    robot.setSpeed(foo, ofRandom(1, 100));
}

//--------------------------------------------------------------
void ofApp::draw(){
    ofEnableAlphaBlending();
    
    ofSetColor(255,160);
    ofDrawBitmapString("OF FPS "+ofToString(ofGetFrameRate()), 30, ofGetWindowHeight()-50);
    ofDrawBitmapString("Robot FPS "+ofToString(robot.getThreadFPS()), 30, ofGetWindowHeight()-65);
    cams[0].begin(ofRectangle(0, 0, ofGetWindowWidth()/2, ofGetWindowHeight()));
    
    #ifdef ENABLE_NATNET
        drawNatNet();
    #endif
    
    
    if (!hideRobot)
        robot.model.draw();
    ofSetColor(255, 0, 255);
    ofPushMatrix();
    ofSetColor(255, 0, 255, 200);
    ofDrawSphere(TCP_POS.get()*ofVec3f(1000, 1000, 1000), 5);
    ofSetColor(255, 255, 0, 200);
    ofDrawSphere(targetTCP.position*ofVec3f(1000, 1000, 1000), 15);
    ofPopMatrix();
    workSurface.draw();
    cams[0].end();
    
    movement.draw();
    
    ofPushMatrix();
    ofSetColor(255, 0, 255);
    gml.draw();
    ofPopMatrix();
    
    panel.draw();
    panelJoints.draw();
    panelWorkSurface.draw();
    
    /* 3D Navigation */
    hightlightViewports();
}

void ofApp::exit(){
    bMove = false;
    panel.saveToFile("settings.xml");
    panelWorkSurface.saveToFile("worksurface.xml");
    if(robot.isThreadRunning())
        robot.stopThread();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    if(key == 'm'){
        bMove = !bMove;
    }
    if(key == ' '){
        vector<ofPolyline> strokes;
        float retract;
        if (recordedPath.size() > 0) {
            // DEBUGGING....testing mocap tracking single point on worksrf
            ofPolyline temp;
            temp.addVertex(ofPoint (0,0,0));
            
            retract = 0;
            strokes.push_back(temp);
        }
        else{
            retract = -.1;
            strokes = gml.getPath(1.0);
        }
        
        workSurface.addStrokes(strokes,retract);
        bTrace = true;
        bFollow = false;
        tagStartTime = ofGetElapsedTimef(); 
        
    }
    if(key == '1'){
        workSurface.setCorner(WorkSurface::UL, TCP_POS);
    }
    if(key == '2'){
        workSurface.setCorner(WorkSurface::UR, TCP_POS);
    }
    if(key == '3'){
        workSurface.setCorner(WorkSurface::LL, TCP_POS);
    }
    if(key == '4'){
        workSurface.setCorner(WorkSurface::LR, TCP_POS);
    }
    if(key == '8'){
        bFigure8 = !bFigure8;
    }
    
    
    handleViewportPresets(key);
    
    if (key == 'h')
        hideRobot = !hideRobot;

    if (key == 'r')
        record = !record;
    
    if (key == 'q'){
        exit();
        std::exit(0);
    }
}

//--------------------------------------------------------------
void ofApp::handleViewportPresets(int key){
    
    float dist = 2000;
    float zOffset = 450;
    
    // TOP VIEW
    if (key == 't'){
        cams[activeCam].reset();
        cams[activeCam].setDistance(dist);
        viewportLabels[activeCam] = "TOP VIEW";
    }
    // LEFT VIEW
    else if (key == 'l'){
        cams[activeCam].reset();
        cams[activeCam].rotate(90, ofVec3f(0,1,0));
        cams[activeCam].rotate(90, ofVec3f(1,0,0));
        cams[activeCam].setDistance(dist);
        cams[activeCam].boom(zOffset);
        viewportLabels[activeCam] = "LEFT VIEW";
    }
    // FRONT VIEW
    else if (key == 'f'){
        cams[activeCam].reset();
        cams[activeCam].rotate(90, ofVec3f(1,0,0));
        cams[activeCam].setDistance(dist);
        cams[activeCam].boom(zOffset);
        viewportLabels[activeCam] = "FRONT VIEW";
    }
    // PERSPECTIVE VIEW
    else if (key == 'p'){
        // hardcoded perspective camera
        ofMatrix4x4 perspective = ofMatrix4x4(0.991627, -0.124872, -0.0329001, 0,
                                              0.055994, 0.186215, 0.980912, 0,
                                              -0.116362, -0.974541, 0.191648, 0,
                                              -748.61, -1057.62,  376.122, 1);
        cams[activeCam].reset();
        cams[activeCam].setGlobalPosition(perspective.getTranslation());
        cams[activeCam].setGlobalOrientation(perspective.getRotate());
        viewportLabels[activeCam] = "PERSPECTIVE VIEW";
    }
    // CUSTOM  VIEW
    else if (key == 'c'){
        cams[activeCam].reset();
        cams[activeCam].setGlobalPosition(savedCamMats[activeCam].getTranslation());
        cams[activeCam].setGlobalOrientation(savedCamMats[activeCam].getRotate());
        viewportLabels[activeCam] = "SAVED VIEW";
    }
    // Record custom view port
    if (key == 's'){
        savedCamMats[activeCam] = cams[activeCam].getGlobalTransformMatrix();
        viewportLabels[activeCam] = "New Viewport Saved!";
        cout << ofToString(savedCamMats[activeCam]) << endl;
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
void ofApp::setupNatNet(){
    string myIP = "192.168.1.141";
    string serverIP = "192.168.1.131";
    sender.setup("192.168.1.255", 7777);
    natnet.setup(myIP, serverIP);  // interface name, server ip
    natnet.setScale(1000);
    natnet.setDuplicatedPointRemovalDistance(25);
}

//--------------------------------------------------------------
void ofApp::updateNatNet(){
    natnet.update();
    
    if (natnet.getNumRigidBody()==1){
        const ofxNatNet::RigidBody &rb = natnet.getRigidBodyAt(0);
        
        // add to the rigid body history
        if (record){
            recordedPath.push_back(rb);
            
            // store previous 20 rigid bodies
            if (recordedPath.size()>20)
                recordedPath.erase(recordedPath.begin());
            
            // check if the rigid body is moving
            if (recordedPath.size() > 0){
                float dist = .25;
                ofVec3f curr = rb.matrix.getTranslation();
                ofVec3f prev = recordedPath[recordedPath.size()-1].matrix.getTranslation();
                isMoving = curr.squareDistance(prev) > dist*dist ;
            }
        }

        // move the worksurface based on the rigid body
        updateWorksurface(rb);
        
    }
    
    
    /* Handle OSC */
    
//    ofxOscBundle bundle;
//    int count = 0;
//    for (int i = 0; i < max(0, (int)natnet.getNumMarkersSet() - 1); i++) {
//        for (int j = 0; j < natnet.getMarkersSetAt(i).size(); j++) {
//            ofxOscMessage b;
//            b.setAddress("/natnet/marker");
//            b.addInt32Arg(j);
//            b.addFloatArg(natnet.getMarkersSetAt(i)[j].x);
//            b.addFloatArg(natnet.getMarkersSetAt(i)[j].y);
//            b.addFloatArg(natnet.getMarkersSetAt(i)[j].z);
//            bundle.addMessage(b);
//            count++;
//        }
//    }
//    
//    for (int i = 0; i < natnet.getNumRigidBody(); i++) {
//        const ofxNatNet::RigidBody &RB = natnet.getRigidBodyAt(i);
//        
//        ofxOscMessage m;
//        m.setAddress("/natnet/rigidbody");
//        m.addInt32Arg(i);
//        for(int j = 0; j < RB.markers.size(); j++){
//            m.addFloatArg(RB.markers[j].x);
//            m.addFloatArg(RB.markers[j].y);
//            m.addFloatArg(RB.markers[j].z);
//        }
//        count++;
//        bundle.addMessage(m);
//    }
//    if(count > 0){
//        sender.sendBundle(bundle);
//    }

}

//--------------------------------------------------------------
void ofApp::drawNatNet(){
//    cam.begin();
    
    ofDrawAxis(100);
    
    drawHistory();
    
//    ofFill();
//    
//    // draw all markers set
//    ofSetColor(255, 128);
//    for (int i = 0; i < max(0, (int)natnet.getNumMarkersSet() - 1); i++) {
//        for (int j = 0; j < natnet.getMarkersSetAt(i).size(); j++) {
//            ofDrawBox(natnet.getMarkersSetAt(i)[j], 3);
//        }
//    }
//    
//    // draw all markers
//    ofSetColor(255, 30);
//    for (int i = 0; i < natnet.getNumMarker(); i++) {
//        ofDrawBox(natnet.getMarker(i), 3);
//    }
//    
//    ofNoFill();
//    
//    // draw filtered markers
//    ofSetColor(255);
//    for (int i = 0; i < natnet.getNumFilterdMarker(); i++) {
//        ofDrawBox(natnet.getFilterdMarker(i), 10);
//    }
//    
//    // draw rigidbodies
//    for (int i = 0; i < natnet.getNumRigidBody(); i++) {
//        const ofxNatNet::RigidBody &RB = natnet.getRigidBodyAt(i);
//        
//        if (RB.isActive())
//            ofSetColor(0, 255, 0);
//        else
//            ofSetColor(255, 0, 0);
//        
//        ofPushMatrix();
//        glMultMatrixf(RB.getMatrix().getPtr());
//        ofDrawAxis(30);
//        ofPopMatrix();
//        
//        glBegin(GL_LINE_LOOP);
//        for (int n = 0; n < RB.markers.size(); n++) {
//            glVertex3fv(RB.markers[n].getPtr());
//        }
//        glEnd();
//        
//        for (int n = 0; n < RB.markers.size(); n++) {
//            ofDrawBox(RB.markers[n], 5);
//        }
//    }
    
//    // draw skeletons
//    for (int j = 0;  j < natnet.getNumSkeleton(); j++) {
//        const ofxNatNet::Skeleton &S = natnet.getSkeletonAt(j);
//        ofSetColor(0, 0, 255);
//        
//        for (int i = 0; i < S.joints.size(); i++) {
//            const ofxNatNet::RigidBody &RB = S.joints[i];
//            ofPushMatrix();
//            glMultMatrixf(RB.getMatrix().getPtr());
//            ofDrawBox(5);
//            ofPopMatrix();
//        }
//    }
    
//    cam.end();
//    
//    string str;
//    str += "frames: " + ofToString(natnet.getFrameNumber()) + "\n";
//    str += "data rate: " + ofToString(natnet.getDataRate()) + "\n";
//    str += string("connected: ") + (natnet.isConnected() ? "YES" : "NO") + "\n";
//    str += "num markers set: " + ofToString(natnet.getNumMarkersSet()) + "\n";
//    str += "num marker: " + ofToString(natnet.getNumMarker()) + "\n";
//    str += "num filtered (non regidbodies) marker: " +
//    ofToString(natnet.getNumFilterdMarker()) + "\n";
//    str += "num rigidbody: " + ofToString(natnet.getNumRigidBody()) + "\n";
//    str += "num skeleton: " + ofToString(natnet.getNumSkeleton()) + "\n";
//    
//    ofSetColor(255);
//    ofDrawBitmapString(str, 10, 20);
}

//--------------------------------------------------------------
void ofApp::updateWorksurface(const ofxNatNet::RigidBody &rb){
    
    
    if (recordedPath.size() > 1){
        
        // get the previous transformation matrix
        ofxNatNet::RigidBody prev = recordedPath[recordedPath.size()-2];
        
        // find the difference between the current transformation matrix
        ofMatrix4x4 diff = prev.matrix.getInverse() * rb.matrix;
        
        if (bFollow){
            ofQuaternion tempQ = targetTCP.rotation * 1000;
            ofVec3f tempP = targetTCP.position * 1000;
            
            tempP = tempP * diff;
            tempQ = rb.getMatrix().getRotate();

            targetTCP.rotation = tempQ / 1000;
            targetTCP.position = tempP / 1000;
        }
        
        // apply matrix to each of the recorded bodies
        for (auto &tp: recordedPath){
            tp.matrix *= diff;
            
            // update markers
            for (int i=0; i<tp.markers.size(); i++)
                tp.markers[i] = tp.markers[i] * diff;
        }
        
        // apply the difference to the corners of the worksurface
//        for (int i=0; i<4; i++){
//            auto v = workSurface.targetPoints[i].get() * ofVec3f(1000,1000,1000) * diff;
//            workSurface.targetPoints[i] = v/1000;
//        }
        
        
        // initialize rigidbody worksurface
        if (rbWorksrf.getVertices()[0].z == 0){
            // orient worksurface with rigid body
            rbWorksrf.getVertices()[0] = rbWorksrf.getVertices()[0] * rb.matrix;
            rbWorksrf.getVertices()[1] = rbWorksrf.getVertices()[1] * rb.matrix;
            rbWorksrf.getVertices()[2] = rbWorksrf.getVertices()[2] * rb.matrix;
            rbWorksrf.getVertices()[3] = rbWorksrf.getVertices()[3] * rb.matrix;
        }
        else{
            // update rigidbody worksurface
            rbWorksrf.getVertices()[0] = rbWorksrf.getVertices()[0] * diff;
            rbWorksrf.getVertices()[1] = rbWorksrf.getVertices()[1] * diff;
            rbWorksrf.getVertices()[2] = rbWorksrf.getVertices()[2] * diff;
            rbWorksrf.getVertices()[3] = rbWorksrf.getVertices()[3] * diff;
            
            // update worksurface corner
            workSurface.targetPoints[0] = rbWorksrf.getVertices()[0] / 1000;
            workSurface.targetPoints[1] = rbWorksrf.getVertices()[1] / 1000;
            workSurface.targetPoints[2] = rbWorksrf.getVertices()[2] / 1000;
            workSurface.targetPoints[3] = rbWorksrf.getVertices()[3] / 1000;
            
            // override worksurface position and orientation
            workSurface.orientation = rb.getMatrix().getRotate();
            workSurface.position = rb.getMatrix().getTranslation();
        }

        
        
//        workSurface.position = rb.getMatrix().getTranslation();
//        workSurface.orientation = rb.getMatrix().getRotate();
        
    }
    
}

//--------------------------------------------------------------
void ofApp::drawHistory(){

    // show path
    ofPolyline tp;
    for (auto &path: recordedPath){
        ofSetColor(ofColor::azure);
        tp.addVertex(path.getMatrix().getTranslation());
    }
    tp.draw();
    
    // show rigid body
    ofPolyline bodies;
    float alpha = 255;
    float step = 255 / (recordedPath.size()+1);
    for (auto &rb: recordedPath){
        ofSetColor(ofColor::navajoWhite, alpha);
        for (int i = 0; i < rb.markers.size(); i++)
            bodies.addVertex(rb.markers[i]);
        alpha -= step;
    }
    bodies.draw();
    
    // show test srf
    ofPushStyle();
    ofSetLineWidth(5);
    if (isMoving)
        ofSetColor(255, 0, 0);
    else
        ofSetColor(ofColor::aqua);
    rbWorksrf.draw();
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
    // clear viewport label
    viewportLabels[activeCam] = "";
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
