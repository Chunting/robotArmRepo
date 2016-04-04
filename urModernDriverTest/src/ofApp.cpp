//Copyright (c) 2016, Daniel Moore, Madaline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.

#include "ofApp.h"
#include "URUtils.h"

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
    parameters.setup();
    
    panel.setup(parameters.robotArmParams);
    panel.setPosition(10, 10);
    
    workSurface.setup();
    
     
    panelJoints.setup(parameters.joints);
    panelJoints.setPosition(ofGetWindowWidth()-panelJoints.getWidth()-10, 10);
    panelWorkSurface.setup(workSurface.workSurfaceParams);
    panelWorkSurface.setPosition(panel.getWidth()+10, 10);
    panelWorkSurface.loadFromFile("worksurface.xml");
    
#ifdef ENABLE_NATNET
    natNet.setup("en6", "192.168.1.131");
#endif
    robot.setup("192.168.1.9",0, 1);
    robot.start();
    movement.setup();
    panel.add(movement.movementParams);
    speeds.assign(6, 0);
    parameters.bMove = false;
    // get the current pose on start up
    parameters.bCopy = true;
    panel.loadFromFile("settings.xml");
    gml.setup();
    gml.loadFile("gml/53520.gml");
    
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
    natNet.update();
#endif
    
  
    // pass the current joints from the robot to the kinematic solver
    vector<double> currentJointPos = robot.getJointPositions();
    movement.setCurrentJointPosition(currentJointPos);
    
    // update GUI params
    for(int i = 0; i < currentJointPos.size(); i++){
        parameters.jointPos[i] = (float)currentJointPos[i];
    }
    parameters.tcpPosition = robot.getToolPoint();
    
    // set target TCP to a default orientation, then modify
    targetTCP.rotation = ofQuaternion(90, ofVec3f(0, 0, 1));
    targetTCP.rotation*=ofQuaternion(90, ofVec3f(1, 0, 0));
    targetTCP.rotation*=ofQuaternion(0, ofVec3f(0,1, 0));
  
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
    // follow a user-defined position and orientation
    else if(parameters.bFollow){
        
        // follow mocap rigid body
        if (natNet.recordedPath.size() > 1){
        
//            auto &rb = recordedPath[0];
//            targetTCP.position = rb.matrix.getTranslation()/1000;
//            targetTCP.rotation = rb.matrix.getRotate();
            
            updateWorksurface(natNet.getCurrentRigidBody());
            
        }else{
            // go from current to next position
            targetTCP.position.interpolate(parameters.targetTCPPosition.get(), 0.1);
            // go from current orientation to next orientation (???)
            targetTCP.rotation *= ofQuaternion(parameters.targetTCPOrientation);
        }
        
        // update GUI params
        parameters.targetTCPPosition = targetTCP.position;
        parameters.tcpOrientation = targetTCP.rotation.getEuler();
    }
    // follow a pre-defined path
    else if(parameters.bTrace){
        
        // update the worksurface
        workSurface.update();
        
        // get the target point on the worksurface
        Joint workSrfTarget = workSurface.getTargetPoint(ofGetElapsedTimef()-tagStartTime);
        targetTCP.position = workSrfTarget.position;
        targetTCP.rotation *= workSrfTarget.rotation;
        
        // update GUI params
        parameters.targetTCPPosition = targetTCP.position;
        parameters.targetTCPOrientation = ofVec4f(targetTCP.rotation.x(), targetTCP.rotation.y(), targetTCP.rotation.z(), targetTCP.rotation.w());

    }
    // draw out a figure 8 in mid-air
    else if(parameters.bFigure8){
        
        // use a preset orientation
        targetTCP.rotation = ofQuaternion(90, ofVec3f(0, 0, 1));
        targetTCP.rotation*=ofQuaternion(90, ofVec3f(1, 0, 0));
        
        // update the target position
        targetTCP.position.interpolate(parameters.targetTCPPosition.get()+ofVec3f(cos(ofGetElapsedTimef()*0.25)*0.2, 0, sin(ofGetElapsedTimef()*0.25*2)*0.2), 0.5);
        
        // update GUI params
       parameters.tcpOrientation = targetTCP.rotation.getEuler();
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
    if(!parameters.bStop)
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
        natNet.draw();
    #endif
    
    
    if (!hideRobot)
        robot.model.draw();
    ofSetColor(255, 0, 255);
    ofPushMatrix();
    ofSetColor(255, 0, 255, 200);
    ofDrawSphere(toMM(parameters.tcpPosition.get()), 5);
    ofSetColor(255, 255, 0, 200);
    ofDrawSphere(toMM(targetTCP.position), 15);
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
    parameters.bMove = false;
    panel.saveToFile("settings.xml");
    panelWorkSurface.saveToFile("worksurface.xml");
    if(robot.isThreadRunning())
        robot.waitForThread();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    if(key == 'm'){
        parameters.bMove = !parameters.bMove;
    }
    if(key == ' '){
        vector<ofPolyline> strokes;
        float retract;
        if (natNet.recordedPath.size() > 0) {
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
        parameters.bTrace = true;
        parameters.bFollow = false;
        tagStartTime = ofGetElapsedTimef(); 
        
    }
    if(key == '1'){
        workSurface.setCorner(WorkSurface::UL, parameters.tcpPosition);
    }
    if(key == '2'){
        workSurface.setCorner(WorkSurface::UR, parameters.tcpPosition);
    }
    if(key == '3'){
        workSurface.setCorner(WorkSurface::LL, parameters.tcpPosition);
    }
    if(key == '4'){
        workSurface.setCorner(WorkSurface::LR, parameters.tcpPosition);
    }
    if(key == '8'){
        parameters.bFigure8 = !parameters.bFigure8;
    }
    
    
    handleViewportPresets(key);
    
    if (key == 'h')
        hideRobot = !hideRobot;

    if (key == 'r')
        natNet.record = !natNet.record;
    
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
void ofApp::updateWorksurface(const ofxNatNet::RigidBody &rb){
    
    if (natNet.recordedPath.size() > 1){
        
        // get the previous transformation matrix
        ofxNatNet::RigidBody prev = natNet.recordedPath[natNet.recordedPath.size()-2];
        
        // find the difference between the current transformation matrix
        ofMatrix4x4 diff = prev.matrix.getInverse() * rb.matrix;
        
        if (parameters.bFollow){
            ofQuaternion tempQ = toMM(targetTCP.rotation);
            ofVec3f tempP = toMM(targetTCP.position);;
            
            tempP = tempP * diff;
            tempQ = rb.getMatrix().getRotate();

            targetTCP.rotation = toMeters(tempQ);
            targetTCP.position = toMeters(tempP);

        }
        
        // apply matrix to each of the recorded bodies
        for (auto &tp: natNet.recordedPath){
            tp.matrix *= diff;
            
            // update markers
            for (int i=0; i<tp.markers.size(); i++)
                tp.markers[i] = tp.markers[i] * diff;
        }
        
        
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
            workSurface.targetPoints[0] = toMeters(rbWorksrf.getVertices()[0]);// / 1000;
            workSurface.targetPoints[1] = toMeters(rbWorksrf.getVertices()[1]);// / 1000;
            workSurface.targetPoints[2] = toMeters(rbWorksrf.getVertices()[2]);// / 1000;
            workSurface.targetPoints[3] = toMeters(rbWorksrf.getVertices()[3]);// / 1000;
            
            // override worksurface orientation
            workSurface.orientation = rb.getMatrix().getRotate();
        }
        
    }
    
}

//--------------------------------------------------------------
void ofApp::updateWorksurface(vector<ofxNatNet::Marker> &markers){
    
    if (markers.size() != 4)
        cout << "wrong number of unlabled makers for the worksurface: " << markers.size() << endl;
    else{
        
        // update mocap worksurface
        rbWorksrf.getVertices()[0] = markers[0];
        rbWorksrf.getVertices()[1] = markers[1];
        rbWorksrf.getVertices()[2] = markers[2];
        rbWorksrf.getVertices()[3] = markers[3];
        
        // update worksurface corners
        workSurface.targetPoints[0] = toMeters(markers[0]);
        workSurface.targetPoints[1] = toMeters(markers[1]);
        workSurface.targetPoints[2] = toMeters(markers[2]);
        workSurface.targetPoints[3] = toMeters(markers[3]);
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
