//Copyright (c) 2016, Daniel Moore, Madaline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.

#include "ofApp.h"
#include "URUtils.h"

//--------------------------------------------------------------
void ofApp::setup(){
    ofSetFrameRate(60);
    ofSetVerticalSync(true);
    ofBackground(0);
    ofSetLogLevel(OF_LOG_NOTICE);
    setupViewports();
    
    parameters.setup();
    robot.setup(parameters);
    setupGUI();
    setupTimeline();
    positionGUI();
#ifdef ENABLE_NATNET
    natNet.setup("en6", "192.168.1.131");
#endif
}

void ofApp::setupViewports(){
    
    viewportReal = ofRectangle((21*ofGetWindowWidth()/24)/2, 0, (21*ofGetWindowWidth()/24)/2, 7*ofGetWindowHeight()/8);
    viewportSim = ofRectangle(0, 0, (21*ofGetWindowWidth()/24)/2, 7*ofGetWindowHeight()/8);
    
    activeCam = 0;
    
    
    for(int i = 0; i < N_CAMERAS; i++){
        cams.push_back(new ofEasyCam());
        savedCamMats.push_back(ofMatrix4x4());
        viewportLabels.push_back("");
    }
    
    cams[0]->begin(viewportReal);
    cams[0]->end();
    cams[0]->enableMouseInput();
    
<<<<<<< HEAD
    
    cams[1]->begin(viewportSim);
    cams[1]->end();
    cams[1]->enableMouseInput();
    
}
=======
#ifdef ENABLE_NATNET
    setupNatNet();
#endif
    robot.setup("192.168.1.9",0, 1);
    robot.start();
    movement.setup();
    speeds.assign(6, 0);
    bMove = false;
    
    // get the current pose on start up
    bCopy = true;
>>>>>>> parent of 31eef24... Drawing on tracked surface

void ofApp::setupTimeline(){
    
    
    gizmo.setDisplayScale(1.0);
    tcpNode.setPosition(ofVec3f(0.5, 0.5, 0.5)*1000);
    tcpNode.setOrientation(parameters.targetTCP.rotation);
    gizmo.setNode( tcpNode);
    
    ofDirectory dir;
    string dirName = "timeline/saves/"+ofGetTimestampString();
    dir.createDirectory(ofToDataPath(dirName));
    
    timeline.setup();
 
    timeline.setFrameRate(60);
    timeline.setDurationInFrames(timeline.getFrameRate()*30);
    timeline.setLoopType(OF_LOOP_NORMAL);
    
    nodeTrack = new ofxTLNodeTrack();
    nodeTrack->setNode(tcpNode);
    nodeTrack->setTimeline(&timeline);
    nodeTrack->setXMLFileName(dirName+"/_keyframes.xml");
    timeline.addTrack("TargetTCP", nodeTrack);
    timeline.setWorkingFolder(dirName);
    timeline.setFrameBased(false);
    
    nodeTrack->lockNodeToTrack = true;
}

<<<<<<< HEAD
void ofApp::setupGUI(){
    
    panel.setup(parameters.robotArmParams);
    panel.add(parameters.pathRecorderParams);
    
    panelJoints.setup(parameters.joints);
    panelTargetJoints.setup(parameters.targetJoints);
    panelJointsSpeed.setup(parameters.jointSpeeds);
    panelJointsIK.setup(parameters.jointsIK);
    
    panel.add(robot.movement.movementParams);
    speeds.assign(6, 0);
    parameters.bMove = false;
    // get the current pose on start up
    parameters.bCopy = true;
    panel.loadFromFile("settings/settings.xml");
}

void ofApp::positionGUI(){
    viewportReal.height -= (panelJoints.getHeight());
    viewportReal.y +=(panelJoints.getHeight());
    panel.setPosition(viewportReal.x+viewportReal.width, 10);
    panelJointsSpeed.setPosition(viewportReal.x, 10);
    panelJointsIK.setPosition(panelJointsSpeed.getPosition().x+panelJoints.getWidth(), 10);
    panelTargetJoints.setPosition(panelJointsIK.getPosition().x+panelJoints.getWidth(), 10);
    panelJoints.setPosition(panelTargetJoints.getPosition().x+panelJoints.getWidth(), 10);
    timeline.setOffset(ofVec2f(viewportSim.x, viewportSim.y+viewportSim.height));
    timeline.setWidth(viewportReal.width+viewportSim.width);
    timeline.setHeight(2*(ofGetWindowHeight()-viewportSim.height)/3);
=======
    handleViewportPresets('p');

>>>>>>> parent of 31eef24... Drawing on tracked surface
}

//--------------------------------------------------------------
void ofApp::update(){
#ifdef ENABLE_NATNET
    natNet.update();
#endif
    //    workSurface.update();
    
    //    workSurfaceTargetTCP = workSurface.getNextPose();
    if(nodeTrack->lockNodeToTrack && !parameters.bCopy){
        gizmo.setNode(tcpNode);
    }else{
        tcpNode.setTransformMatrix( gizmo.getMatrix() );
    }
    moveArm();
    robot.update();
    
<<<<<<< HEAD
    if (viewportReal.inside(ofGetMouseX(), ofGetMouseY()))
    {
        activeCam = 0;
        if(!cams[0]->getMouseInputEnabled()){
            cams[0]->enableMouseInput();
        }
        if(cams[1]->getMouseInputEnabled()){
            cams[1]->disableMouseInput();
        }
        
=======
    toolPoint = robot.getToolPoint();
    targetPointAngles = robot.model.getToolPointMatrix().getEuler();    // is this the right TCP orientation?
    
    
    /* testing hard coded orientations */
//    targetPoint.rotation = ofQuaternion(.707,   0,  0,  .707);  //  90¼ about X-Axis
//    targetPoint.rotation = ofQuaternion(0,  .707,   0,  .707);  //  90¼ about Y-Axis
//    targetPoint.rotation = ofQuaternion(0,  0,  .707,   .707);  //  90¼ about Z-Axis
//    targetPoint.rotation = ofQuaternion(-.707,   0,  0,  .707); // -90¼ about X-Axis
//    targetPoint.rotation = ofQuaternion(0,  -.707,   0,  .707); // -90¼ about Y-Axis
//    targetPoint.rotation = ofQuaternion(0,  0,  -.707,   .707); // -90¼ about Z-Axis
//    targetOrientation = ofVec4f(targetPoint.rotation.x(), targetPoint.rotation.y(), targetPoint.rotation.z(), targetPoint.rotation.w());
    

    
    if(bCopy){
        bCopy = false;
        
        targetPoint.position = toolPoint;
        targetPoint.rotation = robot.model.getToolPointMatrix();
>>>>>>> parent of 31eef24... Drawing on tracked surface
        
      
    }
    if(viewportSim.inside(ofGetMouseX(), ofGetMouseY()))
    {
        activeCam = 1;
        if(!cams[1]->getMouseInputEnabled()){
            cams[1]->enableMouseInput();
        }
        if(cams[0]->getMouseInputEnabled()){
            cams[0]->disableMouseInput();
        }
        if(gizmo.isInteracting() && cams[1]->getMouseInputEnabled()){
            cams[1]->disableMouseInput();
        }
    }
}

void ofApp::moveArm(){
    
    // assign the target pose to the current robot pose
    if(parameters.bCopy){
        parameters.bCopy = false;
        parameters.targetTCP.rotation = ofQuaternion(90, ofVec3f(0, 0, 1));
        parameters.targetTCP.rotation*=ofQuaternion(90, ofVec3f(1, 0, 0));
        
<<<<<<< HEAD
        // get the robot's position
        parameters.targetTCP.position = parameters.actualTCP.position;
        parameters.targetTCP.rotation*=parameters.actualTCP.rotation;
        
        tcpNode.setPosition(parameters.targetTCP.position*1000);
        tcpNode.setOrientation(parameters.targetTCP.rotation);
        gizmo.setNode(tcpNode);
        // update GUI params
        parameters.targetTCPPosition = parameters.targetTCP.position;
        parameters.targetTCPOrientation = ofVec4f(parameters.targetTCP.rotation.x(), parameters.targetTCP.rotation.y(), parameters.targetTCP.rotation.z(), parameters.targetTCP.rotation.w());
=======
        // or slerp from current to target orientation (???)
        targetPoint.rotation.slerp(.1, targetPoint.rotation, ofQuaternion(targetOrientation));
        
        // or just reassign the value (???)
//        targetPoint.rotation = ofQuaternion(targetOrientation);

    }else if(bTrace){
        Joint jTCP = workSurface.getTargetPoint(ofGetElapsedTimef()-tagStartTime);
        targetPoint.position = jTCP.position;
        targetPoint.rotation *= jTCP.rotation;
        targetPointPos = targetPoint.position;
//        targetPoint.rotation *= ofQuaternion(90, ofVec3f(0, 0, 1));
    }else if(bFigure8){
        targetPoint.rotation = ofQuaternion(90, ofVec3f(0, 0, 1));
        targetPoint.rotation*=ofQuaternion(90, ofVec3f(1, 0, 0));
        targetPointAngles = targetPoint.rotation.getEuler();
        targetPoint.position.interpolate(targetPointPos.get()+ofVec3f(cos(ofGetElapsedTimef()*0.25)*0.2, 0, sin(ofGetElapsedTimef()*0.25*2)*0.2), 0.5);
>>>>>>> parent of 31eef24... Drawing on tracked surface
        
    }
    // follow a user-defined position and orientation
    if(parameters.bFollow){
        
        parameters.targetTCP.position.interpolate(tcpNode.getPosition()/1000.0, parameters.followLerp);
        parameters.targetTCP.rotation = tcpNode.getOrientationQuat();
        parameters.targetTCPOrientation = ofVec4f(parameters.targetTCP.rotation.x(), parameters.targetTCP.rotation.y(), parameters.targetTCP.rotation.z(), parameters.targetTCP.rotation.w());
        
    }
}


//--------------------------------------------------------------
void ofApp::draw(){
    ofEnableAlphaBlending();
    
    ofSetColor(255,160);
    ofDrawBitmapString("OF FPS "+ofToString(ofGetFrameRate()), 30, ofGetWindowHeight()-50);
    ofDrawBitmapString("Robot FPS "+ofToString(robot.robot.getThreadFPS()), 30, ofGetWindowHeight()-65);
    gizmo.setViewDimensions(viewportSim.width, viewportSim.height);

    
    cams[1]->begin(viewportSim);
    gizmo.draw( *cams[1] );
    robot.movement.draw(0);
    cams[1]->end();
    
    cams[0]->begin(viewportReal);
#ifdef ENABLE_NATNET
    natNet.draw();
#endif
    tcpNode.draw();
    
    if (!hideRobot){
        robot.robot.model.draw();
    }
    cams[0]->end();
    

    
    
    timeline.draw();
    
    panel.draw();
    panelJoints.draw();
    panelJointsIK.draw();
    panelWorkSurface.draw();
    panelJointsSpeed.draw();
    panelTargetJoints.draw();
    
    syphon.publishScreen();
}

void ofApp::exit(){
    parameters.bMove = false;
    panel.saveToFile("settings/settings.xml");
    robot.close();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    if(key == 'L'){
        nodeTrack->lockNodeToTrack = !nodeTrack->lockNodeToTrack;
    }
    else if(key == 'T'){
        nodeTrack->addKeyframe();
    }
    if(key == 'm'){
        parameters.bMove = !parameters.bMove;
    }
    
    if(key == '8'){
        parameters.bFigure8 = !parameters.bFigure8;
    }
    
    if( key == 'r' ) {
        gizmo.setType( ofxGizmo::OFX_GIZMO_ROTATE );
    }
    if( key == 'g' ) {
        gizmo.setType( ofxGizmo::OFX_GIZMO_MOVE );
    }
    if( key == 's' ) {
        gizmo.setType( ofxGizmo::OFX_GIZMO_SCALE );
    }
    if( key == 'e' ) {
        gizmo.toggleVisible();
    }
    
    if (key == 'h'){
        hideRobot = !hideRobot;
    }
#ifdef ENABLE_NATNET
    if (key == 'n'){
        natNet.record = !natNet.record;
    }
#endif
    
    handleViewportPresets(key);
}

//--------------------------------------------------------------
void ofApp::handleViewportPresets(int key){
    
    float dist = 2000;
    float zOffset = 450;
    
<<<<<<< HEAD
    if(activeCam != -1){
        // TOP VIEW
        if (key == '1'){
            cams[activeCam]->reset();
            cams[activeCam]->setPosition(0, 0, dist);
            cams[activeCam]->lookAt(ofVec3f(0, 0, 0), ofVec3f(0, 0, 1));
            //        cams[activeCam]->movedManually();
            viewportLabels[activeCam] = "TOP VIEW";
=======
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
    natnet.setDuplicatedPointRemovalDistance(20);
}

//--------------------------------------------------------------
void ofApp::updateNatNet(){
    natnet.update();
    
    // add a path trail to rigidBody
    if (natnet.getNumRigidBody()==1){
        const ofxNatNet::RigidBody &rb = natnet.getRigidBodyAt(0);
        
        if (record){
            toolpath.push_back(rb);
        
            // store previous 20 rigid bodies
            if (toolpath.size()>20)
                toolpath.erase(toolpath.begin());
>>>>>>> parent of 31eef24... Drawing on tracked surface
        }
        // LEFT VIEW
        else if (key == '2'){
            cams[activeCam]->reset();
            cams[activeCam]->setPosition(dist, 0, 0);
            cams[activeCam]->lookAt(ofVec3f(0, 0, 0), ofVec3f(0, 0, 1));
            //        cams[activeCam]->movedManually();
            viewportLabels[activeCam] = "LEFT VIEW";
        }
<<<<<<< HEAD
        // FRONT VIEW
        else if (key == '3'){
            cams[activeCam]->reset();
            cams[activeCam]->setPosition(0, dist, 0);
            cams[activeCam]->lookAt(ofVec3f(0, 0, 0), ofVec3f(0, 0, 1));
            //        cams[activeCam]->movedManually();
            viewportLabels[activeCam] = "FRONT VIEW";
        }
        // PERSPECTIVE VIEW
        else if (key == '4'){
            cams[activeCam]->reset();
            cams[activeCam]->setPosition(dist, dist, dist/4);
            cams[activeCam]->lookAt(ofVec3f(0, 0, 0), ofVec3f(0, 0, 1));
            //        cams[activeCam]->movedManually();
            viewportLabels[activeCam] = "PERSPECTIVE VIEW";
        }
=======
        
        // apply the difference to the corners of the worksurface
        for (int i=0; i<4; i++){
            auto v = workSurface.targetPoints[i].get() * ofVec3f(1000,1000,1000) * diff;
            workSurface.targetPoints[i] = v/1000;
        }
        
>>>>>>> parent of 31eef24... Drawing on tracked surface
    }
}


<<<<<<< HEAD
=======
    // show path
    ofPolyline tp;
    for (auto &path: toolpath){
        ofSetColor(ofColor::azure);
        tp.addVertex(path.getMatrix().getTranslation());
    }
    tp.draw();
    
    // show rigid body
    ofPolyline bodies;
    float alpha = 255;
    float step = 255 / (toolpath.size()+1);
    for (auto &rb: toolpath){
        ofSetColor(ofColor::navajoWhite, alpha);
        for (int i = 0; i < rb.markers.size(); i++)
            bodies.addVertex(rb.markers[i]);
        alpha -= step;
    }
    bodies.draw();
    
    
}
>>>>>>> parent of 31eef24... Drawing on tracked surface

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
    setupViewports();
    positionGUI();
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){
    
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){
    parameters.bMove = false;
    if(dragInfo.files.size() > 0){
        ofFile file;
        file.open(dragInfo.files[0]);
        if(file.isDirectory()){
            nodeTrack->clear();
            timeline.clear();
            ofDirectory dir;
            dir.listDir(dragInfo.files[0]);
            dir.allowExt(".xml");
            timeline.loadTracksFromFolder(dragInfo.files[0]);
            for(int i = 0; i < dir.size(); i++){
                if(dir.getName(i) == "_keyframes.xml"){
                    nodeTrack->loadFromXMLRepresentation(dir.getPath(i));
                    nodeTrack->setXMLFileName(dir.getPath(i));
                   
                }
            }
            timeline.setWorkingFolder(dragInfo.files[0]);
        }
    }
}
