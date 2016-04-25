//Copyright (c) 2016, Daniel Moore, Madaline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.

#include "ofApp.h"
#include "URUtils.h"

//--------------------------------------------------------------
void ofApp::setup(){
    ofSetFrameRate(60);
    ofSetVerticalSync(true);
    ofBackground(0);
    ofSetLogLevel(OF_LOG_NOTICE);
    
    string interface_name = "en0"; // or network interface name
    
    //    tcp.setPosition(0, 0, 0);
    parent.setPosition(0, 0, 0);
    //    tcp.setParent(parent);
    
    setupGUI();
#ifdef ENABLE_NATNET
    natNet.setup("en6", "192.168.1.131");
#endif
    
    
    robot.setup(parameters);
    
    panel.add(robot.movement.movementParams);
    speeds.assign(6, 0);
    parameters.bMove = false;
    // get the current pose on start up
    parameters.bCopy = true;
    panel.loadFromFile("settings.xml");
    panelWorkSurface.loadFromFile("settings.xml");
    
    gml.setup();
    gml.loadFile("gml/53514.gml");
    
    
    
    gizmo.setDisplayScale(1.0);

    
    tcpNode.setPosition(ofVec3f(0.5, 0.5, 0.5)*1000);
    tcpNode.setOrientation(parameters.targetTCP.rotation);
    
    //    if(gizmo.load("tcpNode.txt" )) {
    //        tcpNode.setTransformMatrix( gizmo.getMatrix() );
    //    }
    
    
    gizmo.setNode( tcpNode);
}

void ofApp::setupGUI(){
    parameters.setup();
    
    panel.setup(parameters.robotArmParams);
    panel.add(parameters.pathRecorderParams);
    panel.setPosition(10, 10);
    
    workSurface.setup(parameters);
    
    
    panelJoints.setup(parameters.joints);
    panelTargetJoints.setup(parameters.targetJoints);
    panelJointsSpeed.setup(parameters.jointSpeeds);
    panelJointsIK.setup(parameters.jointsIK);
    
    panelJoints.setPosition(ofGetWindowWidth()-panelJoints.getWidth()-10, 10);
    panelJointsIK.setPosition(panelJoints.getPosition().x-panelJoints.getWidth(), 10);
    panelTargetJoints.setPosition(panelJointsIK.getPosition().x-panelJoints.getWidth(), 10);
    panelJointsSpeed.setPosition(panelTargetJoints.getPosition().x-panelJoints.getWidth(), 10);
    panelWorkSurface.setup(workSurface.threeDSurface.workSurfaceParams);
    panelWorkSurface.setPosition(panel.getWidth()+10, 10);
    
    
    
    
    
}


//--------------------------------------------------------------
void ofApp::update(){
#ifdef ENABLE_NATNET
    natNet.update();
#endif
    workSurface.update();
    
    workSurfaceTargetTCP = workSurface.getNextPose();
    tcpNode.setTransformMatrix( gizmo.getMatrix() );
    moveArm();
    robot.update();
    
    if (ofGetMouseX() < ofGetWindowWidth()/N_CAMERAS)
    {
        activeCam = 0;
    }
    else
    {
        activeCam = 1;
    }
    
    
}

void ofApp::moveArm(){
    
    // assign the target pose to the current robot pose
    if(parameters.bCopy){
        parameters.bCopy = false;
        parameters.targetTCP.rotation = ofQuaternion(90, ofVec3f(0, 0, 1));
        parameters.targetTCP.rotation*=ofQuaternion(90, ofVec3f(1, 0, 0));
        
        // get the robot's position
        parameters.targetTCP.position = parameters.actualTCP.position;
        parameters.targetTCP.rotation*=parameters.actualTCP.rotation;
        
        tcpNode.setPosition(parameters.targetTCP.position*1000);
        tcpNode.setOrientation(parameters.actualTCP.rotation);
        // update GUI params
        parameters.targetTCPPosition = parameters.targetTCP.position;
        parameters.targetTCPOrientation = ofVec4f(parameters.targetTCP.rotation.x(), parameters.targetTCP.rotation.y(), parameters.targetTCP.rotation.z(), parameters.targetTCP.rotation.w());
        
        tcpNode.setOrientation(parameters.targetTCP.rotation);
    }
    // follow a user-defined position and orientation
    if(parameters.bFollow){
        
        parameters.targetTCP.position.interpolate(tcpNode.getPosition()/1000.0, parameters.followLerp);
        parameters.targetTCP.rotation = tcpNode.getOrientationQuat();
        parameters.targetTCPOrientation = ofVec4f(parameters.targetTCP.rotation.x(), parameters.targetTCP.rotation.y(), parameters.targetTCP.rotation.z(), parameters.targetTCP.rotation.w());
        
    }
    if(parameters.bTrace || parameters.b3DPath){
        // set target TCP to a default orientation, then modify

        parameters.targetTCP.rotation = ofQuaternion(90, ofVec3f(0, 0, 1));
        parameters.targetTCP.rotation*=ofQuaternion(90, ofVec3f(1, 0, 0));
        parameters.targetTCP.rotation *= workSurfaceTargetTCP.rotation;
        parameters.targetTCP.position = workSurfaceTargetTCP.position;
        
        parameters.targetTCPOrientation = ofVec4f(parameters.targetTCP.rotation.x(), parameters.targetTCP.rotation.y(), parameters.targetTCP.rotation.z(), parameters.targetTCP.rotation.w());
        
        
        tcpNode.setPosition(parameters.targetTCP.position*1000);
        tcpNode.setOrientation(parameters.targetTCP.rotation);
        gizmo.setNode(tcpNode);
        
        // update GUI params
        parameters.targetTCPPosition = parameters.targetTCP.position;
        
        parameters.targetTCPOrientation = ofVec4f(parameters.targetTCP.rotation.x(), parameters.targetTCP.rotation.y(), parameters.targetTCP.rotation.z(), parameters.targetTCP.rotation.w());
        
    }
    if(parameters.bFigure8){
        
        // use a preset orientation
        parameters.targetTCP.rotation = ofQuaternion(90, ofVec3f(0, 0, 1));
        parameters.targetTCP.rotation*=ofQuaternion(90, ofVec3f(1, 0, 0));
        
        // update the target position
        parameters.targetTCP.position.interpolate(parameters.targetTCPPosition.get()+ofVec3f(cos(ofGetElapsedTimef()*0.25)*0.2, 0, sin(ofGetElapsedTimef()*0.25*2)*0.2), 0.5);
    }
    
}


//--------------------------------------------------------------
void ofApp::draw(){
    ofEnableAlphaBlending();
    
    ofSetColor(255,160);
    ofDrawBitmapString("OF FPS "+ofToString(ofGetFrameRate()), 30, ofGetWindowHeight()-50);
    ofDrawBitmapString("Robot FPS "+ofToString(robot.robot.getThreadFPS()), 30, ofGetWindowHeight()-65);
    gizmo.setViewDimensions(ofGetWindowWidth()/2, ofGetWindowHeight());
    cams[0].begin(ofRectangle(0, 0, ofGetWindowWidth()/2, ofGetWindowHeight()));
#ifdef ENABLE_NATNET
    natNet.draw();
#endif
    tcpNode.draw();
    gizmo.draw( cams[0] );
    if (!hideRobot){
        robot.robot.model.draw();
    }
    ofSetColor(255, 0, 255);
    ofEnableDepthTest();
    workSurface.draw();
    ofDisableDepthTest();;
    
    cams[0].end();
    
    
    
    cams[1].begin(ofRectangle(ofGetWindowWidth()/2, 0, ofGetWindowWidth()/2, ofGetWindowHeight()));
    ofEnableDepthTest();
    workSurface.draw();
    for(int i = 0; i < 4; i++){
        robot.movement.draw(i);
    }
    ofDisableDepthTest();
    cams[1].end();
    
    
    
    ofPushMatrix();
    ofSetColor(255, 0, 255);
    gml.draw();
    ofPopMatrix();
    
    panel.draw();
    panelJoints.draw();
    panelJointsIK.draw();
    panelWorkSurface.draw();
    panelJointsSpeed.draw();
    panelTargetJoints.draw();
    
    /* 3D Navigation */
    hightlightViewports();
}

void ofApp::exit(){
    parameters.bMove = false;
    panel.saveToFile("settings.xml");
    panelWorkSurface.saveToFile("workSurface.xml");
    if(robot.robot.isThreadRunning()){
        robot.robot.disconnect();
    }
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    if(key == 'm'){
        parameters.bMove = !parameters.bMove;
    }
    if(key == ' '){
        if(parameters.bTrace){
            parameters.bTrace = false;
            parameters.bFollow = true;
        }else{
            vector<ofPolyline> strokes;
            float retract;
            if (natNet.recordedPath.size() > 0){
                // DEBUGGING....testing mocap tracking single point on worksrf
                ofPolyline temp;
                temp.addVertex(ofPoint (0,0,0));
                
                retract = 0;
                strokes.push_back(temp);
            }else{
                retract = -.1;
                strokes = gml.getPath(0.1);
            }
            
            for(int i = 0; i < strokes.size(); i++){
                strokes[i] = strokes[i].getResampledByCount(strokes[i].getVertices().size()*4.0);
            }
            
            workSurface.threeDSurface.addStrokes(strokes,retract);
            workSurface.twoDSurface.addStrokes(strokes,retract);
            parameters.bTrace = true;
            parameters.bFollow = false;
            workSurface.startTime = ofGetElapsedTimef();
        }
        
    }
    if(key == 'u'){
        workSurface.twoDSurface.setCorner(WorkSurface::UL, parameters.tcpPosition);
    }
    if(key == 'i'){
        workSurface.twoDSurface.setCorner(WorkSurface::UR, parameters.tcpPosition);
    }
    if(key == 'o'){
        workSurface.twoDSurface.setCorner(WorkSurface::LL, parameters.tcpPosition);
    }
    if(key == 'p'){
        workSurface.twoDSurface.setCorner(WorkSurface::LR, parameters.tcpPosition);
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
    
    
    
    handleViewportPresets(key);
    
    if (key == 'h'){
        hideRobot = !hideRobot;
    }
    
    if (key == 'n'){
        natNet.record = !natNet.record;
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
        //        cams[activeCam].movedManually();
        viewportLabels[activeCam] = "TOP VIEW";
    }
    // LEFT VIEW
    else if (key == '2'){
        cams[activeCam].reset();
        cams[activeCam].setPosition(dist, 0, 0);
        cams[activeCam].lookAt(ofVec3f(0, 0, 0), ofVec3f(0, 0, 1));
        //        cams[activeCam].movedManually();
        viewportLabels[activeCam] = "LEFT VIEW";
    }
    // FRONT VIEW
    else if (key == '3'){
        cams[activeCam].reset();
        cams[activeCam].setPosition(0, dist, 0);
        cams[activeCam].lookAt(ofVec3f(0, 0, 0), ofVec3f(0, 0, 1));
        //        cams[activeCam].movedManually();
        viewportLabels[activeCam] = "FRONT VIEW";
    }
    // PERSPECTIVE VIEW
    else if (key == '4'){
        cams[activeCam].reset();
        cams[activeCam].setPosition(dist, dist, dist/4);
        cams[activeCam].lookAt(ofVec3f(0, 0, 0), ofVec3f(0, 0, 1));
        //        cams[activeCam].movedManually();
        viewportLabels[activeCam] = "PERSPECTIVE VIEW";
    }
    else if (key == '6'){
        cams[activeCam].reset();
        cams[activeCam].setPosition(0, 0, -dist);
        cams[activeCam].lookAt(ofVec3f(0, 0, 0), ofVec3f(0, 0, 1));
        //        cams[activeCam].movedManually();
        viewportLabels[activeCam] = "PERSPECTIVE VIEW";
    }
    if(key == '5'){
        //        cams[activeCam].usemouse = true;
    }
    //    // CUSTOM  VIEW
    //    else if (key == '5'){
    //        cams[activeCam].reset();
    //        cams[activeCam].setGlobalPosition(savedCamMats[activeCam].getTranslation());
    //        cams[activeCam].setGlobalOrientation(savedCamMats[activeCam].getRotate());
    //        viewportLabels[activeCam] = "SAVED VIEW";
    //    }
    //    // Record custom view port
    //    if (key == '+'){
    //        savedCamMats[activeCam] = cams[activeCam].getGlobalTransformMatrix();
    //        viewportLabels[activeCam] = "New Viewport Saved!";
    //        cout << ofToString(savedCamMats[activeCam]) << endl;
    //    }
    if(key == 'z'){
        cams[0].disableMouseInput();
        cams[1].disableMouseInput();
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
void ofApp::keyReleased(int key){
    if(key == '5'){
        //        cams[activeCam].usemouse = false;
    }
    if(key == 'z'){
        cams[0].enableMouseInput();
        cams[1].enableMouseInput();
    }
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
