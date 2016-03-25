#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    ofSetFrameRate(120);
    ofSetVerticalSync(true);
    ofBackground(0);
    ofSetLogLevel(OF_LOG_SILENT);
    
    string interface_name = "en0"; // or network interface name
    
    //    targetPoint.setPosition(0, 0, 0);
    parent.setPosition(0, 0, 0);
    //    targetPoint.setParent(parent);
    robotArmParams.add(targetPointPos.set("Target Point POS", ofVec3f(0, 0, 0), ofVec3f(-1, -1, -1), ofVec3f(1, 1, 1)));
    robotArmParams.add(targetPointAngles.set("Target Point Angles", ofVec3f(0, 0, 0), ofVec3f(-TWO_PI, -TWO_PI, -TWO_PI), ofVec3f(TWO_PI, TWO_PI, TWO_PI)));
    robotArmParams.add(toolPoint.set("ToolPoint", ofVec3f(0, 0, 0), ofVec3f(-1, -1, -1), ofVec3f(1, 1, 1)));
    
    panel.setup(robotArmParams);
    panel.add(bFollow.set("Follow TargetPoint", false));
    panel.add(bTrace.set("bTrace GML", false));
    panel.add(bCopy.set("bCopy Tool Point", false));
    workSurface.setup();

    panel.loadFromFile("settings.xml");
    panel.setPosition(10, 10);
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
    panelWorkSurface.setup(workSurface.workSurfacePrarms);
    panelWorkSurface.setPosition(panel.getWidth()+10, 10);
    panelWorkSurface.loadFromFile("worksurface.xml");
    
#ifdef ENABLE_NATNET
    sender.setup("192.168.1.255", 7777);
    natnet.setup(interface_name, "192.168.1.107");  // interface name, server ip
    natnet.setScale(100);
    natnet.setDuplicatedPointRemovalDistance(20);
#endif
    robot.setup("192.168.1.9",0, 1);
    robot.start();
    movement.setup();
    speeds.assign(6, 0);
    bMove = false;
    

    
    gml.loadFile("gml/53520.gml", 0, 0, 640, 480);
    
    /* 3D Navigation */
//    cams[1] = &movement.cam;
    // need to move URMove camera to ofApp

    handleViewportPresets('p');

}

//--------------------------------------------------------------
void ofApp::update(){
#ifdef ENABLE_NATNET
    natnet.update();
    ofxOscBundle bundle;
    int count = 0;
    for (int i = 0; i < max(0, (int)natnet.getNumMarkersSet() - 1); i++) {
        for (int j = 0; j < natnet.getMarkersSetAt(i).size(); j++) {
            ofxOscMessage b;
            b.setAddress("/natnet/marker");
            b.addInt32Arg(j);
            b.addFloatArg(natnet.getMarkersSetAt(i)[j].x);
            b.addFloatArg(natnet.getMarkersSetAt(i)[j].y);
            b.addFloatArg(natnet.getMarkersSetAt(i)[j].z);
            bundle.addMessage(b);
            count++;
        }
    }
    
    for (int i = 0; i < natnet.getNumRigidBody(); i++) {
        const ofxNatNet::RigidBody &RB = natnet.getRigidBodyAt(i);
        ofxOscMessage m;
        m.setAddress("/natnet/rigidbody");
        m.addInt32Arg(i);
        for(int j = 0; j < RB.markers.size(); j++){
            m.addFloatArg(RB.markers[j].x);
            m.addFloatArg(RB.markers[j].y);
            m.addFloatArg(RB.markers[j].z);
        }
        count++;
        bundle.addMessage(m);
    }
    if(count > 0){
        sender.sendBundle(bundle);
    }
#endif
    
    
    
    //    float angle = sqrt(pow(targetPointAngles.get().x, 2)+pow(targetPointAngles.get().y, 2)+pow(targetPointAngles.get().z, 2));
    //    if( angle < 0.00000000001){
    //        targetPoint.setOrientation(ofQuaternion(0, 0, 0, 1));
    //    }else{
    //        targetPoint.setOrientation(ofQuaternion(angle, ofVec3f(targetPointAngles.get().x/angle, targetPointAngles.get().y/angle, targetPointAngles.get().z/angle)));
    //    }
    workSurface.update();
    vector<double> currentJointPos = robot.getJointPositions();
    movement.setCurrentJointPosition(currentJointPos);
    
    for(int i = 0; i < currentJointPos.size(); i++){
        jointPos[i] = (float)currentJointPos[i];
    }
    
    toolPoint = robot.getToolPoint();
    targetPoint.rotation = ofQuaternion(90, ofVec3f(0, 0, 1));
    targetPoint.rotation*=ofQuaternion(90, ofVec3f(1, 0, 0));
    targetPoint.rotation*=ofQuaternion(0, ofVec3f(0,1, 0));
    targetPointAngles = targetPoint.rotation.getEuler();
    if(bCopy){
        bCopy = false;
        targetPoint.position = toolPoint;
        targetPointPos = toolPoint;
    }else if(bFollow){
        targetPoint.position.interpolate(targetPointPos.get(), 0.1);

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
        
    }
    
    // update Joint 'targetPoint' to modify the robot's pos & orient

    movement.addTargetPoint(targetPoint);
    movement.update();
    
    vector<double> target = movement.getTargetJointPos();
    for(int i = 0; i < target.size(); i++){
        targetJointPos[i] = (float)target[i];
    }
    
    vector<double> tempSpeeds;
    tempSpeeds.assign(6, 0);
    
    tempSpeeds = movement.getCurrentSpeed();
    for(int i = 0; i < tempSpeeds.size(); i++){
        jointVelocities[i] = (float)tempSpeeds[i];
    }
    avgAccel = movement.getAcceleration();
    if(bMove){
        robot.setSpeed(tempSpeeds, avgAccel);
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
    if(!stop)
        foo[0] = 1.0;
    robot.setSpeed(foo, ofRandom(1, 100));
}

//--------------------------------------------------------------
void ofApp::draw(){
    ofEnableAlphaBlending();
    
    
    
#ifdef ENABLE_NATNET
    cam.begin();
    
    ofDrawAxis(100);
    
    ofFill();
    
    // draw all markers set
    ofSetColor(255, 128);
    for (int i = 0; i < max(0, (int)natnet.getNumMarkersSet() - 1); i++) {
        for (int j = 0; j < natnet.getMarkersSetAt(i).size(); j++) {
            ofDrawBox(natnet.getMarkersSetAt(i)[j], 3);
        }
    }
    
    // draw all markers
    ofSetColor(255, 30);
    for (int i = 0; i < natnet.getNumMarker(); i++) {
        ofDrawBox(natnet.getMarker(i), 3);
    }
    
    ofNoFill();
    
    // draw filtered markers
    ofSetColor(255);
    for (int i = 0; i < natnet.getNumFilterdMarker(); i++) {
        ofDrawBox(natnet.getFilterdMarker(i), 10);
    }
    
    // draw rigidbodies
    for (int i = 0; i < natnet.getNumRigidBody(); i++) {
        const ofxNatNet::RigidBody &RB = natnet.getRigidBodyAt(i);
        
        if (RB.isActive())
            ofSetColor(0, 255, 0);
        else
            ofSetColor(255, 0, 0);
        
        ofPushMatrix();
        glMultMatrixf(RB.getMatrix().getPtr());
        ofDrawAxis(30);
        ofPopMatrix();
        
        glBegin(GL_LINE_LOOP);
        for (int n = 0; n < RB.markers.size(); n++) {
            glVertex3fv(RB.markers[n].getPtr());
        }
        glEnd();
        
        for (int n = 0; n < RB.markers.size(); n++) {
            ofDrawBox(RB.markers[n], 5);
        }
    }
    
    // draw skeletons
    for (int j = 0;  j < natnet.getNumSkeleton(); j++) {
        const ofxNatNet::Skeleton &S = natnet.getSkeletonAt(j);
        ofSetColor(0, 0, 255);
        
        for (int i = 0; i < S.joints.size(); i++) {
            const ofxNatNet::RigidBody &RB = S.joints[i];
            ofPushMatrix();
            glMultMatrixf(RB.getMatrix().getPtr());
            ofDrawBox(5);
            ofPopMatrix();
        }
    }
    
    cam.end();
    
    string str;
    str += "frames: " + ofToString(natnet.getFrameNumber()) + "\n";
    str += "data rate: " + ofToString(natnet.getDataRate()) + "\n";
    str += string("connected: ") + (natnet.isConnected() ? "YES" : "NO") + "\n";
    str += "num markers set: " + ofToString(natnet.getNumMarkersSet()) + "\n";
    str += "num marker: " + ofToString(natnet.getNumMarker()) + "\n";
    str += "num filtered (non regidbodies) marker: " +
    ofToString(natnet.getNumFilterdMarker()) + "\n";
    str += "num rigidbody: " + ofToString(natnet.getNumRigidBody()) + "\n";
    str += "num skeleton: " + ofToString(natnet.getNumSkeleton()) + "\n";
    
    ofSetColor(255);
    ofDrawBitmapString(str, 10, 20);
#endif
    
    
    
    ofSetColor(255,160);
    ofDrawBitmapString("OF FPS "+ofToString(ofGetFrameRate()), 30, ofGetWindowHeight()-50);
    ofDrawBitmapString("Robot FPS "+ofToString(robot.getThreadFPS()), 30, ofGetWindowHeight()-65);
    cams[0].begin(ofRectangle(0, 0, ofGetWindowWidth()/2, ofGetWindowHeight()));
    robot.model.draw();
    ofSetColor(255, 0, 255);
    ofPushMatrix();
    ofSetColor(255, 0, 255, 200);
    ofDrawSphere(toolPoint.get()*ofVec3f(1000, 1000, 1000), 5);
    ofSetColor(255, 255, 0, 200);
    ofDrawSphere(targetPoint.position*ofVec3f(1000, 1000, 1000), 5);
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
    if(key == ' ' ){
        workSurface.addStrokes(gml.getPath(1.0));
        bTrace = true;
        tagStartTime = ofGetElapsedTimef();
    }
    if(key == '1'){
        workSurface.setCorner(WorkSurface::UL, toolPoint);
    }
    if(key == '2'){
        workSurface.setCorner(WorkSurface::UR, toolPoint);
    }
    if(key == '3'){
        workSurface.setCorner(WorkSurface::LL, toolPoint);
    }
    if(key == '4'){
        workSurface.setCorner(WorkSurface::LR, toolPoint);
    }
    if(key == '8'){
        bFigure8 = !bFigure8;
    }
    
    handleViewportPresets(key);
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
        ofMatrix4x4 perspective = ofMatrix4x4(0.937959, -0.322461, -0.127483, 0,
                                              0.226044, 0.289835,     0.93, 0,
                                              -0.262939, -0.901119, 0.344743, 0,
                                              -971.326, -1499.33,  1038.77, 1);
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
    ofDrawBitmapString(viewportLabels[1], ofGetWindowWidth()/2+30, ofGetWindowHeight()-30);
    
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
