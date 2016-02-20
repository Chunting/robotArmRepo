#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    ofSetFrameRate(60);
    ofSetVerticalSync(true);
    ofBackground(0);
    
    string interface_name = "en0"; // or network interface name
    
#ifdef ENABLE_NATNET
    sender.setup("192.168.1.255", 7777);
    natnet.setup(interface_name, "192.168.1.107");  // interface name, server ip
    natnet.setScale(100);
    natnet.setDuplicatedPointRemovalDistance(20);
#endif
    
    robot = new UrDriver(rt_msg_cond_,
                         msg_cond_, "192.168.1.9");
    
    for(int i = 0; i < 6; i++){
        angles.push_back(ofVec3f());
        jointLength.push_back(ofVec3f());
        jointsQ.push_back(ofQuaternion());
        joints.push_back(ofNode());
        
    }
    
    joints[1].setParent(joints[0]);
    joints[2].setParent(joints[1]);
    joints[3].setParent(joints[2]);
    joints[4].setParent(joints[3]);
    joints[5].setParent(joints[4]);
    
    joints[0].setPosition(0, 0, 0);
    joints[1].setPosition(ofVec3f(0, -109.3, 89.2));
    joints[2].setPosition(ofVec3f(0, 0, 425));
    joints[3].setPosition(ofVec3f(0, 109.3, 392));
    joints[4].setPosition(ofVec3f(0, -109.3, 0));
    joints[5].setPosition(ofVec3f(0, -82.5, 94.75));
    
    
    
    angles[0].set(0, 0, 1);
    angles[1].set(1, 0, 0);
    angles[2].set(1, 0, 0);
    angles[3].set(1, 0, 0);
    angles[4].set(0, 0, 1);
    angles[5].set(0, 1, 0);
    
    jointLength[0].set(0, 0, 0);
    jointLength[1].set(10, 0, 0);
    jointLength[2].set(425, 0, 0);
    jointLength[3].set(392, 0, 0);
    jointLength[4].set(0, 109, 0);
    jointLength[5].set(0, 82, 0);
    

    
    
    

    
    
    
    char buf[256];
    vector<string> foo = robot->getJointNames();
    std::string joint_prefix = "";
    std::vector<std::string> joint_names;
    joint_prefix = "ur5-";
    joint_names.push_back(joint_prefix + "shoulder_pan_joint");
    joint_names.push_back(joint_prefix + "shoulder_lift_joint");
    joint_names.push_back(joint_prefix + "elbow_joint");
    joint_names.push_back(joint_prefix + "wrist_1_joint");
    joint_names.push_back(joint_prefix + "wrist_2_joint");
    joint_names.push_back(joint_prefix + "wrist_3_joint");
    robot->setJointNames(joint_names);
    
    //Bounds for SetPayload service
    //Using a very conservative value as it should be set through the parameter server
    double min_payload = 0.;
    double max_payload = 1.;
    robot->setMinPayload(min_payload);
    robot->setMaxPayload(max_payload);
    sprintf(buf, "Bounds for set_payload service calls: [%f, %f]",
            min_payload, max_payload);
    ofLog()<<buf;
    
    if( robot-> start()){
        ofLog()<<"SUCCESSSS"<<endl;
    }
    
    
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
    
    std::mutex msg_lock; // The values are locked for reading in the class, so just use a dummy mutex
    std::unique_lock<std::mutex> locker(msg_lock);
    while (!robot->rt_interface_->robot_state_->getControllerUpdated()) {
        rt_msg_cond_.wait(locker);
    }
    
    //    clock_gettime(CLOCK_MONOTONIC, &current_time);
    //    elapsed_time = ofGetElapsedTimef();
    //    last_time = ofGetElapsedTimef();
    jointsRaw = robot->rt_interface_->robot_state_->getQActual();

    for(int i = 0; i < joints.size(); i++){
        jointsRaw[i] = ofRadToDeg(jointsRaw[i]);
        if(i == 1 || i == 3){
            jointsRaw[i]+=90;
        }
        jointsQ[i].makeRotate(jointsRaw[i], angles[i]);
        joints[i].setOrientation(jointsQ[i]);
    }
    
    
    vector<double> foo = robot->rt_interface_->robot_state_->getToolVectorActual();
    tool.setPosition(ofVec3f(foo[0], foo[1], foo[2])*1000);
    tool.setOrientation(ofVec3f(ofRadToDeg(foo[3]), ofRadToDeg(foo[4]), ofRadToDeg(foo[5])));
    
    robot->rt_interface_->robot_state_->setControllerUpdated();
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
    
    cam.begin(ofRectangle(0, 0, 640, 480));
    ofEnableDepthTest();
    ofSetColor(255, 255, 255);
    tool.draw();
    joints[0].draw();
    for(int i = 1; i < joints.size(); i++){
        if(i == 0){
            ofSetColor(255, 0, 0);
        }
        if(i == 1){
            ofSetColor(0, 255, 255);
        }
        if(i == 2){
            ofSetColor(0, 0, 255);
        }
        if(i == 3){
            ofSetColor(0, 255, 0);
        }
        if(i == 4){
            ofSetColor(255, 255, 0);
        }
        if(i == 5){
            ofSetColor(255, 0, 255);
        }
        ofDrawLine(joints[i].getGlobalPosition(), joints[i-1].getGlobalPosition());
        joints[i].draw();
    }
    ofDisableDepthTest();
    cam.end();
    
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    
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
