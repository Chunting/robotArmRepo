#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    ofSetFrameRate(120);
    ofSetVerticalSync(true);
    ofBackground(0);
    
    string interface_name = "en0"; // or network interface name
    
#ifdef ENABLE_NATNET
    sender.setup("192.168.1.255", 7777);
    natnet.setup(interface_name, "192.168.1.107");  // interface name, server ip
    natnet.setScale(100);
    natnet.setDuplicatedPointRemovalDistance(20);
#endif
    robot.setup("192.168.1.9",0, 1);
    robot.start();
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
    ofSetColor(255, 0, 255);
    ofDrawBitmapString("GUI FPS "+ofToString(ofGetFrameRate()), 10, 20);
    ofDrawBitmapString("Robot FPS "+ofToString(robot.getThreadFPS()), 10, 40);
    
    cam.begin();
    ofEnableDepthTest();
    robot.model.draw();
    ofDisableDepthTest();
    cam.end();
}

void ofApp::exit(){
    robot.disconnect();
    robot.waitForThread();
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
