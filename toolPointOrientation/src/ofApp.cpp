#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    orientations.setName("Orientation");
    q.makeRotate(0, ofVec3f(0, 0, 0));
    orientations.add(qVec4.set("Q Vec4", ofVec4f(q.x(), q.y(), q.z(), q.w()), ofVec4f(-1, -1, -1, -1), ofVec4f(1, 1, 1, 1)));
    orientations.add(nodePosition.set("Node Pos", ofVec3f(0, 0, 0), ofVec3f(-500, -500, -500), ofVec3f(500, 500, 500)));
    
    panel.setup(orientations);
    cam.lookAt(ofVec3f(0, 0, 0), ofVec3f(0, 0, 1));
    cam.begin();
    cam.end();
}

//--------------------------------------------------------------
void ofApp::update(){
    q = ofQuaternion(qVec4);
    node.setPosition(nodePosition);
    node.setOrientation(q);
    cam.lookAt(ofVec3f(0, 0, 0), ofVec3f(0, 0, 1));
}

//--------------------------------------------------------------
void ofApp::draw(){
    ofBackground(0, 0, 0);
    cam.begin();
    ofDrawAxis(10);
//    node.draw();
    
    ofPushMatrix();
    float angle;
    ofVec3f axis;
    q.getRotate(angle, axis);
    ofRotate(angle, axis.x, axis.y, axis.z);
    ofDrawAxis(10);
    ofPopMatrix();
    cam.end();
    
    panel.draw();
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
