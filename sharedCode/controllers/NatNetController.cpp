//
//  NatNetController.cpp
//  urModernDriverTest
//
//  Created by dantheman on 4/4/16.
//
//

#include "NatNetController.h"

NatNetController::NatNetController(){
    
}
NatNetController::~NatNetController(){
    
}
/////This needs to read from a file;
//void NatNetController::setup(string myIP, string serverIP){
//    natnet.setup(myIP, serverIP);  // interface name, server ip
//    natnet.setScale(1);
//    natnet.setDuplicatedPointRemovalDistance(.020);
//}
//
/////This needs to read from a file;
//void NatNetController::setup(string myIP, string serverIP, int scale){
//    natnet.setup(myIP, serverIP);  // interface name, server ip
//    natnet.setScale(scale);
//    natnet.setDuplicatedPointRemovalDistance(.020);
//}
//void NatNetController::update(){
//    natnet.update();
//    
//    if (natnet.getNumRigidBody() > 0 && !useUnlabledMarkers){
//        const ofxNatNet::RigidBody &rb = natnet.getRigidBodyAt(0);  // more than one rigid body crashes ofxNatNet now
//        
//        // add to the rigid body history
//        if (record){
//            recordedPath.push_back(rb);
//            
//            // store previous 20 rigid bodies
//            if (recordedPath.size()>20)
//                recordedPath.erase(recordedPath.begin());
//            
//            // check if the rigid body is moving
//            if (recordedPath.size() > 0){
//                float dist = .25;
//                ofVec3f curr = rb.matrix.getTranslation();
//                ofVec3f prev = recordedPath[recordedPath.size()-1].matrix.getTranslation();
//                isMoving = curr.squareDistance(prev) > dist*dist ;
//            }
//        }
//        
//        currentRigidBody = rb;
//        
//    }
//    else if (useUnlabledMarkers && natnet.getNumFilterdMarker() > 0){
//  
//        for (int i = 0; i < natnet.getNumFilterdMarker(); i++){
//            markers.push_back(natnet.getFilterdMarker(i));
//        }
//    }
//}
//
//vector<ofxNatNet::Marker> NatNetController::getCurrentMarkers(){
//    return markers;
//}
//
//ofxNatNet::RigidBody NatNetController::getCurrentRigidBody(){
//    if(recordedPath.size() > 0){
//        return currentRigidBody;
//    }
//    else{
//        return ofxNatNet::RigidBody();
//    }
//}
//
//void NatNetController::draw(){
//    
//    // show path
//    ofPolyline tp;
//    for (auto &path: recordedPath){
//        ofSetColor(ofColor::azure);
//        tp.addVertex(path.getMatrix().getTranslation());
//    }
//    tp.draw();
//    
//    // show rigid body
//    ofPolyline bodies;
//    float alpha = 255;
//    float step = 255 / (recordedPath.size()+1);
//    for (auto &rb: recordedPath){
//        ofSetColor(ofColor::navajoWhite, alpha);
//        for (int i = 0; i < rb.markers.size(); i++)
//            bodies.addVertex(rb.markers[i]);
//        alpha -= step;
//    }
//    bodies.draw();
//    
//    // show unlabled makers
//    ofSetColor(255,0,255);
//    for (int i = 0; i < natnet.getNumFilterdMarker(); i++){
//        ofDrawBox(natnet.getFilterdMarker(i), 15);
//    }
//
//    // show test srf
//    ofPushStyle();
//    ofSetLineWidth(5);
//    if (isMoving){
//        ofSetColor(255, 0, 0);
//    }else{
//        ofSetColor(ofColor::aqua);
//    }
//    rbWorksrf.draw();
//    ofPopStyle();
//}
//bool NatNetController::isFrameNew(){
//    
//}

    
