#include "WorkSurface.h"
void WorkSurface::setup(){
    workSurfacePrarms.setName("Work Surface");
    workSurfacePrarms.add(position.set("WS Position", ofVec3f(0, 0, 0), ofVec3f(-1, -1, -1), ofVec3f(1, 1, 1)));
    workSurfacePrarms.add(size.set("WS size", ofVec3f(0, 0, 0), ofVec3f(-1, -1, -1), ofVec3f(1, 1, 1)));
    workSurfacePrarms.add(rotation.set("WS Euler", ofVec3f(0, 0, 0), ofVec3f(-360, -360, -360), ofVec3f(360, 360, 360)));
    
    for(int i = 0; i < 4; i++){
        targetPoints.push_back(ofParameter<ofPoint>());
        workSurfacePrarms.add(targetPoints.back().set("TP-"+ofToString(i), ofPoint(1/(i+1), 1/(i+1), 1/(i+1)), ofPoint(-1, -1, -1), ofPoint(1, 1, 1)));
    }
    warp.setup();
    corners.assign(4, ofPoint(0, 0, 0));
    
    plane.setPosition(0, 0, 0);
    plane.setWidth(1);
    plane.setHeight(1);
    
}
void WorkSurface::setCorners(vector<ofPoint> pts){
    corners = pts;
}
void WorkSurface::setCorner(CORNER i, ofPoint pt){
    switch (i) {
        case UL:
            corners[0] = pt;
            targetPoints[0] = corners[0];
            break;
        case UR:
            corners[1] = pt;
            targetPoints[1] = corners[1];
            break;
        case LL:
            corners[2] = pt;
            targetPoints[2] = corners[2];
            break;
        case LR:
            corners[3] = pt;
            targetPoints[3] = corners[3];
            break;
    }
}
void WorkSurface::update(){
    ofPoint diffOne = targetPoints[0].get() - targetPoints[1].get();
    ofPoint diffTwo = targetPoints[0].get() - targetPoints[2].get();
    
    position = targetPoints[0].get().getMiddle(targetPoints[2].get());
    diffOne.normalize();
    diffTwo.normalize();
    crossed = diffTwo.cross(diffOne);
    orientation.makeRotate(ofPoint(0, 0, 1), crossed);
    rotation = orientation.getEuler();
}
void WorkSurface::addPoint(ofVec3f pt){
    
}
void WorkSurface::addStroke(ofPolyline stroke){
    
}
void WorkSurface::addStrokes(vector<ofPolyline> strokes){
    ofPoint diffUpper = targetPoints[2].get() - targetPoints[0].get();
    ofPoint diffLower = targetPoints[3].get() - targetPoints[1].get();
    lines.clear();
    ofMatrix4x4 mat;
    mat.makeRotationMatrix(orientation);
    mat.setTranslation(targetPoints[3].get());
    for(int i = 0; i < strokes.size(); i++){
        ofPolyline fooLine;
        for(int j = 0; j < strokes[i].getVertices().size(); j++){
            fooLine.addVertex(strokes[i].getVertices()[j]*mat);
        }
        lines.push_back(fooLine);
    }
}

Joint WorkSurface::getTargetPoint(float t){
    Joint foo;
    if(lines.size() > 0){
        float sinTime = ofMap(sin(t * 0.5), -1, 1, 0, 1);
        float sinIndexLength = lines[0].getIndexAtPercent(sinTime);
        ofPoint p = lines[0].getPointAtIndexInterpolated(sinIndexLength);
        
        foo.position = p;
        foo.rotation = orientation;
    }
    return foo;
}
void WorkSurface::draw(){
    ofPushMatrix();
    ofTranslate(position.get()*1000);
    float angle;
    ofVec3f axis;
    orientation.getRotate(angle, axis);
    ofRotate(angle, axis.x, axis.y, axis.z);
    ofDrawAxis(100);
    ofPopMatrix();
    ofPushMatrix();
    ofScale(1000, 1000, 1000);
    for(int i = 0; i < lines.size(); i++){
        lines[i].draw();
    }
    ofPopMatrix();
    
    ofPushMatrix();
    ofSetColor(255, 0, 255);
    ofDrawSphere(targetPoints[0], 10);
    ofDrawLine(targetPoints[0].get()*1000,targetPoints[1].get()*1000);
    ofSetColor(255, 255, 0);
    ofDrawSphere(targetPoints[1], 10);
    ofDrawLine(targetPoints[1].get()*1000,targetPoints[2].get()*1000);
    ofSetColor(255, 0, 255);
    ofDrawSphere(targetPoints[2], 10);
    ofDrawLine(targetPoints[2].get()*1000,targetPoints[3].get()*1000);
    ofSetColor(255, 255, 0);
    ofDrawSphere(targetPoints[3], 10);
    ofDrawLine(targetPoints[3].get()*1000,targetPoints[0].get()*1000);
    ofPopMatrix();
}
void WorkSurface::setRotationX(float x){
    orientationX.makeRotate(x, 1, 0, 0);
}
void WorkSurface::setRotationY(float y){
    orientationY.makeRotate(y, 0, 1, 0);
}
void WorkSurface::setRotationZ(float z){
    orientationZ.makeRotate(z, 0, 0, 1);
}
void WorkSurface::setRotation(float x, float y, float z){
    orientationX.makeRotate(x, 1, 0, 0);
    orientationY.makeRotate(y, 0, 1, 0);
    orientationZ.makeRotate(z, 0, 0, 1);
}