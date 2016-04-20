//Copyright (c) 2016, Daniel Moore, Madaline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
<<<<<<< Updated upstream:sharedCode/world/TwoDWorkSurface.cpp
#include "TwoDWorkSurface.h"
void TwoDWorkSurface::setup(RobotParameters * parameters){
    workSurfaceParams.setName("2D Work Surface");
=======
#include "WorkSurface.h"
void WorkSurface::setup(){
    workSurfaceParams.setName("Work Surface");
>>>>>>> Stashed changes:sharedCode/calibration/WorkSurface.cpp
    workSurfaceParams.add(feedRate.set("feedRate", 0.001, 0.00001, 0.01));
    workSurfaceParams.add(position.set("WS Position", ofVec3f(0, 0, 0), ofVec3f(-1, -1, -1), ofVec3f(1, 1, 1)));
    workSurfaceParams.add(rotation.set("WS Euler", ofVec3f(0, 0, 0), ofVec3f(-360, -360, -360), ofVec3f(360, 360, 360)));
    workSurfaceParams.add(retractDistance.set("retractDistance", 4, 0, 100));
    workSurfaceParams.add(rotateDrawing.set("rotateDrawing", 0, 0, 360));
    workSurfaceParams.add(drawingScale.set("drawingScale", 1, 0, 2));
    workSurfaceParams.add(drawingOffset.set("drawingOffset", ofVec3f(0, 0, 0), ofVec3f(-100, -100, -100), ofVec3f(100, 100, 100)));
<<<<<<< Updated upstream:sharedCode/world/TwoDWorkSurface.cpp
    
=======

>>>>>>> Stashed changes:sharedCode/calibration/WorkSurface.cpp
    for(int i = 0; i < 4; i++){
        targetPoints.push_back(ofParameter<ofPoint>());
        workSurfaceParams.add(targetPoints.back().set("TP-"+ofToString(i), ofPoint(1/(i+1), 1/(i+1), 1/(i+1)), ofPoint(-1, -1, -1), ofPoint(1, 1, 1)));
    }
    
    corners.assign(4, ofPoint(0, 0, 0));
    
    plane.setPosition(0, 0, 0);
    plane.setWidth(1);
    plane.setHeight(1);
    targetIndex = 0;
    timer.setSmoothing(true);
<<<<<<< Updated upstream:sharedCode/world/TwoDWorkSurface.cpp
    
    this->parameters = parameters;
=======
>>>>>>> Stashed changes:sharedCode/calibration/WorkSurface.cpp
    
}
void TwoDWorkSurface::setCorners(vector<ofPoint> pts){
    corners = pts;
    
}
void TwoDWorkSurface::setCorner(CORNER i, ofPoint pt){
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

<<<<<<< Updated upstream:sharedCode/world/TwoDWorkSurface.cpp
void TwoDWorkSurface::update(Joint toolPointPos){
    timer.tick();
    // update the TwoDWorkSurface mesh
    if (surfaceMesh.getVertices().size() == 0){
=======
void WorkSurface::update(ofVec3f toolPointPos){
    timer.tick();
    // update the worksurface mesh
    if (mesh.getVertices().size() == 0){
>>>>>>> Stashed changes:sharedCode/calibration/WorkSurface.cpp
        for (int i=0; i<targetPoints.size(); i++){
            surfaceMesh.addVertex(targetPoints[i].get());
        }
        surfaceMesh.addTriangle(0, 1, 2);
        surfaceMesh.addTriangle(0, 2, 3);
    }
    else{
        surfaceMesh.setVertex(0, targetPoints[0]);
        surfaceMesh.setVertex(1, targetPoints[1]);
        surfaceMesh.setVertex(2, targetPoints[2]);
        surfaceMesh.setVertex(3, targetPoints[3]);
    }
    
    calcNormals();
    
    
    
    ////      USE RIGID BODY FROM NATNET AS TwoDWorkSurface
    ////     update the mesh normal as the average of its two face normals
    //    ofVec3f n = ((mesh.getFace(0).getFaceNormal() + mesh.getFace(1).getFaceNormal())/2).normalize();
    //
    //    orientation.set(n);
    //
    ////    // realign the local axis of the TwoDWorkSurface
    ////    //      this is a bit hacky ... I don't think it works for everything
    //    ofQuaternion conj = orientation.conj();
    ////    orientation *= conj;
    ////    orientation.makeRotate(-45, 0, 0, -1);
    ////    orientation *= conj.conj();
    ////
    ////    conj = orientation.conj();
    //    orientation *= conj;
    //    orientation.makeRotate(-90, 1, 0, 0);
    //    orientation *= conj.conj();
    
    
    //    ofPoint diffOne = targetPoints[0].get() - targetPoints[1].get();
    //    ofPoint diffTwo = targetPoints[0].get() - targetPoints[3].get();
    //
    //    position = targetPoints[2].get().getMiddle(targetPoints[0].get());
    //    diffOne.normalize();
    //    crossed = diffOne.cross(diffTwo);
    //    crossed.normalize();
    //    orientation.makeRotate(ofPoint(0, 0, 1), crossed);
    
    // assign new orientation
    ofVec3f axis;
    float angle;
    orientation.makeRotate(ofVec3f(0, 0, 1), normal);
    toolPoint.setPosition(toolPointPos.position);
    toolPoint.setOrientation(orientation);
<<<<<<< Updated upstream:sharedCode/world/TwoDWorkSurface.cpp

=======
    
    
    
>>>>>>> Stashed changes:sharedCode/calibration/WorkSurface.cpp
    rotation = orientation.getEuler();

    
    // update the position
    ofVec3f centroid;
    for (auto &p : targetPoints){
        centroid += p;
    }
    position = centroid/4;
    if (strokes_original.size() != 0)
        addStrokes(strokes_original, 4);
    
}

<<<<<<< Updated upstream:sharedCode/world/TwoDWorkSurface.cpp
void TwoDWorkSurface::calcNormals(bool flip){
    surfaceMesh.clearNormals();
    for( int i=0; i < surfaceMesh.getVertices().size(); i++ ){
       surfaceMesh.addNormal(ofPoint(0,0,0));
    }
    
    for( int i=0; i < surfaceMesh.getIndices().size(); i+=3 ){
        const int ia = surfaceMesh.getIndices()[i];
        const int ib = surfaceMesh.getIndices()[i+1];
        const int ic = surfaceMesh.getIndices()[i+2];
        ofVec3f e1, e2;
        if(flip){
            e1 = surfaceMesh.getVertices()[ib] - surfaceMesh.getVertices()[ia];
            e2 = surfaceMesh.getVertices()[ib] - surfaceMesh.getVertices()[ic];
        }else{
            e1 = surfaceMesh.getVertices()[ia] - surfaceMesh.getVertices()[ib];
            e2 = surfaceMesh.getVertices()[ic] - surfaceMesh.getVertices()[ib];
=======
void WorkSurface::calcNormals(bool flip){
    mesh.clearNormals();
    for( int i=0; i < mesh.getVertices().size(); i++ ) mesh.addNormal(ofPoint(0,0,0));
    
    for( int i=0; i < mesh.getIndices().size(); i+=3 ){
        const int ia = mesh.getIndices()[i];
        const int ib = mesh.getIndices()[i+1];
        const int ic = mesh.getIndices()[i+2];
        ofVec3f e1, e2;
        if(flip){
            e1 = mesh.getVertices()[ib] - mesh.getVertices()[ia];
            e2 = mesh.getVertices()[ib] - mesh.getVertices()[ic];
        }else{
            e1 = mesh.getVertices()[ia] - mesh.getVertices()[ib];
            e2 = mesh.getVertices()[ic] - mesh.getVertices()[ib];
>>>>>>> Stashed changes:sharedCode/calibration/WorkSurface.cpp
        }
        ofVec3f no = e1.cross( e2 );
        
        // depending on your clockwise / winding order, you might want to reverse the e2 / e1 above if your normals are flipped.
        
        surfaceMesh.getNormals()[ia] += no;
        surfaceMesh.getNormals()[ib] += no;
        surfaceMesh.getNormals()[ic] += no;
    }
    
    
<<<<<<< Updated upstream:sharedCode/world/TwoDWorkSurface.cpp
    for(int i=0; i < surfaceMesh.getNormals().size(); i++ ) {
        surfaceMesh.getNormals()[i].normalize();
        normal+=surfaceMesh.getNormals()[i];
=======
    for(int i=0; i < mesh.getNormals().size(); i++ ) {
        mesh.getNormals()[i].normalize();
        normal+=mesh.getNormals()[i];
>>>>>>> Stashed changes:sharedCode/calibration/WorkSurface.cpp
        normal/=2.0;
    }
    
}



void TwoDWorkSurface::addPoint(ofVec3f pt){
    
}
void TwoDWorkSurface::addStroke(ofPolyline stroke){
    
}
void TwoDWorkSurface::addStrokes(vector<ofPolyline> strokes, float retractDist){
    
    // add retract/approach points & store the original linework
    if (strokes_original.size() == 0){
        
        // add a approach/retract point to the start and end of the path
        for (auto &stroke : strokes){
            auto first = ofVec3f(stroke.getVertices()[0]);
            auto last = ofVec3f(stroke.getVertices()[stroke.getVertices().size()-1]);
            
            first.z += retractDist;
            last.z  += retractDist;
            
            
            stroke.insertVertex(first, 0);
            stroke.addVertex(last);
            stroke.addVertex(first);
            
        }
        
        
        ofPolyline pl;
        for (auto &stroke : strokes){
            for (auto &v : stroke.getVertices())
                pl.addVertex(v);
        }
        strokes_original.push_back(pl);
    }
    
    
    // get the centroid of the line drawing
    ofVec3f centroid;
    for (auto &pl : strokes){
        centroid += pl.getCentroid2D();
    }
    centroid /= strokes.size();
    
    
    lines.clear();
    ofMatrix4x4 mat;
    mat.setRotate(orientation);
    mat.setTranslation(position.get()-drawingOffset.get());
    for(int i = 0; i < strokes.size(); i++){
        ofPolyline fooLine;
        for(int j = 0; j < strokes[i].getVertices().size(); j++){
            // center stroke on canvas
            //            strokes[i].getVertices()[j] -= centroid;
            
            fooLine.addVertex(strokes[i].getVertices()[j]*drawingScale*mat);
        }
        lines.push_back(fooLine);
    }
}

<<<<<<< Updated upstream:sharedCode/world/TwoDWorkSurface.cpp
Joint TwoDWorkSurface::getTargetPoint(float t){
    
    if(lines.size() > 0){
        float length = lines[targetIndex].getLengthAtIndex(lines[targetIndex].getVertices().size()-1);
        float dist = feedRate*t;
        float indexInterpolated = lines[targetIndex].getIndexAtPercent(dist/length);
        
=======
Joint WorkSurface::getTargetPoint(float t){
  
    if(lines.size() > 0){
        float length = lines[targetIndex].getLengthAtIndex(lines[targetIndex].getVertices().size()-1);
        float dist = feedRate*t;
        cout<<dist/length<<endl;
        float indexInterpolated = lines[targetIndex].getIndexAtPercent(dist/length);
        

>>>>>>> Stashed changes:sharedCode/calibration/WorkSurface.cpp
        ofPoint p = lines[targetIndex].getPointAtIndexInterpolated(indexInterpolated);
        
        targetToolPoint.position = p;
        targetToolPoint.rotation = orientation;
<<<<<<< Updated upstream:sharedCode/world/TwoDWorkSurface.cpp
        
=======

>>>>>>> Stashed changes:sharedCode/calibration/WorkSurface.cpp
        if(indexInterpolated > lines[targetIndex].getVertices().size()-1){
            targetIndex++;
        }
        if(targetIndex > lines.size()-1.){
            targetIndex = 0;
        }
        
    }
    return targetToolPoint;
}
void TwoDWorkSurface::draw(){
    ofPushMatrix();
    ofTranslate(position.get()*1000);
    float angle;
    ofVec3f axis;
    orientation.getRotate(angle, axis);
    ofRotate(angle, axis.x, axis.y, axis.z);
    ofDrawAxis(100);
    toolPoint.draw();
    ofPopMatrix();
    ofPushMatrix();
    ofScale(1000, 1000, 1000);
    for(int i = 0; i < lines.size(); i++){
        lines[i].draw();
    }
    surfaceMesh.drawWireframe();
    ofPopMatrix();
    
    
    
    ofPushMatrix();
    {
        ofSetColor(255, 0, 255);
        ofDrawSphere(targetPoints[0].get()*1000, 10);
        ofDrawLine(targetPoints[0].get()*1000,targetPoints[1].get()*1000);
        ofSetColor(255, 255, 0);
        ofDrawSphere(targetPoints[1].get()*1000, 10);
        ofDrawLine(targetPoints[1].get()*1000,targetPoints[2].get()*1000);
        ofSetColor(255, 0, 255);
        ofDrawSphere(targetPoints[2].get()*1000, 10);
        ofDrawLine(targetPoints[2].get()*1000,targetPoints[3].get()*1000);
        ofSetColor(255, 255, 0);
        ofDrawSphere(targetPoints[3].get()*1000, 10);
        ofDrawLine(targetPoints[3].get()*1000,targetPoints[0].get()*1000);
    }
    ofPopMatrix();
}
