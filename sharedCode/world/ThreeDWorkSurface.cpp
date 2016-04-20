#include "ThreeDWorkSurface.h"
ThreeDWorkSurface::ThreeDWorkSurface(){
    
}
ThreeDWorkSurface::~ThreeDWorkSurface(){
    
}

void ThreeDWorkSurface::setup(RobotParameters * params){
    parameters = params;
    workSurfaceParams.setName("3D Work Surface");
    workSurfaceParams.add(feedRate.set("3d feedRate", 0.001, 0.00001, 0.01));
    workSurfaceParams.add(position.set("3d WS Position", ofVec3f(0.2, 0.2, 0.0), ofVec3f(-1, -1, -1), ofVec3f(1, 1, 1)));
    workSurfaceParams.add(rotation.set("3d WS Euler", ofVec3f(0, 0, 0), ofVec3f(-360, -360, -360), ofVec3f(360, 360, 360)));
    workSurfaceParams.add(retractDistance.set("3d retractDistance", 4, 0, 100));
    workSurfaceParams.add(rotateDrawing.set("3d rotateDrawing", 0, 0, 360));
    workSurfaceParams.add(drawingScale.set("3d drawingScale", 1, 0, 2));
    workSurfaceParams.add(drawingOffset.set("3d drawingOffset", ofVec3f(0, 0, 0), ofVec3f(-100, -100, -100), ofVec3f(100, 100, 100)));
    
    ofxAssimpModelLoader loader;
    loader.loadModel(ofToDataPath("mesh_srf.stl"));
    surfaceMesh = loader.getMesh(0);
    position;
    // scale surface to meters
    for (auto &v : surfaceMesh.getVertices()){
        v /= 100;
        v+=position;
    }

}
void ThreeDWorkSurface::update(Joint currentTCP){
    this->currentTCP = currentTCP;
    
}
void ThreeDWorkSurface::draw(){
    ofPushMatrix();

    ofScale(1000, 1000, 1000);
    // show the surface
    ofSetColor(255, 0, 255, 100);
    surfaceMesh.draw();
    
    // show surface normals
    ofSetColor(ofColor::aqua,100);
    for (auto &face : surfaceMesh.getUniqueFaces()){
        ofVec3f n = face.getFaceNormal();
        n /= -100; // scale to meters & flip
        ofVec3f pos = (face.getVertex(0) + face.getVertex(1) + face.getVertex(2)) / 3;
        ofDrawLine(pos, pos+n);
    }
    
    // draw 2D & 3D toolpaths
    ofSetColor(255, 0, 255, 200);
    ofSetLineWidth(3);
    toolpath.draw();
        
    
    ofPopMatrix();
}

Joint ThreeDWorkSurface::getTargetPoint(float t){
    if(toolpath.getVertices().size() > 0){
        float length = toolpath.getLengthAtIndex(toolpath.getVertices().size()-1);
        float dist = feedRate*t;
        float indexInterpolated = toolpath.getIndexAtPercent(dist/length);
        
        ofPoint p = toolpath.getPointAtIndexInterpolated(indexInterpolated);
        orientation.slerp(0.5, orientation, toolpathOrients[(int)indexInterpolated]);
        orientation.slerp(0.5, orientation, toolpathOrients[(int)(indexInterpolated-1)%toolpathOrients.size()]);

        targetToolPoint.position = p;
        targetToolPoint.rotation = orientation;
    }
    return targetToolPoint;
    
}

ofQuaternion ThreeDWorkSurface::eulerToQuat(ofVec3f  rotationEuler) {
    ofQuaternion q;
    float c1 = cos(rotationEuler[2] * 0.5);
    float c2 = cos(rotationEuler[1] * 0.5);
    float c3 = cos(rotationEuler[0] * 0.5);
    float s1 = sin(rotationEuler[2] * 0.5);
    float s2 = sin(rotationEuler[1] * 0.5);
    float s3 = sin(rotationEuler[0] * 0.5);
    
    q[0] = c1*c2*s3 - s1*s2*c3;
    q[1] = c1*s2*c3 + s1*c2*s3;
    q[2] = s1*c2*c3 - c1*s2*s3;
    q[3] = c1*c2*c3 + s1*s2*s3;
    
    return q;
}

void ThreeDWorkSurface::buildToolpath(ofPolyline &path){
    
    path.clear();
    
    
    // make an XY circle as a toolpath ...
    
    float res = 60;
    float radius = .05;
    float theta = 360/res;
    
    for (int i=0; i<res; i++){
        ofPoint p = ofPoint(radius,0,0);
        p.rotate(theta*i, ofVec3f(0,0,1));
        p.x += .0;
        path.addVertex(p);
    }
    path.close();
    
}

void ThreeDWorkSurface::projectToolpath(ofMesh & mesh, vector<ofPolyline> &path2D, vector<ofPolyline> &path){
    path.clear();
    toolpath.clear();
    toolpath2D.clear();
    toolpathOrients.clear();
    for(auto &foolines : path2D){
        for (auto &v : foolines.getVertices()){
            toolpath2D.addVertex(v);
            // find the closest face to the 2D path point
            for (int i=0; i<mesh.getUniqueFaces().size(); i++){
                
                // re-make the face as a 2D polyline so we can check
                // if the toolpath point is inside ... hacky, but it works!
                ofPolyline f;
                f.addVertex(mesh.getFace(i).getVertex(0));
                f.addVertex(mesh.getFace(i).getVertex(1));
                f.addVertex(mesh.getFace(i).getVertex(2));
                f.close();
                f.getVertices()[0].z = 0;
                f.getVertices()[1].z = 0;
                f.getVertices()[2].z = 0;
                v.z = 0;
                // project the 2D point onto the mesh face
                if (f.inside(v)){
                    auto face = mesh.getFace(i);
                    
                    // find the distance between our toolpath point and the mesh face
                    ofVec3f facePos = (mesh.getFace(i).getVertex(0)+mesh.getFace(i).getVertex(1)+mesh.getFace(i).getVertex(2))/3;
                    ofVec3f face2toolPt = v - facePos;
                    float projectedDist = face2toolPt.dot(face.getFaceNormal().getNormalized());
                    
                    // use the distance as the length of a vertical projection vector
                    ofVec3f length = ofVec3f(0,0,-projectedDist);
                    ofVec3f projectedPt = v-length;
                    
                    // save the projected point and face normal
                    toolpath.addVertex(projectedPt);
                    ofQuaternion q;
                    
                    q.makeRotate(ofVec3f(0,0,1), face.getFaceNormal().getNormalized());
                    
                    toolpathOrients.push_back(q);
                    toolpathOrientation.addVertex(face.getFaceNormal().getNormalized());
                }
                
            }
        }
    }
    ofQuaternion q;

    toolpath.close();
    
}

//--------------------------------------------------------------
void ThreeDWorkSurface::projectToolpath(ofMesh & mesh, ofPolyline &path2D, ofPolyline &path){
    
    path.clear();
    toolpath.clear();
    toolpath2D.clear();
    toolpathOrients.clear();
    
    for (auto &v : path2D.getVertices()){
        ofVec3f fooV = v;
        fooV.z = 0;
        // find the closest face to the 2D path point
        for (int i=0; i<mesh.getUniqueFaces().size(); i++){
            
            // re-make the face as a 2D polyline so we can check
            // if the toolpath point is inside ... hacky, but it works!
            ofPolyline f;
            f.addVertex(mesh.getFace(i).getVertex(0));
            f.addVertex(mesh.getFace(i).getVertex(1));
            f.addVertex(mesh.getFace(i).getVertex(2));
            f.close();
            f.getVertices()[0].z = 0;
            f.getVertices()[1].z = 0;
            f.getVertices()[2].z = 0;
            
            // project the 2D point onto the mesh face
            if (f.inside(fooV)){
                auto face = mesh.getFace(i);
                
                // find the distance between our toolpath point and the mesh face
                ofVec3f facePos = (mesh.getFace(i).getVertex(0)+mesh.getFace(i).getVertex(1)+mesh.getFace(i).getVertex(2))/3;
                ofVec3f face2toolPt = v - facePos;
                float projectedDist = face2toolPt.dot(face.getFaceNormal().getNormalized());
                
                // use the distance as the length of a vertical projection vector
                ofVec3f length = ofVec3f(0,0,-projectedDist);
                ofVec3f projectedPt = v-length;
                
                // save the projected point and face normal
                toolpath.addVertex(projectedPt);
                ofQuaternion q;
                
                q.makeRotate(ofVec3f(0,0,1), face.getFaceNormal().getNormalized());
                
                toolpathOrients.push_back(q);
            }
            
        }
    }
    
    toolpath.close();
    
}



void ThreeDWorkSurface::addPoint(ofVec3f pt){
    
}
void ThreeDWorkSurface::addStroke(ofPolyline stroke){
    
}
void ThreeDWorkSurface::setCorners(vector<ofPoint> pts){
    
}
void ThreeDWorkSurface::setCorner(CORNER i, ofPoint pt){
    
}
void ThreeDWorkSurface::addStrokes(vector<ofPolyline> strokes, float retractDist){
//    if (strokes_original.size() == 0){
//        
//        // add a approach/retract point to the start and end of the path
//        for (auto &stroke : strokes){
//            auto first = ofVec3f(stroke.getVertices()[0]);
//            auto last = ofVec3f(stroke.getVertices()[stroke.getVertices().size()-1]);
//            
//            first.z += retractDist;
//            last.z  += retractDist;
//            
//            
//            stroke.insertVertex(first, 0);
//            stroke.addVertex(last);
//            stroke.addVertex(first);
//            
//        }
//        
//        
//        ofPolyline pl;
//        for (auto &stroke : strokes){
//            for (auto &v : stroke.getVertices()){
//                pl.addVertex(v);
//            }
//        }
//        strokes_original.push_back(pl);
//    }
    
   
    lines.clear();
    lines2D.clear();
    for(int i = 0; i < strokes.size(); i++){
        ofPolyline fooLine;
        for(int j = 0; j < strokes[i].getVertices().size(); j++){
            fooLine.addVertex(strokes[i].getVertices()[j]*drawingScale+position);
        }
        lines.push_back(fooLine);
    }
    projectToolpath(surfaceMesh, lines, lines2D);
    

}