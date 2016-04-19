#include "ThreeDWorkSurface.h"
ThreeDWorkSurface::ThreeDWorkSurface(){
    
}
ThreeDWorkSurface::~ThreeDWorkSurface(){
    
}

void ThreeDWorkSurface::setup(RobotParameters * params){
    parameters = params;
    
    ofxAssimpModelLoader loader;
    loader.loadModel(ofToDataPath("mesh_srf.stl"));
    surfaceMesh = loader.getMesh(0);
    
    // scale surface to meters
    for (auto &v : surfaceMesh.getVertices()){
        v /= 100;
    }
    
    // move toolpath surface to be in a more reachable position (temp fix)
    ofVec3f offset = ofVec3f(0,.5,.25);
    for (auto &v : surfaceMesh.getVertices()){
        v += offset;
    }

}
void ThreeDWorkSurface::update(Joint currentTCP){
    
}
void ThreeDWorkSurface::draw(){
    ofPushMatrix();
    ofPopStyle();
    
    ofScale(1000);      // draw in mm
    ofDrawAxis(.05);
    
    // show the surface
    ofSetColor(100);
    surfaceMesh.draw();
    ofSetColor(250,100);
    surfaceMesh.drawWireframe();
    
    // show surface normals
    ofSetColor(ofColor::aqua,100);
    for (auto &face : surfaceMesh.getUniqueFaces()){
        ofVec3f n = face.getFaceNormal();
        n /= -100; // scale to meters & flip
        ofVec3f pos = (face.getVertex(0) + face.getVertex(1) + face.getVertex(2)) / 3;
        ofDrawLine(pos, pos+n);
    }
    
    // draw 2D & 3D toolpaths
    ofSetColor(200, 0, 0);
    toolpath2D.draw();
    ofSetColor(ofColor::chartreuse);
    ofSetLineWidth(3);
    toolpath.draw();
    
    // show toolpath normals
    ofSetColor(ofColor::blue);
    for (int i=0; i<toolpathOrients.size(); i++){
        
        ofQuaternion q = toolpathOrients[i];
        ofMatrix4x4 m44 = ofMatrix4x4(q);
        m44.setTranslation(toolpath[i]);
        ofPushMatrix();
        glMultMatrixf(m44.getPtr());
        ofDrawAxis(.01);
        ofPopMatrix();
        
    }
    ofSetLineWidth(1);
    
    ofPushStyle();
    ofPopMatrix();
}

Joint ThreeDWorkSurface::getTargetPoint(float t){
    if(lines.size() > 0){
        float length = toolpath.getLengthAtIndex(strokes3D[targetIndex].getVertices().size()-1);
        float dist = feedRate*t;
        float indexInterpolated = toolpath.getIndexAtPercent(dist/length);
        
        ofPoint p = toolpath.getPointAtIndexInterpolated(indexInterpolated);
        
        targetToolPoint.position = p;
        targetToolPoint.rotation = toolpathOrients[(int)indexInterpolated];
        
        if(indexInterpolated > strokes3D[targetIndex].getVertices().size()-1){
            targetIndex++;
        }
        if(targetIndex > lines.size()-1.){
            targetIndex = 0;
        }
        
    }
    return targetToolPoint;
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

void ThreeDWorkSurface::projectToolpath(ofMesh mesh, vector<ofPolyline> &path2D, vector<ofPolyline> &path){
    path.clear();
    toolpath.clear();
    toolpathOrients.clear();
    for(auto &line : path2D){
        for (auto &v : line.getVertices()){
            
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
                }
                
            }
        }
    }
    
    toolpath.close();
    
}

//--------------------------------------------------------------
void ThreeDWorkSurface::projectToolpath(ofMesh mesh, ofPolyline &path2D, ofPolyline &path){
    
    path.clear();
    toolpath.clear();
    toolpathOrients.clear();
    
    for (auto &v : path2D.getVertices()){
        
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
    
    
    projectToolpath(surfaceMesh, strokes, strokes3D);
}