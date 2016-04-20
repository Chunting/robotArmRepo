#include "3DPath.h"
void ThreeDPath::setup(){
    // set the Z axis as the forward axis by default
    makeZForward = true;
    
    ptIndex = 0;
    centroid = ofPoint(.5,.25,.25); // all coordinates are in meters
    
    // load/create different paths
    parsePts("path_XZ.txt", path_XZ);
    parsePts("path_YZ.txt", path_YZ);
    parsePts("path_SPIRAL.txt", path_SPIRAL);
    path_PERIODIC = buildPath();
    
    ofPath p;
    p.setCircleResolution(2000);
    p.circle(0., 0., 0.3, 0.325);

    vector<ofPolyline> fooCircle = p.getOutline();
    
    // assign path and make profile
    profile = buildProfile(.025,4);
    path = fooCircle[0];
    buildPerpFrames(path);
    reverse = false;
}

void ThreeDPath::keyPressed(int key){
        float step = .01;   // 10 millimeters
        
        if (key == OF_KEY_UP){
            for (auto &p : path.getVertices())
                p.y += step;
            buildPerpFrames(path);
        }else if(key == OF_KEY_DOWN){
            for (auto &p : path.getVertices())
                p.y -= step;
            buildPerpFrames(path);
        }else if(key == OF_KEY_RIGHT){
            for (auto &p : path.getVertices())
                p.x += step;
            buildPerpFrames(path);
        }else if(key == OF_KEY_LEFT){
            for (auto &p : path.getVertices())
                p.x -= step;
            buildPerpFrames(path);
        }else if (key == '!'){
            makeZOut = false;
            makeZForward = true;
        }
        else if (key == '@'){
            makeZForward = false;
            makeZOut = true;
        }
        else if (key == '#'){
            makeZForward = false;
            makeZOut = false;
        }
        
        else if (key == '$'){
            path = path_XZ;
            buildPerpFrames(path);
        }
        else if (key == '%'){
            path = path_YZ;
            buildPerpFrames(path);
        }
        else if (key == '^'){
            path = path_SPIRAL;
            buildPerpFrames(path);
        }
        else if (key == '&'){
            path = path_PERIODIC;
            buildPerpFrames(path);
        }
}
<<<<<<< HEAD
<<<<<<< HEAD

ofMatrix4x4 ThreeDPath::getNextPose(){
<<<<<<< Updated upstream

=======
=======
ofVec3f ThreeDPath::getNextNormal(){
>>>>>>> origin/master
=======
ofVec3f ThreeDPath::getNextNormal(){
>>>>>>> origin/master
>>>>>>> Stashed changes
    if(ptf.framesSize()>0){
        ptIndex = (ptIndex +1) % ptf.framesSize();
        
        orientation = ptf.frameAt(ptIndex);
        
        if (makeZForward)
            orientation = zForward(orientation);
        else if (makeZOut)
            orientation = zOut(orientation);
        else
            orientation = flip(orientation);
        
        return ptf.calcCurrentNormal();
    }
}

<<<<<<< HEAD
<<<<<<< HEAD
void ThreeDPath::draw(){
    
<<<<<<< Updated upstream
=======
    // draw all the perp frames if we are paused
    for (int i=0; i<ptf.framesSize(); i++){
        ofMatrix4x4 m44 = ptf.frameAt(i);
=======
=======
>>>>>>> origin/master
ofMatrix4x4 ThreeDPath::getNextPoint(){
    if(ptf.framesSize()>0){
        if(!reverse){
            ptIndex++;
        }else{
            ptIndex--;
        }
        if(ptIndex >= ptf.framesSize()){
            reverse = true;
            ptIndex = ptf.framesSize()-1;
        }
        if(ptIndex < 0){
            reverse = false;
            ptIndex = 0;
        }
<<<<<<< HEAD
>>>>>>> origin/master
=======
>>>>>>> origin/master
        
        orientation = ptf.frameAt(ptIndex);
//        
        if (makeZForward)
            orientation = zForward(orientation);
        else if (makeZOut)
            orientation = zOut(orientation);
        else
            orientation = flip(orientation);
        return orientation;
    }
}
void ThreeDPath::draw(){
    

    
>>>>>>> Stashed changes
    // show the current orientation plane
    ofSetColor(ofColor::lightYellow);
    ofSetLineWidth(3);
    ofPushMatrix();
    ofMultMatrix(orientation);
    profile.draw();
    ofDrawAxis(.010);
    ofPopMatrix();
    
    // show the target point
    ofSetColor(ofColor::yellow);
    if (path.size() > 0){
        ofDrawSphere(path.getVertices()[ptIndex], .003);
    }
    
    
    // show the 3D path
    ofSetLineWidth(.01);
    ofSetColor(ofColor::aqua);
    path.draw();
    
}

//--------------------------------------------------------------
void ThreeDPath::parsePts(string filename, ofPolyline &polyline){
    ofFile file = ofFile(ofToDataPath(filename));
    
    if(!file.exists()){
        ofLogError("The file " + filename + " is missing");
    }
    ofBuffer buffer(file);
    
    //Read file
    for (ofBuffer::Line it = buffer.getLines().begin(), end = buffer.getLines().end(); it != end; ++it) {
        string line = *it;
        
        float scalar = 10;
        
        ofVec3f offset;
        if (filename == "path_XZ.txt")
            offset = ofVec3f(0, .25, 0);
        else if (filename == "path_YZ.txt")
            offset = ofVec3f(.25, 0, 0);
        else
            offset = ofVec3f(.25, .25, 0);
        
        line = line.substr(1,line.length()-2);              // remove end { }
        vector<string> coords = ofSplitString(line, ", ");  // get x y z coordinates
        
        ofVec3f p = ofVec3f(ofToFloat(coords[0])*scalar,ofToFloat(coords[1])*scalar,ofToFloat(coords[2])*scalar);
        p += offset;
        
        polyline.addVertex(p);
    }
    
    // interpolate points to smooth
    ofPolyline temp;
    
    for (int i=0; i<polyline.getVertices().size()-1; i++){
        
        ofVec3f p0 = polyline.getVertices()[i];
        ofVec3f p1 = polyline.getVertices()[i+1];
        
        for (int j=1; j<4; j++){
            float t = j/4.0;
            temp.addVertex(p0.interpolate(p1, t));
        }
        
    }
    
    polyline.clear();
    polyline = temp;
}

//--------------------------------------------------------------
ofPolyline ThreeDPath::buildPath(){
    
    ofPolyline temp;
    
    ofNode n0;
    ofNode n1;
    ofNode n2;
    
    n0.setPosition(centroid.x,centroid.y,centroid.z);
    n1.setParent(n0);
    n1.setPosition(0,0,.1);
    n2.setParent(n1);
    n2.setPosition(0,.0015,0);
    
    float totalRotation = 0;
    float step = .25;
    while (totalRotation < 360){
        
        n0.pan(step);
        n1.tilt(2);
        n2.roll(1);
        
        ofPoint p = n2.getGlobalPosition().rotate(90, ofVec3f(1,0,0));
        
        // and point to path
        temp.addVertex(p);
        
        // add point to perp frames
        ptf.addPoint(p);
        
        totalRotation += step;
    }
    
    temp.close();
    return temp;
}

//--------------------------------------------------------------
void ThreeDPath::buildPerpFrames(ofPolyline polyline){
    
    // reset the perp frames
    ptf.clear();
    
    for (auto &p : polyline){
        ptf.addPoint(p);
    }
    
}

//--------------------------------------------------------------
ofPolyline ThreeDPath::buildProfile(float radius, int res){
    ofPolyline temp;
    
    // make a plane
    if (res == 4){
        temp.addVertex(ofVec3f(-radius/2, radius/2,0));
        temp.addVertex(ofVec3f( radius/2, radius/2,0));
        temp.addVertex(ofVec3f( radius/2,-radius/2,0));
        temp.addVertex(ofVec3f(-radius/2,-radius/2,0));
    }
    // make a polygon
    else{
        float theta = 360/res;
        for (int i=0; i<res; i++){
            ofPoint p = ofPoint(0,0,radius);
            temp.addVertex(p.rotate(theta*i, ofVec3f(1,0,0)));
        }
    }
    
    temp.close();
    return temp;
}

//--------------------------------------------------------------
ofMatrix4x4 ThreeDPath::flip(ofMatrix4x4 originalMat){
    
    ofVec3f pos  = originalMat.getTranslation();
    ofVec3f z = originalMat.getRowAsVec3f(2);   // local y-axis
    
    originalMat.setTranslation(0,0,0);
    originalMat.rotate(180, z.x, z.y, z.z);     // rotate about the y
    originalMat.setTranslation(pos);
    
    return originalMat;
}


//--------------------------------------------------------------
ofMatrix4x4 ThreeDPath::zForward(ofMatrix4x4 originalMat){
    
    ofVec3f pos  = originalMat.getTranslation();
    ofVec3f y = originalMat.getRowAsVec3f(1);   // local y-axis
    
    originalMat.setTranslation(0,0,0);
    originalMat.rotate(90, y.x, y.y, y.z);     // rotate about the y
    originalMat.setTranslation(pos);
    
    return originalMat;
}


//--------------------------------------------------------------
ofMatrix4x4 ThreeDPath::zOut(ofMatrix4x4 originalMat){
    
    ofVec3f pos  = originalMat.getTranslation();
    ofVec3f x = originalMat.getRowAsVec3f(0);   // local x-axis
    
    originalMat.setTranslation(0,0,0);
    originalMat.rotate(-90, x.x, x.y, x.z);      // rotate about the y
    originalMat.setTranslation(pos);
    
    return originalMat;
}
