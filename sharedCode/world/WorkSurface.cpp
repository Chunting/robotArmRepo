#include "WorkSurface.h"

WorkSurface::WorkSurface(){
    
}

WorkSurface::~WorkSurface(){
    
}

void WorkSurface::setup(RobotParameters * parameters){
    this->parameters = parameters;
    
}

void WorkSurface::update(Joint _currentTCP){
    currentTCP = _currentTCP;
}

void WorkSurface::draw(){
    ofPushMatrix();
    {
        
        ofPushStyle();
        {
            ofSetColor(255, 255, 0, 200);
            surfaceMesh.draw();
            
        }
        ofPopStyle();
    }
    ofPopMatrix();
}