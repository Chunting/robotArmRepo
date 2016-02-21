#pragma once

#include "ofMain.h"

#include "ofxNatNet.h"
#include "ofxOsc.h"
#include "UR5.h"
class ofApp : public ofBaseApp{
    
public:
    void setup();
    void update();
    void draw();
    
    void keyPressed(int key);
    void keyReleased(int key);
    void mouseMoved(int x, int y );
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void mouseEntered(int x, int y);
    void mouseExited(int x, int y);
    void windowResized(int w, int h);
    void dragEvent(ofDragInfo dragInfo);
    void gotMessage(ofMessage msg);
    
    
           //Motion Capture Visualization
    ofxNatNet natnet;

    ofEasyCam cam;
    //Motion Capture OSC Server
    ofxOscSender sender;
    UR5 robot;
};
