#pragma once

#include "ofMain.h"
#include "ur_driver.h"
#include "ofxNatNet.h"
#include "ofxOsc.h"
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
    
    
    // Robot Arm
    UrDriver* robot;
    std::condition_variable rt_msg_cond_;
    std::condition_variable msg_cond_;
    bool has_goal_;
    std::thread* rt_publish_thread_;
    std::thread* mb_publish_thread_;
    double io_flag_delay_;
    double max_velocity_;
    std::vector<double> joint_offsets_;
    std::string base_frame_;
    std::string tool_frame_;
    bool use_ros_control_;
    std::thread* ros_control_thread_;

    
    float elapsed_time, last_time;
    ofVec3f pt;
    vector<ofNode> joints;
    vector<double> jointsRaw;
    vector<ofQuaternion> jointsQ;
    vector<ofVec3f> jointLength;
    vector<ofVec3f> angles;
    ofNode tool;
    ofNode toolD;
    //Motion Capture Visualization
    ofxNatNet natnet;
    ofEasyCam cam;
    
    //Motion Capture OSC Server
    ofxOscSender sender;
};
