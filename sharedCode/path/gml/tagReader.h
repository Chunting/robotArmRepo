//
//  tagReader.h
//  tag_Capture_simple
//
//  Created by Dan Moore on 3/13/13.
// Copyright (c) 2016, Daniel Moore, Madaline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
//
#pragma once
#include "ofMain.h"
#include "ofxXmlSettings.h"

class tagReader{
    
public:
    
    tagReader();
    ~tagReader();
    
    struct brush{
        int num;
        int size;
        float time;
    };
    
    struct playbackTag{
        vector<ofColor> colors;
        vector<brush> brushes;
        vector<ofPolyline> strokes;
        deque<float> stops;
        float startTime;
        float endTime;
        float offsetX;
        float offsetY;
        float screenX;
        float screenY;
        string name;
    };
    

    void setup(float x, float y, float width, float height);
    void loadFile(string path);
    void clear();
    
    
    vector<playbackTag> tags;
protected:
    int count;
    float zeroX;
    float zeroY;
    float maxX;
    float maxY;
    
};