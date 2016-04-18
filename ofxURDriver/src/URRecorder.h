//
//  URRecorder.h
//  urModernDriverTest
//
// Copyright (c) 2016, Daniel Moore, Madaline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
//

#pragma once
#include "ofMain.h"

struct RobotFrame{
    
};
class URRecorder{
public:
    URRecorder();
    ~URRecorder();
    void setup();
    void update();
    void save();
    bool bRecord;
    bool bSave;
};