//
//  URMove.cpp
//  ofxURDriver
//
//  Created by Dan Moore on 2/20/16.
//
//
#include "URMove.h"
URMove::URMove(){
    
}
URMove::~URMove(){
    
}
void URMove::setup(){
    movementParams.setName("UR Movements");
    movementParams.add(maxSpeed.set("Max Speed", 0, 0, 1000));
    movementParams.add(minSpeed.set("Min Speed", 0, 0, 1000));
    movementParams.add(timeDiff.set("Delta T", 125, 0, 250));
    
    for(int i = 0; i < 8; i++){
        previews.push_back(new UR5KinematicModel());
        previews.back()->setup();
        cams.push_back(ofEasyCam());
        
    }
    selectedSolution = -1;
    deltaTimer.setSmoothing(true);
    distance = 0;
    
    
}


void URMove::update(){
    deltaTimer.tick();
    deltaT = 1.0/timeDiff;
    deltaTime = deltaTimer.getPeriod();
    targetPoint.position = targetPoint.position.interpolate(newTargetPoint.position, 0.03);
    targetPoint.rotation.slerp(0.03, targetPoint.rotation, newTargetPoint.rotation);
    mat.setTranslation(targetPoint.position);
    mat.setRotate(targetPoint.rotation);
    urKinematics(mat);
  
 
}

vector<double> URMove::getTargetJointPos(){
    if(selectedSolution > -1){
        return inversePosition[selectedSolution];
    }else{
        return currentPose;
    }
}

vector<double> URMove::getCurrentSpeed(){
      computeVelocities();
    for(int i = 0; i < currentJointSpeeds.size(); i++){
        currentJointSpeeds[i] = ofLerp(lastJointSpeeds[i], currentJointSpeeds[i], 0.9);
    }
    return currentJointSpeeds;
}

void URMove::setCurrentJointPosition(vector<double> pose){
    currentPose = pose;
    urKinematics(currentPose);
    selectedSolution = selectSolution();
}

void URMove::computeVelocities(){
    currentJointSpeeds.clear();
    currentJointSpeeds.assign(6, 0);
    if(selectedSolution != -1){
        if(currentPose.size() > 0){
            for(int i = 0; i < inversePosition[selectedSolution].size(); i++){
                currentJointSpeeds[i] = (inversePosition[selectedSolution][i]-currentPose[i])/deltaTime;
                if(abs(currentJointSpeeds[i]) > PI){
                    ofLog(OF_LOG_ERROR)<<"TOO FAST "<<ofToString(currentJointSpeeds[i], 10)<<endl;
                }
                if(i > 4){
                    currentJointSpeeds[i] = 0;
                }
                
            }
        }
    }
    lastJointSpeeds = currentJointSpeeds;
}

void URMove::addTargetPoint(Joint target){
    newTargetPoint = target;    
}


void URMove::draw(){
    int j = 0;
    if(inversePosition.size() > 0){
        cams[0].begin(ofRectangle(ofGetWindowWidth()/2, 0, ofGetWindowWidth()/2, ofGetWindowHeight()));
        ofPushMatrix();
        previews[0]->draw();
        ofPopMatrix();
        ofPushMatrix();
        ofSetColor(255, 0, 255);
        ofDrawSphere(targetPoint.position*ofVec3f(1000, 1000, 1000), 10);
        ofPopMatrix();
        cams[0].end();
        j+=1;
        j = j%4;
    }
    
    
    cam.begin(ofRectangle(0, 0, ofGetWindowWidth()/2, ofGetWindowHeight()));
    ofPushMatrix();
    ofSetColor(255, 255, 0);
    ofDrawSphere(targetPoint.position*ofVec3f(1000, 1000, 1000), 10);
    ofPopMatrix();
    cam.end();
    
    
    ofPushMatrix();
    ofTranslate(ofGetWindowWidth()-100, 0);
    for(int i = 0; i < currentJointSpeeds.size(); i++){
        ofSetColor(255, 255, 0);
        if(i%2==0)
            ofSetColor(255, 0, 255);
        ofDrawRectangle(0, i*50, ofMap(currentJointSpeeds[i], 0, TWO_PI, 0, 100, true), 50);
    }
    ofPopMatrix();
}

int URMove::selectSolution(){
    vector<int> nearestSolution;
    vector<int> count;
    if(currentPose.size() > 0 && inversePosition.size() > 0){
        vector<double> minDistances;
        vector<vector<double> > diffs;
        diffs.resize(inversePosition.size());
        nearestSolution.resize(inversePosition[0].size());
        count.resize(inversePosition.size());
        minDistances.assign(inversePosition.size(), DBL_MAX);
        for(int i = 0; i < inversePosition.size(); i++){
            diffs[i].resize(inversePosition[i].size());
            for(int j = 0; j < inversePosition[i].size(); j++){
                diffs[i][j]=(inversePosition[i][j]-abs(currentPose[j]));
            }
        }
        
        for(int i = 0; i < diffs.size(); i++){
            for(int j = 0; j < diffs[i].size(); j++){
                if(diffs[i][j] < minDistances[i]){
                    nearestSolution[j] = i;
                    minDistances[i] = diffs[i][j];
                    distance = minDistances[i];
                }
            }
        }
        vector<int> count;
        count.resize(inversePosition.size());
        for(int i =0; i < nearestSolution.size();i++){
            count[nearestSolution[i]]++;
        }
        int nearest = INT_MIN;
        int max = INT_MIN;
        for(int i = 0; i < count.size(); i++){
            if(count[i] > max){
                nearest = i;
                max = count[i];
            }
        }
        ofLog()<<"nearest "<<nearest<<endl;
        return 0;
    }else{
        return -1;
    }
}

void URMove::urKinematics(vector<double> input){
    if(input.size() == 6){
        urKinematics(input[0], input[1], input[2], input[3], input[4], input[5]);
    }
}

void URMove::urKinematics(ofMatrix4x4 input){
    double q_sols[8*6];
    double* T = new double[16];

    for(int i = 0; i < 4; i++){
        T[i] = (double)input._mat[i][0];
        T[i+(4)] = (double)input._mat[i][1];
        T[i+(8)] = (double)input._mat[i][2];
        T[i+(12)] = (double)input._mat[i][3];
    }

    int num_sols = kinematics.inverse(T, q_sols);
    inversePosition.clear();
    for(int i=0;i<num_sols;i++){
        vector<double> fooSol;
        fooSol.push_back(q_sols[i*6]);
        fooSol.push_back(q_sols[i*6+1]);
        fooSol.push_back(q_sols[i*6+2]);
        fooSol.push_back(q_sols[i*6+3]);
        fooSol.push_back(q_sols[i*6+4]);
        fooSol.push_back(q_sols[i*6+5]);
        //        ofLog()<<ofToString(fooSol)<<endl;
        inversePosition.push_back(fooSol);
    }
    
    if(inversePosition.size() > 0){
        for(int i = 0; i < inversePosition.size(); i++){
            previews[i]->jointsRaw = inversePosition[i];
            //            cout<<ofToString(previews[i]->jointsRaw)<<endl;
            previews[i]->jointsProcessed.resize(previews[i]->jointsRaw.size());
            for(int j = 0; j < previews[i]->joints.size(); j++){
                if(j == 1 || j == 3){
                    inversePosition[i][j] = fmodf(inversePosition[i][j], TWO_PI);
                    previews[i]->jointsRaw[j]= inversePosition[i][j];
                }
                previews[i]->jointsProcessed[j] = ofRadToDeg(previews[i]->jointsRaw[j]);
                if(j == 1 || j == 3){
                    previews[i]->jointsProcessed[j]+=90;
                }
                previews[i]->joints[j].rotation.makeRotate(previews[i]->jointsProcessed[j], previews[i]->joints[j].axis);
            }
        }
    }
    
}
void URMove::urKinematics(double o, double t, double th, double f, double fi, double s){
    double q[6] = {o, t, th, f, fi, s};
    double* T = new double[16];
    kinematics.forward(q, T);
    double q_sols[8*6];
    int num_sols = kinematics.inverse(T, q_sols);
    inversePosition.clear();
    for(int i=0;i<num_sols;i++){
        vector<double> fooSol;
        fooSol.push_back(q_sols[i*6+0]);
        fooSol.push_back(q_sols[i*6+1]);
        fooSol.push_back(q_sols[i*6+2]);
        fooSol.push_back(q_sols[i*6+3]);
        fooSol.push_back(q_sols[i*6+4]);
        fooSol.push_back(q_sols[i*6+5]);
        //        ofLog()<<ofToString(fooSol)<<endl;
        inversePosition.push_back(fooSol);
    }
}

