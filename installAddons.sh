#!/bin/bash

# Dependencies for DepthKit
#make sure you are in the DepthKit project when you run this script

cd ../../addons

PREFIX="git clone http://github.com/"
${PREFIX}kylemcdonald/ofxTiming
${PREFIX}kylemcdonald/ofxCV
${PREFIX}CreativeInquiry/ofxNatNet
${PREFIX}julapy/ofxQuadWarp

cd ../apps/robotArmRepo
