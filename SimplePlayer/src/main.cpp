#include "ofMain.h"
#include "ofxBvh.h"

class ofApp : public ofBaseApp {
public:
    ofxBvh bvh;
    ofEasyCam cam;
    
    void setup() {
        ofBackground(0);
//        bvh.load("bvh/MotionData-180216/erisa003-short.bvh");
        bvh.load("bvh/MotionData-180216/Take54.bvh");
        bvh.play();
        bvh.setLoop(true);
    }
    void update() {
        if(!bvh.isPlaying()) {
            bvh.setPosition((float) mouseX / ofGetWidth());
        }
        bvh.update();
    }
    void draw() {
        ofSetColor(255);
        cam.begin();
        bvh.draw();
        cam.end();
    }
    void keyPressed(int key) {
        if(key == '\t') {
            if(bvh.isPlaying()) {
                bvh.stop();
            } else {
                bvh.play();
            }
        }
        if(key == 'f') {
            ofToggleFullscreen();
        }
    }
};

int main() {
    ofSetupOpenGL(1024, 1024, OF_WINDOW);
    ofRunApp(new ofApp());
}
