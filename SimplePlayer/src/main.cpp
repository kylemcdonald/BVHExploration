#include "ofMain.h"
#include "ofxBvh.h"

float getHeight(ofxBvh& bvh) {
    float height = 0;
    for(auto& joint : bvh.getJoints()) {
        glm::vec3 cur = joint->getPosition();
        cout << cur << endl;
        height = std::max(height, cur.y);
    }
    return height;
}

class ofApp : public ofBaseApp {
public:
    ofxBvh bvh;
    ofEasyCam cam;
    float height;
    string filename = "";
    
    void setup() {
        ofBackground(0);
    }
    void dragged(ofDragInfo& drag) {
        if(drag.files.empty()) return;
        filename = drag.files[0];
        if(ofFile(filename).getExtension() != "bvh") return;
        bvh = ofxBvh(filename);
        bvh.update();
        height = getHeight(bvh);
        bvh.play();
        bvh.setLoop(true);
    }
    void update() {
        if (bvh.getNumFrames() == 0) return;
        if(!bvh.isPlaying()) {
            bvh.setPosition((float) mouseX / ofGetWidth());
        }
        bvh.update();
    }
    void draw() {
        float w = ofGetWidth(), h = ofGetHeight();
        if (bvh.getNumFrames() == 0) {
            ofDrawBitmapString("Drop a file to play.", w/2, h/2);
            return;
        }
        ofSetColor(255);
        cam.begin();
        ofPushMatrix();
        float scale = 0.75 * h / height;
        ofScale(scale, scale, scale);
        ofTranslate(0, -height/2);
        bvh.draw();
        ofPopMatrix();
        cam.end();
        stringstream text;
        text
        << filename << endl
        << "Frame: " << bvh.getFrame() << "/" << bvh.getNumFrames() << " @ " << bvh.getFrameRate() << "fps" << endl
        << "Time: " << round(bvh.getTime()) << "s / " << round(bvh.getDuration()) << "s" << endl
        << "Position: " << bvh.getPosition() << endl
        << "Height: " << height << endl;
        ofDrawBitmapString(text.str(), 10, 20);
        ofDrawBitmapString(ofToString(round(ofGetFrameRate())) + "fps", 10, h-20);
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
    ofSetupOpenGL(1280, 720, OF_WINDOW);
    ofRunApp(new ofApp());
}
