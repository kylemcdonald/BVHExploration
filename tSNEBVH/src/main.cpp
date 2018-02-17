#include "ofMain.h"
#include "ofxBvh.h"

void exportPositions(ofxBvh& bvh, string filename, bool relative=false) {
    ofFile output;
    output.open(filename, ofFile::WriteOnly);
    int m = bvh.getNumFrames();
    for(int j = 0; j < m; j++) {
        int n = bvh.getNumJoints();
        bvh.setFrame(j);
        bvh.update();
        for(int i = 0; i < n; i++) {
            const ofxBvhJoint* joint = bvh.getJoint(i);
            ofVec3f position = joint->getPosition();
            if(relative) {
                ofxBvhJoint* parent = joint->getParent();
                if(parent != NULL) {
                    position -= parent->getPosition();
                }
            }
            output << position.x << '\t' << position.y << '\t' << position.z;
            if(i + 1 < n) {
                output << "\t";
            }
        }
        if(j + 1 < m) {
            output << "\n";
        }
    }
    output.close();
}

class ofApp : public ofBaseApp {
public:
    ofxBvh bvh;
    ofEasyCam cam;
    ofMesh mesh;
    string embeddingFilename;
    int embeddingIndex = 0;
    deque<int> recentIndices;
    
    void setup() {
        ofBackground(0);
        
//        bvh.load("bvh/Daito/Take54.bvh");
//        exportPositions(bvh, "Take54-absolute-export.tsv", false);
//        exportPositions(bvh, "Take54-relative-export.tsv", true);
        
        bvh.load("bvh/MotionData-180216/erisa003.bvh");
//        exportPositions(bvh, "erisa003-absolute-export.tsv", false);
//        exportPositions(bvh, "erisa003-relative-export.tsv", true);
        
//        bvh.play();
        bvh.setLoop(true);
        
        loadNextEmbedding();
    }
    void loadNextEmbedding() {
        ofDirectory files;
        files.allowExt("tsv");
        files.listDir("embeddings");
        ofFile path = files[embeddingIndex % files.size()];
        embeddingFilename = path.getBaseName();
        loadEmbedding(path);
        embeddingIndex++;
    }
    void loadEmbedding(ofFile path) {
        mesh.clear();
        mesh.setMode(OF_PRIMITIVE_POINTS);
        
        ofBuffer buffer = ofBufferFromFile(path.getAbsolutePath());
        for(auto& line : buffer.getLines()) {
            if(line.size()) {
                ofVec2f x;
                stringstream(line) >> x;
                mesh.addVertex(ofVec3f(x));
            }
        }
        
        int n = mesh.getNumVertices();
        for(int i = 0; i < n; i++) {
            float hue = ofMap(i, 0, n, 0, 255);
            ofColor color = ofColor::fromHsb(hue, 255, 255);
            mesh.addColor(color);
        }
    }
    void draw() {
        ofSetColor(255);
        ofDrawBitmapString(embeddingFilename, 10, 20);
        
        float scale = ofGetHeight();
        float offset = (ofGetWidth() - scale) / 2;
        
        ofPushMatrix();
        ofTranslate(offset, 0);
        ofScale(scale, scale);
        mesh.draw();
        ofPopMatrix();
        
        int skipFrames = 15;
        int selectedFrame, nearestIndex;
        if(bvh.isPlaying()) {
            selectedFrame = bvh.getFrame();
            nearestIndex = selectedFrame / skipFrames;
        } else {
            float nearestDistance = -1;
            nearestIndex = 0;
            ofVec2f mouse(mouseX - offset, mouseY);
            mouse /= scale;
            int n = mesh.getNumVertices();
            for(int i = 0; i < n; i++) {
                const ofVec3f& x = mesh.getVertices()[i];
                float distance = mouse.squareDistance(x);
                if(nearestDistance < 0 || distance < nearestDistance) {
                    nearestDistance = distance;
                    nearestIndex = i;
                }
            }
            selectedFrame = nearestIndex * skipFrames;
            if(selectedFrame >= 0 && selectedFrame < bvh.getNumFrames()) {
                bvh.setFrame(selectedFrame);
            }
        }
        bvh.update();
        
        recentIndices.push_front(nearestIndex);
        while(recentIndices.size() > 32) {
            recentIndices.pop_back();
        }
        
        int n = recentIndices.size();
        ofPushMatrix();
        ofNoFill();
        ofTranslate(offset, 0);
        for(int i = 0; i < n-1; i++) {
            float opacity = ofMap(i, 0, n, 255, 0);
            ofVec2f position = mesh.getVertex(recentIndices[i]) * scale;
            ofVec2f nextPosition = mesh.getVertex(recentIndices[i+1]) * scale;
            ofSetColor(255, opacity);
            ofDrawCircle(position, 10);
            ofDrawLine(position, nextPosition);
        }
        ofPopMatrix();
        
        cam.begin();
        bvh.draw();
        cam.end();
    }
    void keyPressed(int key) {
        if(key == ' ') {
            loadNextEmbedding();
        }
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
