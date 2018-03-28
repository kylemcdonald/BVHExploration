#include "ofMain.h"
#include "ofxBvh.h"
//labels
vector<string>label_str = {"tPose", "hand", "foot", "all", "fun", "sad", "robot", "sexy", "junkie", "bouncie", "wavey", "swingy"};

void exportPositions(ofxBvh& bvh, string filename, bool relative=false, float bpm = 90) {
    ofFile output;
    output.open(filename, ofFile::WriteOnly);
    int m = bvh.getNumFrames();
    
    float fps = 120;
    
    float two_bars = 60. / bpm * 8 * 2;
    float eight_bars  = 60. / bpm * 8 * 8;
    int two_bars_in_frames = two_bars * fps;
    int eight_bars_in_frames = eight_bars * fps;

    int label ;
    for(int j = 0; j < m; j++) {
        int n = bvh.getNumJoints();
        
        if(j < two_bars_in_frames)
        {
            label = 0;
        }
        else if(j < two_bars_in_frames + eight_bars_in_frames * 11  )
        {
            label = (j - two_bars_in_frames) / eight_bars_in_frames + 1;
        }
        else
        {
            label = 12;
        }
        
        output << label << "\t";
        
        bvh.setFrame(j);
        bvh.update();
        for(int i = 0; i < n; i++) {
            const ofxBvhJoint* joint = bvh.getJoints()[i];
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

float smoothStep(float x) {
    return 3*(x*x) - 2*(x*x*x);
}

class AnimatedMesh {
public:
    ofMesh a, b;
    ofMesh getInterpolated(float t, bool copyColors=true) {
        if(t == 0) {
            return a;
        }
        if(t == 1) {
            return b;
        }
        ofMesh x;
        x.setMode(a.getMode());
        if(copyColors) {
            x.addColors(a.getColors());
        }
        auto& av = a.getVertices();
        auto& bv = b.getVertices();
        int n = av.size();
        if(av.size() != bv.size()) {
            return a;
        }
        for(int i = 0; i < n; i++) {
            ofVec3f v(av[i]);
            v.interpolate(bv[i], t);
            x.addVertex(v);
        }
        return x;
    }
    float transitionTime = 0;
    float transitionDuration = 1;
    void transition(float transitionDuration=1) {
        if(a.getNumVertices() > 0) {
            transitionTime = ofGetElapsedTimef();
            this->transitionDuration = transitionDuration;
        }
    }
    float tPrevious = 0;
    ofMesh getCurrent(bool copyColors=true) {
        float curTime = ofGetElapsedTimef();
        float t = (curTime - transitionTime) / transitionDuration;
        if(tPrevious < 1 && t >= 1) {
            std::swap(a,b);
        }
        tPrevious = t;
        if(t >= 1) {
            t = 0;
        }
        return getInterpolated(smoothStep(t), copyColors);
    }
};

class ofApp : public ofBaseApp {
public:
    ofxBvh bvh;
    ofEasyCam cam;
    AnimatedMesh meshPair;
    string embeddingFilename;
    int embeddingIndex = 0;
    deque<int> recentIndices;
    
    void setup() {
        ofBackground(0);
        glPointSize(4);
        
//        bvh.load("bvh/Daito/Take54.bvh");
//        exportPositions(bvh, "Take54-absolute-export.tsv", false);
//        exportPositions(bvh, "Take54-relative-export.tsv", true);
        
        bvh.load("bvh/erisa004_erisa004_mcp.bvh");
        exportPositions(bvh, "erisa004-absolute-export.tsv", false, 90);
//        exportPositions(bvh, "erisa003-relative-export.tsv", true);
//        exportQuaternions(bvh, "erisa003-quaternions.tsv");
        
//        bvh.play();
        bvh.setLoop(true);
        
        loadNextEmbedding();
    }
    void loadNextEmbedding() {
        ofDirectory files;
        files.allowExt("tsv");
        files.listDir("embeddings");
        if(files.size() > 0) {
            ofFile path = files[embeddingIndex % files.size()];
            embeddingFilename = path.getBaseName();
            loadEmbedding(path);
            embeddingIndex++;
        }
    }
    void loadEmbedding(ofFile path) {
        ofMesh& mesh = meshPair.b;
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
        
        meshPair.transition();
    }
    void update() {
        
    }
    void draw() {
        float t = ofMap(sin(ofGetElapsedTimef()), -1, +1, 0, 1);
        ofMesh mesh = meshPair.getCurrent(); //meshPair.getInterpolated(t);
        
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
        
        cam.begin();
        bvh.draw();
        cam.end();
        
        if(mesh.getNumVertices() == 0) {
            return;
        }
        
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
