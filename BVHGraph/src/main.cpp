#include "ofMain.h"
#include "ofxBvh.h"

void exportPositions(ofxBvh& bvh, string filename, string delimiter=",") {
    ofFile local;
    ofFile global;
    local.open(filename + "-local-positions.csv", ofFile::WriteOnly);
    global.open(filename + "-global-positions.csv", ofFile::WriteOnly);
    int n = bvh.getNumFrames();
    int m = bvh.getJoints().size();
    for(int i = 0; i < n; i++) {
        bvh.setFrame(i);
        bvh.update();
        for(int j = 0; j < m; j++) {
            auto joint = bvh.getJoints()[j];
            
            glm::vec3 lp(joint->globalMat[3]);
            if (!joint->isRoot()) {
                lp -= glm::vec3(joint->getParent()->globalMat[3]);
            }
            local << lp.x << delimiter << lp.y << delimiter << lp.z;
            
            glm::vec3 gp(joint->globalMat[3]);
            global << gp.x << delimiter << gp.y << delimiter << gp.z;
            
            if(j + 1 < m) {
                global << delimiter;
                local << delimiter;
            }
        }
        global << "\n";
        local << "\n";
    }
    global.close();
    local.close();
}

// assumes the data is [joints x frames]
// outputs in the format [frames x joints]
void exportRotations(const vector<vector<glm::quat>>& rotations, string filename, string delimiter=",") {
    ofFile quats;
    ofFile euler;
    quats.open(filename + "-quats.csv", ofFile::WriteOnly);
    euler.open(filename + "-euler.csv", ofFile::WriteOnly);
    int m = rotations.size();
    int n = rotations[0].size();
    for(int i = 0; i < n; i++) {
        for(int j = 0; j < m; j++) {
            const glm::quat& q = rotations[j][i];
            
            glm::vec4 nq(q.x, q.y, q.z, q.w);
            nq = (nq / 2.f) + 0.5f;
            quats << nq.x << delimiter << nq.y << delimiter << nq.z << delimiter << nq.w;
            
            glm::vec3 ne(glm::eulerAngles(q));
            ne = (ne / float(TWO_PI)) + 0.5f;
            euler << ne.x << delimiter << ne.y << delimiter << ne.z;
            
            if(j + 1 < m) {
                quats << delimiter;
                euler << delimiter;
            }
        }
        quats << "\n";
        euler << "\n";
    }
    quats.close();
    euler.close();
}

class ofApp : public ofBaseApp {
public:
    ofxBvh bvh;
    ofEasyCam cam;
    bool showLocal = true;
    bool normalized = false;
    
    vector<string> jointRotationNames;
    vector<vector<ofMesh>> jointRotationMeshes;
    vector<vector<ofMesh>> jointRotationMeshesNormalized;
//    ofMesh skeletons;
    
    void setup() {
        ofLog() << "Loading file...";
        
        ofJson settings = ofLoadJson("settings.json");
        string fn = settings["filename"];
        int startFrame = settings["startFrame"];
        bool useCentering = settings["useCentering"];
        string visualization = settings["visualization"];
        
        ofBackground(0);
        bvh.load(fn);
        bvh.cropToFrame(startFrame);
        bvh.play();
        
        ofLog() << "Collecting all rotations...";
        
        // collect all joint rotation data
        int n = bvh.getNumFrames();
        int m = bvh.getJoints().size();
        vector<vector<glm::quat>> frameRotationData(m, vector<glm::quat>(n));
        for(int i = 0; i < n; i++) {
            bvh.setFrame(i);
            bvh.update();
            int j = 0;
            for(auto joint : bvh.getJoints()) {
                glm::quat q(joint->localMat);
                frameRotationData[j][i] = glm::normalize(q);
                j++;
            }
        }
        
        // align orientation of all quats
        for(int j = 0; j < m; j++) {
            for(int i = 1; i < n; i++) {
                glm::quat& q = frameRotationData[j][i];
                glm::quat& pq = frameRotationData[j][i-1];
                float current = 0, inverted = 0;
                for(int k = 0; k < 4; k++) {
                    current += abs(pq[k] - q[k]);
                    inverted += abs(pq[k] + q[k]);
                }
                if(inverted < current) {
                    q *= -1;
                }
            }
        }
        
        // "center" all quats to initial orientation
        if(useCentering) {
            cout << "initial centers:" << endl;
            for(int j = 0; j < m; j++) {
                glm::quat initial = frameRotationData[j][0];
                cout << initial << endl;
                for(int i = 0; i < n; i++) {
                    frameRotationData[j][i] *= glm::inverse(initial);
                }
            }
        }
        
        if(settings["export"]) {
            string basename = ofFile(fn).getBaseName();
//            ofLog() << "Exporting all rotations";
//            exportRotations(frameRotationData, basename);
            ofLog() << "Exporting all positions";
            exportPositions(bvh, basename);
        }
        
        // build meshes
//        skeletons.setMode(OF_PRIMITIVE_LINES);
//        for(int i = 0; i < n; i++) {
//            bvh.setFrame(i);
//            bvh.update();
//            for(int j = 0; j < m; j++) {
//                auto joint = bvh.getJoints()[j];
//                glm::vec3 pos(joint->globalMat[3]);
//                if(!joint->isRoot()) {
//                    glm::vec3 posParent(joint->getParent()->globalMat[3]);
//                    skeletons.addVertex(posParent);
//                    skeletons.addVertex(pos);
//                }
//            }
//        }
        
        float minRange = 1e-10;
        int components = visualization == "quat" ? 4 : 3;
        for(int j = 0; j < m; j++) {
            vector<ofMesh> meshes, meshesNormalized;
            int reasonable = 0;
            const ofxBvhJoint* joint = bvh.getJoints()[j];
            string name = joint->name;
            if(name == "Solving") continue;
//            glm::quat& initial = frameRotationData[0][j];
            for(int k = 0; k < components; k++) {
                ofMesh mesh, meshNormalized;
                mesh.setMode(OF_PRIMITIVE_LINE_STRIP);
                meshNormalized.setMode(OF_PRIMITIVE_LINE_STRIP);
                float minValue = 0, maxValue = 0;
                float minAngle = 0, maxAngle = 0;
                vector<float> values;
                for(int i = 0; i < n; i++) {
                    glm::quat q = frameRotationData[j][i];
                    
                    float x;
                    if (visualization == "quat") {
                        // raw quaternion approach
                        x = q[k];
                        x = (x / 2) + 0.5;
                    } else {
                        // euler approach
                        // has the nice property of always looping around
                        // bad property of gimbal lock
                        x = glm::eulerAngles(q)[k]; // should this be different for different orders?
                        x = (x / TWO_PI) + 0.5; // from -PI/+PI to 0/1
                    }
                    
                    // axis-angle approach
//                    float angle = glm::angle(q);
//                    if(i == 0 || angle < minAngle) minAngle = angle;
//                    if(i == 0 || angle > maxAngle) maxAngle = angle;
//                    glm::vec3 axis = glm::normalize(glm::axis((glm::quat) q));
//                    axis *= angle;
//                    float x = vector<float>{axis.x, axis.y, axis.z}[k];
//                    x = (x / (4 * PI)) + 0.5;
                    
                    if(i == 0 || x < minValue) minValue = x;
                    if(i == 0 || x > maxValue) maxValue = x;
                    
                    // quantize
//                    int quantization = 8;
//                    x = floor(x * quantization) / (float) quantization;
                    
                    values.push_back(x);
                }
                float range = maxValue - minValue;
                if(range > minRange) reasonable++;
//                cout << name << " " << range << endl;
                
//                cout << name << " angle " << minAngle << " to " << maxAngle << endl;
                
                for(int i = 0; i < n; i++) {
                    float x = values[i];
                    mesh.addVertex(ofVec3f((float) i / n, x));
                    x = (x - minValue) / (maxValue - minValue);
                    meshNormalized.addVertex(ofVec3f((float) i / n, x));
                }
                meshes.emplace_back(mesh);
                meshesNormalized.emplace_back(meshNormalized);
            }
            if(reasonable > 0) { // change to > to exclude some
                jointRotationMeshes.emplace_back(meshes);
                jointRotationMeshesNormalized.emplace_back(meshesNormalized);
                jointRotationNames.emplace_back(name);
            }
        }
    }
    void update() {
        if(!bvh.isPlaying()) {
            int index = ((float) mouseX / ofGetWidth()) * bvh.getNumFrames();
            bvh.setFrame(index);
        }
        bvh.update();
    }
    void draw() {
        ofSetColor(255);
        
        int index = bvh.getFrame();
        float x = (ofGetWidth() * index) / bvh.getNumFrames();
        ofDrawLine(x, 0, x, ofGetHeight());
        ofDrawBitmapString(ofToString(index), ofGetWidth() / 2, ofGetHeight() - 20);
        
        cam.begin();
//        if(ofGetKeyPressed(' ')) {
//            skeletons.draw();
//        } else {
        bvh.draw();
//        }
        cam.end();
        
        float nodeScale = 50;
        ofCamera view = cam;
        int n = jointRotationMeshes.size();
        float height = ofGetHeight() / n;
        for(int i = 0; i < n; i++) {
            string name = jointRotationNames[i];
            
//            const vector<ofMesh>& mesh = jointRotationMeshes[i];
            const vector<ofMesh>& mesh = normalized ?
                jointRotationMeshesNormalized[i] :
                jointRotationMeshes[i];
            drawRotationGraph(mesh,
                              ofRectangle(0, i * height, ofGetWidth(), height),
                              name);
            const ofxBvhJoint* joint = bvh.getJoint(name);
            
            ofNode node;
            node.setOrientation(showLocal ?
                joint->localMat :
                joint->globalMat);
            view.begin(ofRectangle(x + 10, i * height, height, height));
            ofScale(nodeScale);
            node.draw();
            view.end();
        }
    }
    void drawRotationGraph(const vector<ofMesh>& meshes, ofRectangle viewport, string name="") {
        ofPushStyle();
        ofPushMatrix();
        ofTranslate(viewport.x, viewport.y);
        ofSetColor(255);
        ofDrawBitmapString(name, 10, 20);
        ofScale(viewport.width, viewport.height);
        for(int i = 0; i < meshes.size(); i++) {
            ofSetColor(vector<ofColor>{ofColor::cyan, ofColor::magenta, ofColor::yellow, ofColor::white}[i]);
            meshes[i].draw();
        }
        ofPopMatrix();
        ofPopStyle();
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
        if(key == 'l') {
            showLocal = !showLocal;
        }
        if(key == 'n') {
            normalized = !normalized;
        }
    }
};

int main() {
    ofSetupOpenGL(1024, 1024, OF_WINDOW);
    ofRunApp(new ofApp());
}
