//
//  ObjectDetection.cpp
//  opencv
//
//  Created by 심지훈 on 02/10/2019.
//  Copyright © 2019 Shim. All rights reserved.
//

#include "ObjectDetection.hpp"
#include "Utility.hpp"

vector<string> classes;
vector<Scalar> colors;

float confThreshold = 0.4f;
float maskThreshold = 0.5f;

static bool verbose = false;
void setDetectionVerbose(bool on){ verbose = on; }

// Runs the forward pass to get output from the output layers
vector<String> outNames(2);
vector<Mat> outs;

// ---------------------------------------------------------------- model cache
// The original detect() called readNetFromTensorflow() on every single frame,
// which re-parsed the whole graph per frame and made the pipeline unusably
// slow. Load once, reuse forever.
static Net& cachedNet() {
    static Net net;
    return net;
}

static string& cachedPath() {
    static string path;
    return path;
}

void loadNameOfClasses(string classesFilePath){
    //Load names of classes
    string line;
    ifstream ifs(classesFilePath.c_str());
    if(!ifs.good()){
        cerr << "WARNING: cannot open class list: " << classesFilePath << "\n";
        return;
    }
    classes.clear();
    while(getline(ifs,line)){
        while(!line.empty() && (line.back()=='\r' || line.back()==' ')) line.pop_back();
        if(!line.empty()) classes.push_back(line);
    }
    cout << "[dnn] loaded " << classes.size() << " class names\n";
}

void loadColors(string colorFilePath){
    //Load the colors
    string line;
    ifstream colorFptr(colorFilePath.c_str());
    if(!colorFptr.good()){
        cerr << "WARNING: cannot open color file: " << colorFilePath << "\n";
        return;
    }
    colors.clear();
    while(getline(colorFptr, line)){
        while(!line.empty() && (line.back()=='\r' || line.back()==' ')) line.pop_back();
        if(line.empty()) continue;

        //The original passed NULL as the end pointer for g and b, so g was
        //re-parsed from pEnd and b ended up equal to g. Thread the end pointer
        //through all three conversions.
        char* pEnd = nullptr;
        double r = strtod(line.c_str(), &pEnd);
        double g = strtod(pEnd,     &pEnd);
        double b = strtod(pEnd,     &pEnd);
        colors.push_back(Scalar((float)b, (float)g, (float)r));  // file is RGB, OpenCV wants BGR
    }
    cout << "[dnn] loaded " << colors.size() << " colors\n";
}

static Scalar colorFor(int classId){
    if(colors.empty()) return Scalar(255, 178, 50);
    return colors[classId % (int)colors.size()];
}

// Draw the predicted bounding box, colorize and show the mask on the image
void drawBox(Mat& frame, int classId, float conf, Rect box, Mat& objectMask)
{
    if(box.width <= 0 || box.height <= 0) return;

    Scalar color = colorFor(classId);

    //Draw a rectangle displaying the bounding box
    rectangle(frame, Point(box.x, box.y), Point(box.x+box.width, box.y+box.height), color, 3);

    //Get the label for the class name and its confidence
    string label = format("%.2f", conf);
    if (classId >= 0 && classId < (int)classes.size())
        label = classes[classId] + ":" + label;

    //Display the label at the top of the bounding box
    int baseLine;
    Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
    box.y = max(box.y, labelSize.height);
    rectangle(frame, Point(box.x, box.y - round(1.5*labelSize.height)),
                     Point(box.x + round(1.5*labelSize.width), box.y + baseLine),
              Scalar(255, 255, 255), FILLED);
    putText(frame, label, Point(box.x, box.y), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0,0,0), 1);

    if (objectMask.empty()) return;

    // Resize the mask, threshold, color and apply it on the image
    resize(objectMask, objectMask, Size(box.width, box.height));
    Mat mask = (objectMask > maskThreshold);
    mask.convertTo(mask, CV_8U);

    Mat coloredRoi = (0.3 * color + 0.7 * frame(box));
    coloredRoi.convertTo(coloredRoi, CV_8UC3);

    // Draw the contours on the image
    vector<Mat> contours;
    Mat hierarchy;
    findContours(mask, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);
    drawContours(coloredRoi, contours, -1, color, 1, LINE_8, hierarchy, 100);
    coloredRoi.copyTo(frame(box), mask);
}

// ---------------------------------------------------------------- YOLO11-seg
// outs[0]: [1, 4 + nc + nm, numAnchors]  -> cx, cy, w, h | class scores | mask coeffs
// outs[1]: [1, nm, mh, mw]                -> mask prototypes
void postprocess(Mat& frame, const vector<Mat>& outs)
{
    if (outs.size() < 2) {
        if (verbose) {
            cerr << "[dnn] postprocess got " << outs.size() << " output(s):";
            for (const Mat& o : outs) cerr << ' ' << o.size();
            cerr << "\n";
        }
        CV_Error(cv::Error::StsError,
                 "postprocess: expected 2 network outputs, got " + to_string(outs.size()));
    }

    if (verbose) {
        cerr << "[dnn] output0 dims=" << outs[0].dims << " type=" << outs[0].type() << " size=[";
        for (int i = 0; i < outs[0].dims; ++i) cerr << outs[0].size[i] << (i + 1 < outs[0].dims ? " x " : "");
        cerr << "]  output1 dims=" << outs[1].dims << " size=[";
        for (int i = 0; i < outs[1].dims; ++i) cerr << outs[1].size[i] << (i + 1 < outs[1].dims ? " x " : "");
        cerr << "]\n";
    }

    const Mat& out0 = outs[0];   // 1 x 116 x 8400
    const Mat& proto = outs[1];  // 1 x 32 x 160 x 160

    int total = out0.size[out0.dims - 2];
    int numAnchors = out0.size[out0.dims - 1];
    CV_Assert(out0.isContinuous());
    const float* data = out0.ptr<float>();

    int numClasses = total - 4 - 32;   // 32 mask coefficients in YOLO11-seg
    if (numClasses <= 0)
        CV_Error(cv::Error::StsError,
                 "postprocess: unexpected output width " + to_string(total));

    // ---- memory layout of out0 ----------------------------------------
    // The graph declares output0 as [1, 4+nc+nm, anchors] (channels major), but
    // an importer that mis-handles the graph's Transpose hands back those same
    // dims over an anchors-major buffer, i.e. [1, anchors, 4+nc+nm]. The dims
    // alone cannot tell the two apart and reading with the wrong stride yields
    // plausible-looking garbage (silly class names, scores far above 1).
    //
    // The class scores settle it: that branch of the graph ends in Sigmoid, so
    // every one of them must land in [0, 1] under the correct stride. Sample
    // the tensor and keep the layout that satisfies that. Cached, because it is
    // a property of the model, not of the frame.
    static int layout = -1;   // 0 = channels major, 1 = anchors major
    if (layout < 0) {
        float maxCM = 0.f, maxAM = 0.f;
        for (int a = 0; a < numAnchors; a += 97) {
            for (int c = 4; c < 4 + numClasses; ++c) {
                maxCM = max(maxCM, data[(size_t)c * numAnchors + a]);
                maxAM = max(maxAM, data[(size_t)a * total + c]);
            }
        }
        layout = (maxCM <= maxAM) ? 0 : 1;
        if (verbose)
            cerr << "[dnn] layout probe: channels-major max=" << maxCM
                 << " anchors-major max=" << maxAM
                 << " -> using " << (layout ? "anchors" : "channels") << " major\n";
    }

    // value of channel c for anchor a, under whichever layout won the probe
    auto at = [&](int a, int c) -> float {
        return layout == 0 ? data[(size_t)c * numAnchors + a]
                           : data[(size_t)a * total + c];
    };

    // Highest class score seen this frame, kept even when it falls below the
    // threshold so a quiet frame can be told apart from a broken one.
    float frameBest = 0.f;

    // --- collect candidates above threshold
    vector<Rect> boxes;
    vector<int>   classIds;
    vector<float> scores;
    vector<Mat>   coeffs;      // 32 x 1 per detection
    vector<int>   anchorIdx;

    for (int a = 0; a < numAnchors; ++a) {
        int   bestClass = -1;
        float bestScore = 0.f;
        for (int c = 0; c < numClasses; ++c) {
            float s = at(a, 4 + c);
            if (s > bestScore) { bestScore = s; bestClass = c; }
        }
        frameBest = max(frameBest, bestScore);
        if (bestClass < 0 || bestScore < confThreshold) continue;

        // cx, cy, w, h are in network-input pixel space (640x640)
        float cx = at(a, 0), cy = at(a, 1), w = at(a, 2), h = at(a, 3);
        float x1 = (cx - w * 0.5f) * ((float)frame.cols / 640.f);
        float y1 = (cy - h * 0.5f) * ((float)frame.rows / 640.f);
        float x2 = (cx + w * 0.5f) * ((float)frame.cols / 640.f);
        float y2 = (cy + h * 0.5f) * ((float)frame.rows / 640.f);

        x1 = max(0.f, min(x1, (float)frame.cols - 1.f));
        y1 = max(0.f, min(y1, (float)frame.rows - 1.f));
        x2 = max(0.f, min(x2, (float)frame.cols - 1.f));
        y2 = max(0.f, min(y2, (float)frame.rows - 1.f));

        boxes.push_back(Rect((int)x1, (int)y1, (int)(x2 - x1) + 1, (int)(y2 - y1) + 1));
        classIds.push_back(bestClass);
        scores.push_back(bestScore);
        anchorIdx.push_back(a);

        Mat c(32, 1, CV_32F);
        float* cp = c.ptr<float>();
        for (int k = 0; k < 32; ++k) cp[k] = at(a, 4 + numClasses + k);
        coeffs.push_back(c);
    }

    if (boxes.empty()) {
        if (verbose)
            cerr << "[dnn] nothing above " << confThreshold
                 << " (best score this frame " << frameBest << ")\n";
        return;
    }

    // --- NMS
    // signature: NMSBoxes(bboxes, scores, score_threshold, nms_threshold, indices, eta, top_k)
    vector<int> keep;
    NMSBoxes(boxes, scores, confThreshold, 0.45f, keep);

    if (verbose) {
        cerr << "[dnn] " << boxes.size() << " candidates above "
             << confThreshold << ", keeping " << keep.size() << " after NMS\n";
        // Dump the strongest few so the numbers can be sanity-checked.
        vector<int> byScore = keep;
        sort(byScore.begin(), byScore.end(),
             [&](int a, int b) { return scores[a] > scores[b]; });
        const int dump = min(6, (int)byScore.size());
        for (int i = 0; i < dump; ++i) {
            int k = byScore[i];
            const string& nm = classes[classIds[k]];
            cerr << "[dnn]   " << classIds[k] << " " << nm
                 << "  score=" << scores[k]
                 << "  box=[" << boxes[k].x << "," << boxes[k].y << " "
                 << boxes[k].width << "x" << boxes[k].height << "]"
                 << "  frame=" << frame.cols << "x" << frame.rows << "\n";
        }
    }

    // --- decode masks for surviving detections
    const int nm = proto.size[proto.dims - 3];
    const int mh = proto.size[proto.dims - 2];
    const int mw = proto.size[proto.dims - 1];

    Mat protoFlat = proto.reshape(1, nm);

    for (int idx : keep) {
        const float* cData = coeffs[idx].ptr<float>();
        Mat mask = coeffs[idx].t() * protoFlat;     // 1 x mh x mw
        mask = mask.reshape(1, mh).clone();          // mh x mw

        // crop to the box before resizing: saves work and kills stray bleed
        Rect box = boxes[idx] & Rect(0, 0, mw, mh);
        if (box.width > 0 && box.height > 0) {
            Rect scaled((int)(box.x * mw / (float)frame.cols),
                        (int)(box.y * mh / (float)frame.rows),
                        max(1, (int)(box.width  * mw / (float)frame.cols)),
                        max(1, (int)(box.height * mh / (float)frame.rows)));
            scaled &= Rect(0, 0, mw, mh);
            if (scaled.width > 0 && scaled.height > 0)
                mask = mask(scaled).clone();
        }

        drawBox(frame, classIds[idx], scores[idx], boxes[idx], mask);
    }
}

Mat detect(Mat& input, string modelPath, ModelFormat format){
    if (input.empty()) {
        cout << "ERROR: detect() called with an empty image\n";
        return input;
    }

    Net& net = cachedNet();
    if (net.empty() || cachedPath() != modelPath) {
        cout << "[dnn] loading " << modelPath << " (one time)\n";
        net = readNetFromONNX(modelPath);
        net.setPreferableBackend(DNN_BACKEND_OPENCV);
        net.setPreferableTarget(DNN_TARGET_CPU);
        cachedPath() = modelPath;
    }

    // Create a 4D blob from a frame.
    Mat blob;
    blobFromImage(input, blob, 1.0 / 255.0, Size(640, 640), Scalar(), true, false);

    //Sets the input to the network
    net.setInput(blob);

    // Runs the forward pass to get output from the output layers
    outNames[0] = "output0";   // [1, 116, 8400] boxes + class scores + mask coeffs
    outNames[1] = "output1";   // [1,  32, 160,160] mask prototypes
    net.forward(outs, outNames);

    // Shape sanity check. postprocess() indexes outs[0] as a flat
    // (total x numAnchors) array and outs[1] as (nm x mh x mw); if the graph
    // were mis-imported those indices would walk off the allocation, so fail
    // loudly here instead of reading garbage.
    if (outs.size() < 2) {
        cerr << "[dnn] network produced " << outs.size() << " output(s), need 2\n";
        CV_Error(cv::Error::StsError, "detect: model produced too few outputs");
    }
    {
        const Mat& d0 = outs[0];
        int channels = d0.dims >= 3 ? d0.size[d0.dims - 2] : d0.rows;
        int numAnchors = d0.dims >= 3 ? d0.size[d0.dims - 1] : d0.cols;
        if (d0.type() != CV_32F || numAnchors < 1000 || channels < 4 + 32) {
            cerr << "[dnn] unusable output0: dims=" << d0.dims << " type=" << d0.type()
                 << " channels=" << channels << " anchors=" << numAnchors
                 << " (want CV_32F, >=36 channels, >=1000 anchors)\n"
                 << "       Re-export the ONNX at a lower opset, e.g.\n"
                 << "       yolo export model=yolo11n-seg.pt format=onnx opset=12\n";
            CV_Error(cv::Error::StsError, "detect: unusable network output shape");
        }
    }

    // Extract the bounding box and mask for each of the detected objects
    postprocess(input, outs);

    return input;
}
