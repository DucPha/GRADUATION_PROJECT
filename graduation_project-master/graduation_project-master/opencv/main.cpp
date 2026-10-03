//
//  main.cpp
//  opencv
//
//  Created by 심지훈 on 31/03/2019.
//  Copyright © 2019 Shim. All rights reserved.
//

#include <string>
#include <filesystem>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

#include "AdvancedLaneDetection.hpp"
#include "Calibration.hpp"
#include "ObjectDetection.hpp"
#include "Utility.hpp"

//    Order of Pipeline
//    Mat trans = detector.transformingView(undis, BIRDEYE_VIEW);
//    Mat sobel = detector.sobelColorThresholding(trans);
//    Mat curve = detector.windowSearch(sobel);
//    Mat temp = detector.transformingView(curve, NORMAL_VIEW);
//    Mat output = detector.drawPolyArea(img);

static void printUsage(const char* argv0) {
    cout <<
        "Usage: " << argv0 << " [options]\n\n"
        "  --video <path>      input clip            (default: ../Advanced-Lane-Lines-master/project_video.mp4)\n"
        "  --out <dir>         output directory      (default: ./output_cpp)\n"
        "  --model <path>      ONNX segmentation net (default: ../models/yolo11n-seg.onnx)\n"
        "  --classes <path>    class names           (default: ../models/mscoco_labels.names)\n"
        "  --colors <path>     per-class BGR colors  (default: ../models/colors.txt)\n"
        "  --codec <name>      mp4v|avc1|mjpg|xvid|auto  (default: auto)\n"
        "  --verbose           print DNN output shapes and detection counts\n"
        "  --no-dnn            lane detection only, skip object detection\n"
        "  --preview           open a window for every intermediate stage\n"
        "  --max-frames <n>    stop after n frames   (default: 0 = whole clip)\n"
        "  --fps <f>           output frame rate      (default: 30, 0 = match source)\n"
        "  -h, --help          this message\n";
}

// Which fourcc actually works depends entirely on how the FFMPEG plugin in the
// installed OpenCV was built -- the prebuilt Windows package has no MPEG-4
// encoder, so the hardcoded 'MP4V' from the original code silently produced
// 0-byte files. Probe the candidates and report what we got.
struct WriterChoice {
    string ext;
    int    fourcc;
    string label;
};

static vector<WriterChoice> codecCandidates(const string& requested) {
    vector<WriterChoice> all = {
        {".mp4", VideoWriter::fourcc('m','p','4','v'), "mp4v/mp4"},
        {".mp4", VideoWriter::fourcc('a','v','c','1'), "avc1/mp4"},
        {".mp4", VideoWriter::fourcc('M','J','P','G'), "mjpg/mp4"},
        {".avi", VideoWriter::fourcc('M','J','P','G'), "mjpg/avi"},
        {".avi", VideoWriter::fourcc('X','V','I','D'), "xvid/avi"},
    };
    if (requested.empty() || requested == "auto") return all;

    vector<WriterChoice> picked;
    for (const WriterChoice& c : all) {
        string name = c.label.substr(0, c.label.find('/'));
        if (name == requested) picked.push_back(c);
    }
    if (picked.empty()) {
        cerr << "ERROR: unknown codec '" << requested
             << "' (use mp4v, avc1, mjpg, xvid or auto)\n";
        exit(1);
    }
    return picked;
}

static bool openWriter(VideoWriter& w, const string& dir, const string& stem,
                       const vector<WriterChoice>& choices, double fps, Size size,
                       WriterChoice* used) {
    for (const WriterChoice& c : choices) {
        w.open(dir + stem + c.ext, c.fourcc, fps, size);
        if (w.isOpened()) {
            *used = c;
            return true;
        }
        w.release();
    }
    return false;
}

int main(int argc, const char* argv[]) {
    // Defaults are relative to the project root so the program runs from
    // anywhere inside the repo.
    const string root = "..";

    string video = root + "/Advanced-Lane-Lines-master/project_video.mp4";
    string outputPath = "./output_cpp/";
    string modelPath = root + "/models/yolo11n-seg.onnx";
    string colorPath = root + "/models/colors.txt";
    string labelPath = root + "/models/mscoco_labels.names";

    bool dnnEnabled = true;
    bool preview = false;
    int  maxFrames = 0;
    double fps = 30.0;
    string codec = "auto";
    bool verbose = false;

    for (int i = 1; i < argc; ++i) {
        string a = argv[i];
        auto need = [&](const char* what) -> string {
            if (i + 1 >= argc) {
                cerr << "ERROR: " << what << " requires a value\n";
                exit(1);
            }
            return argv[++i];
        };

        if      (a == "-h" || a == "--help")    { printUsage(argv[0]); return 0; }
        else if (a == "--video")                video       = need("--video");
        else if (a == "--out")                  outputPath  = need("--out");
        else if (a == "--model")                modelPath   = need("--model");
        else if (a == "--classes")              labelPath   = need("--classes");
        else if (a == "--colors")               colorPath   = need("--colors");
        else if (a == "--max-frames")           maxFrames   = stoi(need("--max-frames"));
        else if (a == "--fps")                  fps         = stod(need("--fps"));
        else if (a == "--codec")                codec       = need("--codec");
        else if (a == "--verbose")              verbose     = true;
        else if (a == "--no-dnn")               dnnEnabled  = false;
        else if (a == "--preview")              preview     = true;
        else {
            cerr << "ERROR: unknown option '" << a << "'\n";
            // A path holding a space that reached us unquoted (a shell or
            // launcher that split it) shows up here as a bare path fragment,
            // which is otherwise a baffling error message.
            if (a.find('\\') != string::npos || a.find('/') != string::npos)
                cerr << "       this looks like part of a file path: put the "
                        "value in quotes, e.g. --video \"C:\\path with "
                        "spaces\\clip.mp4\"\n";
            printUsage(argv[0]);
            return 1;
        }
    }

    if (!fileExists(video)) {
        string alt = firstExisting({
            "../Advanced-Lane-Lines-master/project_video.mp4",
            "Advanced-Lane-Lines-master/project_video.mp4",
            "../Advanced-Lane-Lines-master/challenge_video.mp4",
        });
        if (alt.empty()) {
            cerr << "ERROR: video not found: " << video << "\n"
                 << "       pass one with --video <path>\n";
            return 1;
        }
        cout << "[info] using " << alt << " instead\n";
        video = alt;
    }

    if (dnnEnabled) {
        if (!fileExists(modelPath) || !fileExists(labelPath) || !fileExists(colorPath)) {
            cout << "[warn] DNN assets missing, continuing with lane detection only.\n"
                 << "       model : " << modelPath << "\n"
                 << "       labels: " << labelPath << "\n"
                 << "       colors: " << colorPath << "\n";
            dnnEnabled = false;
        }
    }

    if (outputPath.back() != '/' && outputPath.back() != '\\')
        outputPath += '/';

    // VideoWriter::open() fails silently when the directory is missing, so
    // create it up front and report the failure instead of writing nothing.
    {
        std::error_code ec;
        std::filesystem::create_directories(outputPath, ec);
        if (ec) {
            cerr << "ERROR: cannot create output directory " << outputPath
                 << ": " << ec.message() << "\n";
            return 1;
        }
    }

    Size size(960, 540);

    VideoCapture capture(video);
    if (!capture.isOpened()) {
        cerr << "ERROR: cannot open video: " << video << "\n";
        return 1;
    }

    if (dnnEnabled) {
        loadColors(colorPath);
        loadNameOfClasses(labelPath);
        setDetectionVerbose(verbose);
    }

    // fps <= 0 means "match the source clip". A writer opened at 0 fps still
    // produces a file, but the muxer writes a degenerate time base, and every
    // later reader stops after the first sample -- so resolve it to a real
    // number before anything touches VideoWriter.
    if (fps <= 0) {
        double srcFps = capture.get(CAP_PROP_FPS);
        fps = (srcFps > 0.1 && srcFps < 1000) ? srcFps : 30.0;
        if (verbose)
            cout << "[info] source fps -> " << fps << "\n";
    }

    AdvnacedLaneDetection detector;

    int delay = (int)(1000.0 / fps + 0.5);

    Mat frame;
    Mat output = Mat::zeros(size, CV_8UC3);

    VideoWriter lane_vid, sobel_vid, curve_vid, trans_vid, detected_vid;
    vector<WriterChoice> choices = codecCandidates(codec);
    WriterChoice used;
    if (!openWriter(sobel_vid, outputPath, "sobel", choices, fps, size, &used)) {
        cerr << "ERROR: no usable video codec found in this OpenCV build.\n"
             << "       tried:";
        for (const WriterChoice& c : choices) cerr << ' ' << c.label;
        cerr << "\n       try --codec mjpg or --codec xvid\n";
        return 1;
    }
    openWriter(lane_vid,     outputPath, "lane",     choices, fps, size, &used);
    openWriter(curve_vid,    outputPath, "curve",    choices, fps, size, &used);
    openWriter(trans_vid,    outputPath, "trans",    choices, fps, size, &used);
    openWriter(detected_vid, outputPath, "detected", choices, fps, size, &used);

    string videoExt = used.ext;

    cout << "[info] video    : " << video << "\n"
         << "[info] output   : " << outputPath << " (codec " << used.label << ")\n"
         << "[info] dnn      : " << (dnnEnabled ? modelPath : string("disabled")) << "\n"
         << "[info] frame size: " << size.width << "x" << size.height << "\n";

    long frameNo = 0;
    long emptyLanes = 0;

    while (capture.read(frame)) {
        if (maxFrames > 0 && frameNo >= maxFrames) break;

        resize(frame, frame, size);

        Mat trans = detector.transformingView(frame, BIRDEYE_VIEW);
        Mat sobel = detector.sobelColorThresholding(trans);

        // Mask out the vehicle bonnet.
        rectangle(sobel, Point(300, 350), Point(600, 540), Scalar(0), -1);

        // sobel is the single-channel AND of the sobel and colour thresholds.
        // The writers (and imshow) want 3 channels, so keep a BGR copy for
        // output and hand windowSearch the binary original.
        Mat sobelBgr;
        if (sobel.channels() == 1) cvtColor(sobel, sobelBgr, COLOR_GRAY2BGR);
        else                        sobelBgr = sobel;

        Mat curve = detector.windowSearch(sobel);
        if (detector.getFitPointCount() == 0) {
            emptyLanes++;
            continue;  // no lane pixels in this frame; drawing would assert
        }

        Mat lane = detector.drawPolyArea(frame);
        if (dnnEnabled) {
            try {
                // Run the network on the clean frame, not on `lane`.
                // drawPolyArea tints the whole lane corridor green, and feeding
                // that back in costs real accuracy -- scores on project_video
                // drop to ~0.25-0.4 with no vehicle ever crossing the
                // threshold. detect() also annotates in place, so hand it a
                // copy to keep `frame` and `lane` intact.
                output = frame.clone();
                detect(output, modelPath);
            } catch (const cv::Exception& e) {
                cerr << "[error] DNN inference failed: " << e.what() << "\n";
                dnnEnabled = false;
                output = lane;
            }
        } else {
            output = lane;
        }

        trans_vid.write(trans);
        sobel_vid.write(sobelBgr);
        curve_vid.write(curve);
        lane_vid.write(lane);
        detected_vid.write(output);

        if (preview) {
            imshow("trans", trans);
            imshow("sobel", sobelBgr);
            imshow("curve", curve);
            imshow("lane", lane);
            if (dnnEnabled) imshow("detected", output);
        }

        if (waitKey(delay) == 'q') break;

        frameNo++;
    }

    capture.release();
    lane_vid.release();
    sobel_vid.release();
    curve_vid.release();
    trans_vid.release();
    detected_vid.release();

    cout << "[done] processed " << frameNo << " frames";
    if (emptyLanes) cout << " (" << emptyLanes << " skipped: no lane pixels)";
    cout << "\n[done] videos written to " << outputPath << " as *"
         << videoExt.substr(1) << "\n";

    return 0;
}
