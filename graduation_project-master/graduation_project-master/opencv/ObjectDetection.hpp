//
//  ObjectDetection.hpp
//  opencv
//
//  Created by 심지훈 on 02/10/2019.
//  Copyright © 2019 Shim. All rights reserved.
//

#ifndef ObjectDetection_hpp
#define ObjectDetection_hpp

#include "detectionHeader.h"

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <iostream>

//The original code loaded a TensorFlow Mask R-CNN graph through
//readNetFromTensorflow(). That importer needs a TensorFlow-enabled OpenCV build
//and was removed outright in OpenCV 5, so the default backend is now a
//self-contained ONNX segmentation model (YOLO11-seg, ships in /models).
enum class ModelFormat {
    Auto,
    YoloSeg
};

void loadNameOfClasses(string classesFilePath);
void loadColors(string colorFilePath);

void drawBox(Mat& frame, int classId, float conf, Rect box, Mat& objectMask);
void postprocess(Mat& frame, const vector<Mat>& outs);

Mat detect(Mat& input, string modelPath, ModelFormat format = ModelFormat::Auto);

void setDetectionVerbose(bool on);

#endif /* ObjectDetection_hpp */
