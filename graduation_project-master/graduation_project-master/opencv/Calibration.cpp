//
//  Calibration.cpp
//  opencv
//
//  Created by 심지훈 on 11/08/2019.
//  Copyright © 2019 Shim. All rights reserved.
//

#include "Calibration.hpp"
#include "Utility.hpp"

Calibration::Calibration(Mat intrinsic, Mat distCoeffs):intrinsic(intrinsic), distCoeffs(distCoeffs)
{
    if(this->intrinsic.empty() || this->distCoeffs.empty())
        cout<<"ERROR : Camera Matrix is empty\n";
}

Calibration::Calibration() { }

void Calibration::cameraCalibration(String chessBoardPath){
    
    vector<Mat> chessBoardImages;
    
    //chessBoardPath is treated as a glob pattern so callers can point at
    //"camera_cal/calibration*.jpg" instead of having to rename the files to
    //1.jpg..15.jpg as the original hardcoded loop required.
    vector<String> matches;
    if (String(chessBoardPath).find('*') != String::npos) {
        matches = vector<String>();
        glob(String(chessBoardPath), matches, false);
        sort(matches.begin(), matches.end());
    } else {
        for(int num = 1; num<=numBoards; ++num) {
            String candidate = chessBoardPath + to_string(num) + ".jpg";
            if (fileExists(candidate)) matches.push_back(candidate);
        }
    }
    
    if (matches.empty()) {
        cerr << "ERROR: no chessboard images matched: " << chessBoardPath << "\n";
        return;
    }
    
    for(const String& path : matches) {
        Mat temp = imread(path);
        if (temp.empty()) {
            cerr << "WARNING: could not read " << path << ", skipping\n";
            continue;
        }
        resize(temp, temp, this->size);
        chessBoardImages.push_back(temp);
    }
    
    cout << "Loaded " << chessBoardImages.size() << " chessboard images\n";
    
    vector<vector<Point3f>> object_points;
    vector<vector<Point2f>> image_points;
    vector<Point2f> corners;
    int success = 0;
    
    Mat gray_image;
    
    vector<Point3f> obj;
    for(int j=0; j<numSquares; ++j)
        obj.push_back(Point3f((float)(j / numCornersHor), (float)(j % numCornersHor), 0.0f));
    
    
    vector<Mat>::iterator pos = chessBoardImages.begin();
    
    //The original code hoisted "image = *pos" out of the loop and never
    //advanced it, so calibrateCamera() ended up fed the same first image
    //numBoards times. Iterate properly instead.
    for(; pos != chessBoardImages.end() && success < numBoards; ++pos) {
        Mat image = *pos;
        
        cvtColor(image, gray_image, COLOR_BGR2GRAY);
        bool found = findChessboardCorners(image, this->boardSize, corners);
        if(found)
            drawChessboardCorners(gray_image, this->boardSize, corners, found);
        
        if(found != 0){
            image_points.push_back(corners);
            object_points.push_back(obj);
            
            cout<<"Snap Stored! (" << success+1 << "/" << numBoards << ")\n";
            success++;
        }
    }
    
    if(image_points.empty()){
        cerr << "ERROR: chessboard corners found in 0 images, cannot calibrate\n";
        return;
    }
    
    Mat image = chessBoardImages.front();
    
    vector<Mat> rvecs;
    vector<Mat> tvecs;
    
    intrinsic.ptr<float>(0)[0] = 1;
    intrinsic.ptr<float>(1)[1] = 1;
    
    double rms = calibrateCamera(object_points, image_points, image.size(), intrinsic, distCoeffs, rvecs, tvecs);
    cout << "Calibrated on " << success << " snaps, RMS reprojection error = " << rms << "\n";
    
    if(!intrinsic.empty())
        cout<<"Intrinsic Done\n";
    if(!distCoeffs.empty())
        cout<<"DistCoeffs Done\n";
    
    
}

void Calibration::saveCameraMatrix(String filePath){
    cv::FileStorage fs(filePath+"cameraMatrix.xml", FileStorage::WRITE);
    fs<<"camera_matrix"<<intrinsic<<"distortion_coefficients"<<distCoeffs;
    fs.release();
    
}

void Calibration::loadCameraMatrix(String filePath, String name){
    cv::FileStorage fs(filePath+name, FileStorage::READ);
    fs["camera_matrix"]>>this->intrinsic;
    fs["distortion_coefficients"]>>this->distCoeffs;
    fs.release();
}

Mat Calibration::getUndistortedImg(Mat input){
    
    Mat img;
    if(input.size() != this->size )
        resize(input, input, this->size);

    undistort(input, img, this->intrinsic, this->distCoeffs);
    return img;
}
