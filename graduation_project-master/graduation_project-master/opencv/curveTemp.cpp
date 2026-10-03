//    Mat bottom_half_y = sobel.rowRange(sobel.size().height / 2, sobel.size().height);
//    Mat hist;
//    reduce(bottom_half_y, hist, 0, REDUCE_SUM,CV_32FC1);
//
//    int midPoint = (int)hist.cols / 2;
//    Point leftx_base;
//    Point rightx_base;
//
//    Mat bottomLeft = hist.colRange(0, midPoint);
//    Mat bottomRight = hist.colRange(midPoint, hist.cols);
//
//    minMaxLoc(bottomLeft, nullptr, nullptr, nullptr, &leftx_base);
//    minMaxLoc(bottomRight, nullptr, nullptr, nullptr, &rightx_base);
//
//    int nwindows = 9;
//    int windowHeight = (int)sobel.size().height / nwindows;
//
//    vector<Point> nonzero;
//    findNonZero(sobel, nonzero);
//
//    Point leftx_current = leftx_base;
//    rightx_base.x += midPoint;
//    Point rightx_current = rightx_base;
//
//    int margin = 50;
//
//    vector<Point> good_left_idx;
//    vector<Point> good_right_idx;
//    Scalar left_mean;
//    Scalar right_mean;
//
//    Mat merge[3] = {sobel, sobel, sobel};
//    Mat out;
//    cv::merge(merge, 3, out);
//
//    for(int window =0 ; window < nwindows; ++window){
//        int win_y_low = sobel.size().height - (window + 1) *  windowHeight;
//        int win_y_high = sobel.size().height - window * windowHeight;
//
//        int win_xleft_low = leftx_current.x - margin;
//        int win_xleft_high = leftx_current.x + margin;
//
//        int win_xright_low = rightx_current.x - margin;
//        int win_xright_high = rightx_current.x + margin;
//
//        rectangle(out, Point(win_xleft_low, win_y_low), Point(win_xleft_high, win_y_high), Scalar(0,255,0));
//        rectangle(out, Point(win_xright_low, win_y_low), Point(win_xright_high,win_y_high), Scalar(0,255,0));
//
//        vector<Point> leftIdx;
//        vector<Point> rightIdx;
//        for(auto _nonzero : nonzero){
//            if((_nonzero.y >= win_y_low) && (_nonzero.y < win_y_high) && (_nonzero.x >= win_xleft_low)
//               && ( _nonzero.x < win_xleft_high)){
//                leftIdx.push_back(_nonzero);
//                good_left_idx.push_back(_nonzero);
//            }
//
//            if((_nonzero.y >= win_y_low) && (_nonzero.y < win_y_high) && (_nonzero.x >= win_xright_low)
//               && ( _nonzero.x < win_xright_high)){
//                rightIdx.push_back(_nonzero);
//                good_right_idx.push_back(_nonzero);
//            }
//        }
//        left_mean = mean(leftIdx);
//        right_mean = mean(rightIdx);
//
//        if( left_mean != Scalar(0,0,0,0))
//            leftx_current = Point(left_mean[0], left_mean[1] );
//
//        if( right_mean != Scalar(0,0,0,0) )
//            rightx_current = Point(right_mean[0], right_mean[1]);
//
//    }
//
//
//
//
//    vector<float> leftx, lefty;
//    vector<float> rightx, righty;
//    for(auto _good_left : good_left_idx){
//        leftx.push_back(_good_left.x);
//        lefty.push_back(_good_left.y);
//    }
//
//    for(auto _good_right : good_right_idx){
//        rightx.push_back(_good_right.x);
//        righty.push_back(_good_right.y);
//    }
//
//    int leftRow = (int)leftx.size();
//    Mat leftxMat(leftRow, 1 , CV_32FC1, leftx.data());
//    Mat leftyMat(leftRow, 1, CV_32FC1, lefty.data());
//    Mat leftDst(3,1, CV_32FC1 );
//    polyfit(leftyMat, leftxMat, leftDst, 2);
//
//    int rightRow = (int)rightx.size();
//    Mat rightxMat(rightRow,1, CV_32FC1, rightx.data());
//    Mat rightyMat(rightRow, 1, CV_32FC1, righty.data());
//    Mat rightDst(3,1, CV_32FC1 );
//    polyfit(rightyMat, rightxMat, rightDst, 2);
//
//    float leftCoef[3] = {leftDst.at<float>(2,0), leftDst.at<float>(1,0), leftDst.at<float>(0,0)};
//    float rightCoef[3] = {rightDst.at<float>(2,0), rightDst.at<float>(1, 0), rightDst.at<float>(0,0) };
//    float leftFitx,rightFitx;
//    vector<Point> leftFitPt, rightFitPt;
//    for(int i=0; i<out.size().height; ++i){
//        leftFitx = leftCoef[0] * i * i + leftCoef[1] * i + leftCoef[2];
//        rightFitx = rightCoef[0] * i * i + rightCoef[1] * i + rightCoef[2];
//        leftFitPt.push_back(Point(leftFitx, i));
//        rightFitPt.push_back(Point(rightFitx, i));
//    }
//    polylines(out, leftFitPt, false, Scalar(0,255,255));
//    polylines(out, rightFitPt, false, Scalar(0,255,255));
