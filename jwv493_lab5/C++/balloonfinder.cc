#include "balloonfinder.h"

#include <Eigen/Dense>
#include <cassert>
#include <opencv2/core/eigen.hpp>

#include "navtoolbox.h"

BalloonFinder::BalloonFinder(bool debuggingEnabled, bool calibrationEnabled,
                             const Eigen::Vector3d& blueTrue_I,
                             const Eigen::Vector3d& redTrue_I) {
  debuggingEnabled_ = debuggingEnabled;
  calibrationEnabled_ = calibrationEnabled;
  blueTrue_I_ = blueTrue_I;
  redTrue_I_ = redTrue_I;
  V_.resize(3, 0);
  W_.resize(3, 0);
}

// Returns true if the input contour touches the edge of the input image;
// otherwise returns false.
//
bool touchesEdge(const cv::Mat& image, const std::vector<cv::Point>& contour) {
  const size_t borderWidth = static_cast<size_t>(0.01 * image.rows);

  for (const auto& pt : contour) {
    if (pt.x <= borderWidth || pt.x >= (image.cols - borderWidth) ||
        pt.y <= borderWidth || pt.y >= (image.rows - borderWidth))
      return true;
  }
  return false;
}

Eigen::Vector3d BalloonFinder::eCB_calibrated() const {
  using namespace Eigen;
  const SensorParams sp;
  const size_t N = V_.cols();
  if (N < 2 || !calibrationEnabled_) {
    return Vector3d::Zero();
  }
  const VectorXd aVec = VectorXd::Ones(N);
  const Matrix3d dRCB = navtbx::wahbaSolver(aVec, W_, V_);
  const Matrix3d RCB = navtbx::euler2dc(sp.eCB());
  return navtbx::dc2euler(dRCB * RCB);
}

bool BalloonFinder::findBalloonsOfSpecifiedColor(
    const cv::Mat* image, const Eigen::Matrix3d RCI, const Eigen::Vector3d rc_I,
    const BalloonFinder::BalloonColor color,
    std::vector<Eigen::Vector2d>* rxVec) {
  using namespace cv;
  bool returnValue = false;
  rxVec->clear();
  Mat original;
  // Clone the original image for debugging purposes
  if (debuggingEnabled_) original = image->clone();
  const size_t nCols_m1 = image->cols - 1;
  const size_t nRows_m1 = image->rows - 1;
  // Blur the image to reduce small-scale noise
  Mat framep;
  GaussianBlur(*image, framep, Size(21, 21), 0, 0);

  // *************************************************************************
  //
  // Implement the rest of the function here.  Your goal is to find a balloon of
  // the color specified by the input 'color', and find its center in image
  // plane coordinates (see the comments below for a discussion on image plane
  // coordinates), expressed in pixels.  Suppose rx is an Eigen::Vector2d object
  // that holds the x and y position of a balloon center in image plane
  // coordinates.  You can push rx onto rxVec as follows: rxVec->push_back(rx)
  //
  // *************************************************************************

  // Convert Image to hsv (easier to mask for colors)
  cv::cvtColor(framep, framep, cv::COLOR_BGR2HSV);
  // Binary Frames using inRange() function

  cv::Scalar red_lower_l{0,60,100}; cv::Scalar red_lower_h{10, 255, 255};
  cv::Scalar red_upper_l{160, 60, 100}; cv::Scalar red_upper_h{179, 255, 255};
  cv::Scalar blue_l{90, 75, 60}; cv::Scalar blue_h{130, 255, 255};

  if(color == BalloonColor::RED){ //red
    cv::Mat matLower, matUpper;
    cv::inRange(framep, red_lower_l, red_lower_h, matLower);
    cv::inRange(framep, red_upper_l, red_upper_h, matUpper);
    framep = matLower | matUpper;
  }
  else if(color == BalloonColor::BLUE){ // blue
    cv::inRange(framep, blue_l, blue_h, framep);
  }

  // Erode and dialate (from lecture)
  int iters{20};
	cv::erode(framep, framep, cv::Mat(), cv::Point(-1,1), iters);
	cv::dilate(framep, framep, cv::Mat(), cv::Point(-1,-1), iters);  
  
  //find contours
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(framep, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  float maxAR{1.6}; // Aspect Ratio
  float minAR{1.15};
  float minR{35}; // Radius (px)
  float minArea{4000};

  float minCicularity{0.7};



  for(int ii{0}; ii < contours.size(); ii++){

    //find enclosing elipse
    cv::Point2f center{}; float radius{};
    cv::minEnclosingCircle(contours[ii], center, radius);

    // Find AR of box
    float aspectRatio{};
    int minPointsForElipse{5};
    if(contours[ii].size() >= minPointsForElipse){
      cv::RotatedRect boundingRectangle{cv::fitEllipse(contours[ii])};
      const cv::Size2f rectSize = boundingRectangle.size;
      aspectRatio = static_cast<float>(std::max(rectSize.width, rectSize.height)) /
            std::min(rectSize.width, rectSize.height);
    }
    const cv::Scalar color = cv::Scalar(255, 255, 255);
    cv::drawContours(original, contours, ii, color, 2, cv::LINE_8, hierarchy, 0);

    float area{}, perimeter{};
    float circularity{};
    area = cv::contourArea(contours[ii]);
    perimeter = cv::arcLength(contours[ii], true);

    if (perimeter > 0){
      circularity = 4 * 3.14 * area / (perimeter * perimeter);
    }

    std::vector<cv::Point> approx;
    double epsilon = 0.02f * cv::arcLength(contours[ii], true);
    cv::approxPolyDP(contours[ii], approx, epsilon, true);


    std::string label = "A: " + std::to_string(area) + "\n AR: " + std::to_string(aspectRatio);
    //std::string label = "AREA";
    cv::putText(original, label, center, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,255), 5, cv::LINE_AA);

    //cv::circle(original, center, static_cast<int>(50), cv::Scalar(0,0,255), 2);


    if (aspectRatio > minAR && aspectRatio < maxAR && radius > minR && minArea <= area && circularity >= minCicularity && approx.size() > 6) {
      // this means the bounding elipse is for a baloon (hopefully);
      cv::circle(original, center, static_cast<int>(radius), color, 2);
      cv::circle(original, center, static_cast<int>(25), color, -1);
      Eigen::Vector2d centerPoint;
      //((x - cx)/fx, (y - cy)/fy)
      // centerPoint[0] = (center.x - sensorParams_.cx())/sensorParams_.f();
      // centerPoint[1] = (center.y - sensorParams_.cy())/sensorParams_.f();
      centerPoint[0] = nCols_m1 - center.x;
      centerPoint[1] = nRows_m1 - center.y;
      rxVec->push_back(centerPoint);
      returnValue = true;
    }
  }



  // The debugging section below plots the back-projection of the true balloon
  // 3d location on the original image.  The balloon centers you find should be
  // close to the back-projected coordinates in xc_pixels.  Feel free to alter
  // the code in the debugging section below, or add other such sections, so
  // that you can see how your estimated balloon centers compare with the
  // back-projected centers.
  if (debuggingEnabled_) {
    Eigen::Vector2d xc_pixels;
    Scalar trueProjectionColor;
    if (color == BalloonColor::BLUE) {
      xc_pixels = backProject(RCI, rc_I, blueTrue_I_);
      trueProjectionColor = Scalar(255, 0, 0);
    } else {
      xc_pixels = backProject(RCI, rc_I, redTrue_I_);
      trueProjectionColor = Scalar(0, 0, 255);
    }
    Point2f center;
    // The image plane coordinate system, in which xc_pixels is expressed, has
    // its origin at the lower-right of the image, x axis pointing left and y
    // axis pointing up, whereas the variable 'center' below, used by OpenCV for
    // plotting on the image, is referenced to the image's top left corner and
    // has the opposite x and y directions.  The measurements returned in rxVec
    // should be given in the image plane coordinate system like xc_pixels.
    // Hence, once you've found a balloon center from your image processing
    // techniques, you'll need to convert it to the image plane coordinate
    // system using an inverse of the mapping below.
    center.x = nCols_m1 - xc_pixels(0);
    center.y = nRows_m1 - xc_pixels(1);
    circle(original, center, 20, trueProjectionColor, FILLED);
    namedWindow("Display", WINDOW_NORMAL);
    resizeWindow("Display", 1000, 1000);
    imshow("Display", original);
    waitKey(0);
  }
  return returnValue;
}

void BalloonFinder::findBalloons(
    const cv::Mat* image, const Eigen::Matrix3d RCI, const Eigen::Vector3d rc_I,
    std::vector<std::shared_ptr<const CameraBundle>>* bundles,
    std::vector<BalloonColor>* colors) {

  // Crop image to 4k size.  This removes the bottom 16 rows of the image,
  // which are an artifact of the camera API.
  const cv::Rect croppedRegion(0, 0, sensorParams_.imageWidthPixels(),
                               sensorParams_.imageHeightPixels());
  cv::Mat croppedImage = (*image)(croppedRegion);
  // Convert camera instrinsic matrix K and distortion parameters to OpenCV
  // format
  cv::Mat K, distortionCoeffs, undistortedImage;
  Eigen::Matrix3d Kpixels = sensorParams_.K() / sensorParams_.pixelSize();
  Kpixels(2, 2) = 1;
  cv::eigen2cv(Kpixels, K);
  cv::eigen2cv(sensorParams_.distortionCoeffs(), distortionCoeffs);
  // Undistort image
  cv::undistort(croppedImage, undistortedImage, K, distortionCoeffs);

  // Find balloons of specified color
  std::vector<BalloonColor> candidateColors = {BalloonColor::RED,
                                               BalloonColor::BLUE};
  for (auto color : candidateColors) {
    std::vector<Eigen::Vector2d> rxVec;
    if (findBalloonsOfSpecifiedColor(&undistortedImage, RCI, rc_I, color,
                                     &rxVec)) {
      for (const auto& rx : rxVec) {
        std::shared_ptr<CameraBundle> cb = std::make_shared<CameraBundle>();
        cb->RCI = RCI;
        cb->rc_I = rc_I;
        cb->rx = rx;
        bundles->push_back(cb);
        colors->push_back(color);
      }
    }
  }
}
