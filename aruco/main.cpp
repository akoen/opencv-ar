#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/aruco.hpp"

#include <sstream>
#include <iostream>
#include <fstream>
#include <math.h>

using namespace std;
using namespace cv;

const float calibrationSquareDimension = 0.01905f; //meters
const float arucoSquareDimension = 0.1016f; //meters
const Size chessboardDimensions = Size(6,9);

void createArucoMarkers() {
  Mat outputMarker;
  // Load marker dictionary
  Ptr<aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);

  for(int i = 0; i < 50; i++) {
    aruco::drawMarker(markerDictionary, i, 500, outputMarker, 1);
    ostringstream convert;
    string imageName = "4x4Marker_";
    convert << imageName << i << ".jpg";
    imwrite(convert.str(), outputMarker);
  }
}

  void createKnownBoardPosition(Size boardSize, float squareEdgeLength, vector<Point3f>& corners) {
    for(int i = 0; i< boardSize.height; i++) {
      for (int j = 0; j < boardSize.width; j++) {
        corners.push_back(Point3f(j * squareEdgeLength, i * squareEdgeLength, 0.0f));
      }
    }
  }

void getChessboardCorners(vector<Mat> images, vector<vector<Point2f>>& allFoundCorners, bool showResults = false) {
  for (vector<Mat>::iterator iter = images.begin(); iter != images.end(); iter++) {
    vector<Point2f> pointBuf;
    bool found = findChessboardCorners(*iter, Size(9,6), pointBuf, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);

    if(found) {
      allFoundCorners.push_back(pointBuf);
    }

    if(showResults) {
      drawChessboardCorners(*iter, Size(9,6), pointBuf, found);
      imshow("Looking for Corners", *iter);
      waitKey(0);
    }
  }
}

int startWebcamMonitoring(const Mat& cameraMatrix, const Mat& distanceCoefficients, float arucoDimensions) {
  Mat frame;

  vector<int> markerIds;
  vector<vector<Point2f>> markerCorners, rejectedCandidates;
  Ptr<aruco::DetectorParameters> parameters = aruco::DetectorParameters::create();

  Ptr<aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);

  Mat im_src = imread("pete.jpg"); // Source image

  VideoCapture vid(0);

  if(!vid.isOpened()) {
    return -1;
  }

  namedWindow("Webcam", WINDOW_AUTOSIZE);

  vector<Vec3d> rotationVectors, translationVectors;

  while(true) {
    if(!vid.read(frame))
      break;

    aruco::detectMarkers(frame, markerDictionary, markerCorners, markerIds, parameters, rejectedCandidates);
    aruco::estimatePoseSingleMarkers(markerCorners, arucoSquareDimension, cameraMatrix, distanceCoefficients, rotationVectors, translationVectors);


    vector<Point> pts_src;
    pts_src.push_back(Point(0,0));
    pts_src.push_back(Point(im_src.cols, 0));
    pts_src.push_back(Point(im_src.cols, im_src.rows));
    pts_src.push_back(Point(0, im_src.rows));

    // Display axes on markers
    // vector<Point> pts_dst;
    for(int i = 0; i < markerIds.size(); i++) {
      aruco::drawAxis(frame, cameraMatrix, distanceCoefficients, rotationVectors[i], translationVectors[i], 0.1f);

      // pts_dst.push_back(markerCorners[i][0]);

    }

    // Using the detected markers, locate the quadrilateral on the target frame where the new scene is going to be displayed.
    vector<Point> pts_dst;
    
    if(markerIds.size() == 4) {

      Point imageTL {0, 0};
      Point imageTR {frame.cols, 0};
      Point imageBL {0, frame.rows};
      Point imageBR {frame.cols, frame.rows};
      
      // Get corners of markers
      vector<Point2f> corners;
      int tLeft, tRight, bLeft, bRight;
      
      for (int i{0}; i < 4; i++) {
        for(int c{0}; c < 4; c++) {
          corners.push_back(markerCorners[i][c]);
        }
      }

      // Keep only 4 corner points
      int totalX{0};
      int totalY{0};

      for(int i{0}; i < corners.size(); i++) {
        totalX += corners[i].x;
        totalY += corners[i].y;
      }

      int avgX = totalX / corners.size();
      int avgY = totalY / corners.size();

      Point center {avgX, avgY};

      circle(frame, center, 5, Scalar(255, 255, 150));

      // vector<int> distancesToCenter;
      // for(int i{0}; i < corners.size(); i++) {
      //   distancesToCenter.push_back(pow(center.x - corners[i].x, 2) + pow(center.y - corners[i].y, 2));
      // }

      // for(int r{0}; r < 12; r++) {
      //   corners.erase(corners.begin() + (min_element(distancesToCenter.begin(), distancesToCenter.end()) - distancesToCenter.begin()));
      // } 

      // Find points closest to corners
      vector<int> distancesToTL;
      for(int i{0}; i < corners.size(); i++) {
        distancesToTL.push_back(pow(corners[i].x, 2) + pow(corners[i].y, 2));
      }

      pts_dst.push_back(corners[min_element(distancesToTL.begin(), distancesToTL.end()) - distancesToTL.begin()]);

      vector<int> distancesToTR;
      for(int i{0}; i < corners.size(); i++) {
        distancesToTR.push_back(pow(frame.cols - corners[i].x, 2) + pow(corners[i].y, 2));
      }

      pts_dst.push_back(corners[min_element(distancesToTR.begin(), distancesToTR.end()) - distancesToTR.begin()]);

      vector<int> distancesToBR;
      for(int i{0}; i < corners.size(); i++) {
        distancesToBR.push_back(pow(frame.cols - corners[i].x, 2) + pow(frame.rows - corners[i].y, 2));
      }

      pts_dst.push_back(corners[min_element(distancesToBR.begin(), distancesToBR.end()) - distancesToBR.begin()]);

      vector<int> distancesToBL;
      for(int i{0}; i < corners.size(); i++) {
        distancesToBL.push_back(pow(corners[i].x, 2) + pow(frame.rows - corners[i].y, 2));
      }

      pts_dst.push_back(corners[min_element(distancesToBL.begin(), distancesToBL.end()) - distancesToBL.begin()]);


      Mat h = findHomography(pts_src, pts_dst);

      Mat im_out;
      warpPerspective(im_src, im_out, h, frame.size());
      fillConvexPoly(frame, pts_dst, 0, 16);

      frame = frame + im_out;
    }
    
    // Show enhanced image
    imshow("Webcam", frame);
    if(waitKey(30) >= 0) break;
  }

  return 1;
}

void cameraCalibration(vector<Mat> calibrationImages, Size boardSize, float squareEdgeLength, Mat& cameraMatrix, Mat& distanceCoefficients) {
  vector<vector<Point2f>> checkerboardImageSpacePoints;
  getChessboardCorners(calibrationImages, checkerboardImageSpacePoints, false);

  vector<vector<Point3f>> worldSpaceCornerPoints(1);

  createKnownBoardPosition(boardSize, squareEdgeLength, worldSpaceCornerPoints[0]);
  worldSpaceCornerPoints.resize(checkerboardImageSpacePoints.size(), worldSpaceCornerPoints[0]);

  vector<Mat> rVectors, tVectors;
  distanceCoefficients = Mat::zeros(8, 1, CV_64F);

  calibrateCamera(worldSpaceCornerPoints, checkerboardImageSpacePoints, boardSize, cameraMatrix, distanceCoefficients, rVectors, tVectors);
}

bool saveCameraCalibration(string name, Mat cameraMatrix, Mat distanceCoefficients) {
  ofstream outStream(name);
  if(outStream) {
    uint16_t rows = cameraMatrix.rows;
    uint16_t columns = cameraMatrix.cols;

    outStream << rows << endl;
    outStream << columns << endl;

    for(int r = 0; r < rows; r++) {
      for(int c = 0; c < columns; c++) {
        double value = cameraMatrix.at<double>(r,c);
        outStream << value << endl;
      }
    }

    rows = distanceCoefficients.rows;
    columns = distanceCoefficients.cols;

    outStream << rows << endl;
    outStream << columns << endl;

    for(int r = 0; r < rows; r++) {
      for(int c = 0; c < columns; c++) {
        double value = distanceCoefficients.at<double>(r,c);
        outStream << value << endl;
      }
    }

    outStream.close();
    return true;
  }

  return false;
}

bool loadCameraCalibration(string name, Mat& cameraMatrix, Mat& distanceCoefficients) {
  ifstream inStream(name);
  if(inStream) {
    uint16_t rows;
    uint16_t columns;

    inStream >> rows;
    inStream >> columns;

    cameraMatrix = Mat(Size(columns, rows), CV_64F);

    for(int r = 0; r < rows; r++) {
      for(int c = 0; c < columns; c++) {
        double read = 0.0f;
        inStream >> read;
        cameraMatrix.at<double>(r,c) = read;
        cout << cameraMatrix.at<double>(r,c) << "\n";
      }
    }
    // Distance Coefficients
    inStream >> rows;
    inStream >> columns;

    distanceCoefficients = Mat::zeros(rows, columns, CV_64F);

    for(int r = 0; r < rows; r++) {
      for(int c = 0; c < columns; c++) {
        double read = 0.0f;
        inStream >> read;
        distanceCoefficients.at<double>(r,c) = read;
        cout << distanceCoefficients.at<double>(r,c) << "\n";
      }
    }
    inStream.close();
    return true;
  }

  return false;
}

void cameraCalibrationProcess(Mat& cameraMatrix, Mat& distanceCoefficients){
  Mat frame;
  Mat drawToFrame;

  vector<Mat> savedImages;

  vector<vector<Point2f>> markerCorners, rejectedCandidates;

  VideoCapture vid(0);

  if(!vid.isOpened()) {
    return;
  }

  int framesPerSecond = 20;

  namedWindow("Webcam", WINDOW_AUTOSIZE);

  while(true) {
    if(!vid.read(frame))
      break;

    vector<Vec2f> foundPoints;
    bool found = false;

    found = findChessboardCorners(frame, chessboardDimensions, foundPoints, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
    frame.copyTo(drawToFrame);
    drawChessboardCorners(drawToFrame, chessboardDimensions, foundPoints, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);

    if(found)
      imshow("Webcam", drawToFrame);
    else
      imshow("Webcam", frame);

    char character = waitKey(1000 / framesPerSecond);

    switch(character) {
    case ' ':
      //save image
      if(found) {
        Mat temp;
        frame.copyTo(temp);
        savedImages.push_back(temp);
        cout << "Image registered";
      }
      break;
    case 13: // Return
      // start calibration
      if(savedImages.size() > 15)
        {
          cameraCalibration(savedImages, chessboardDimensions, calibrationSquareDimension, cameraMatrix, distanceCoefficients);
          saveCameraCalibration("calibration-data.txt", cameraMatrix, distanceCoefficients);
          cout << "Camera calibrated" << endl;
        }
      break;
    case 27: // ESC
      //exit
      return;
    }
  }
}

int main(int argv, char** argc) {

  Mat cameraMatrix = Mat::eye(3,3, CV_64F);

  Mat distanceCoefficients;

  // cameraCalibrationProcess(cameraMatrix, distanceCoefficients);
  loadCameraCalibration("calibration-data.txt", cameraMatrix, distanceCoefficients);
  startWebcamMonitoring(cameraMatrix, distanceCoefficients, 0.011f);

  return 0;
}
