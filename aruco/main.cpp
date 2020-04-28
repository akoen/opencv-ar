#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/aruco.hpp"

#include <sstream>
#include <iostream>
#include <fstream>
#include <string>

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
    for(int i = 0; i < markerIds.size(); i++) {
      // aruco::drawAxis(frame, cameraMatrix, distanceCoefficients, rotationVectors[i], translationVectors[i], 0.1f);
      aruco::drawDetectedMarkers(frame, markerCorners);

      vector<Point> pts_dst;
      pts_dst.push_back(markerCorners[i][0]);
      pts_dst.push_back(markerCorners[i][1]);
      pts_dst.push_back(markerCorners[i][2]);
      pts_dst.push_back(markerCorners[i][3]);

      Mat h = findHomography(pts_src, pts_dst);

      Mat im_out;
      warpPerspective(im_src, im_out, h, frame.size());
      fillConvexPoly(frame, pts_dst, 0, 16);

      frame = frame + im_out;
      
    }

    // vector<Point> pts_dst;
    // float scalingFac = 0.02;

    // Point refPt1, refPt2, refPt3, refPt4;

    // vector<int>::iterator it = find(markerIds.begin(), markerIds.end(), 25);
    // int index = distance(markerIds.begin(), it);
    // refPt1 = markerCorners.at(index).at(1);

    // // finding top right corner point of the target quadrilateral
    // it = find(markerIds.begin(), markerIds.end(), 33);
// index = distance(markerIds.begin(), it);
      // refPt2 = markerCorners.at(index).at(2);

      // float distance = norm(refPt1 - refPt2);
        // pts_dst.push_back(Point(refPt1.x - round(scalingFac * distance), refPt1.y - round(scalingFac * distance)));
        // pts_dst.push_back(Point(refPt2.x - round(scalingFac * distance), refPt2.y - round(scalingFac * distance)));

        // // finding bottom right corner point of the target quadrilateral
        // it = find(markerIds.begin(), markerIds.end(), 30);
        // index = std::distance(markerIds.begin(), it);
        // refPt3 = markerCorners.at(index).at(0);
        // pts_dst.push_back(Point(refPt3.x + round(scalingFac*distance), refPt3.y + round(scalingFac*distance)));

        // // finding bottom left corner point of the target quadrilateral
        // it = find(markerIds.begin(), markerIds.end(), 23);
        // index = std::distance(markerIds.begin(), it);
        // refPt4 = markerCorners.at(index).at(0);
        // pts_dst.push_back(Point(refPt4.x - round(scalingFac*distance), refPt4.y + round(scalingFac*distance)));


        // Mat h = findHomography(pts_src, pts_dst);

        // Mat warpedImage;

        // warpPerspective(im_src, warpedImage, h, frame.size(), INTER_CUBIC);

        // // Prepare mask to copy from warped image to original frame.
        // Mat mask = Mat::zeros(frame.rows, frame.cols, CV_8UC1);
        // fillConvexPoly(mask, pts_dst, Scalar(255, 255, 255), LINE_AA);

        // // Erode the mask to not copy the boundary effects from the warping
        // Mat element = getStructuringElement( MORPH_RECT, Size(5,5));
        // //            Mat element = getStructuringElement( MORPH_RECT, Size(3,3));
        // erode(mask, mask, element);

        // Mat imOut = frame.clone();
        // warpedImage.copyTo(imOut, mask);

        // Mat concatenatedOutput;
        // hconcat(frame, imOut, concatenatedOutput);
    
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
