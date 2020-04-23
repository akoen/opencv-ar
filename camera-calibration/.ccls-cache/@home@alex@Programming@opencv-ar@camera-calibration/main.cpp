#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>

using namespace std;
using namespace cv;

// Define dimensions of checkerboard
int CHECKERBOARD[2]{6, 9};

int main() {
  // Vector to store vectors of 3D points for checkerboard
  vector<vector<Point3f>> object_points;

  // Vector to store vecors of 2D points for checkerboard
  vector<vector<Point2f>> image_points;

  // Define world coordinates for 3D points
  vector<Point3f> objp;
  for (int i{0}; i < CHECKERBOARD[1]; i++) {
    for (int j{0}; j < CHECKERBOARD[0]; j++)
      objp.push_back(cv::Point3f(j,i,0));
  }

  vector<String> images;
  string path = "./img/*.jpg";

  glob(path, images);

  Mat frame, gray;
  // Vector to store pixel coordinates of checkerboard corners
  vector<Point2f> corners;
  bool success;

  for (int i{0}; i < images.size(); i++) {
    frame = imread(images[i]);
    // convert frame to grayscale
    cvtColor(frame, gray, COLOR_BGR2GRAY);

    // Find corners of checkerboard
    success = findChessboardCorners(
        gray, Size(CHECKERBOARD[0], CHECKERBOARD[1]), corners,
        CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK |
            CALIB_CB_NORMALIZE_IMAGE);

    // If corners are detected, refine corrdinates

    if (success) {
      TermCriteria criteria(TermCriteria::MAX_ITER + TermCriteria::EPS, 30, 0.001);

      // refine pixel coordinates for 2d points
      cornerSubPix(gray, corners, Size(11, 11), Size(-1, 1), criteria);

      // Display corners on checkerboard
      drawChessboardCorners(frame, Size(CHECKERBOARD[0], CHECKERBOARD[1]),
                            corners, success);

      object_points.push_back(objp);
      image_points.push_back(corners);
    }

    imshow("Image", frame);
    waitKey(0);
  }

  destroyAllWindows();
  Mat intrinsic, distCoeffs, rvecs, tvecs;

  // Perform camera calibration using known 3D points (objpoints) and detected
  // corners (imgpoints)

  calibrateCamera(object_points, image_points, Size(gray.rows, gray.cols),
                  intrinsic, distCoeffs, rvecs, tvecs);

  cout << "Intrinsic values: " << intrinsic << endl;
  cout << "distCoeffs: " << distCoeffs << endl;
  cout << "Rotation vector: " << rvecs << endl;
  cout << "Translation vector:" << tvecs << endl;

  return 0;
}
