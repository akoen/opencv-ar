#include <opencv2/aruco.hpp>
#include <iostream>

int main(void) {
  cv::Mat markerImage;
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);
  cv::aruco::drawMarker(dictionary, 33, 200, markerImage, 1);
  std::cout << "Hello World" << std::endl;
  return 0;
}
