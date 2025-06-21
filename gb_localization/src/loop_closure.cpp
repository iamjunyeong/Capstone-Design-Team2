// test.cpp
#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
	std::cout << "OpenCV version : " << CV_VERSION << std::endl;
	cv::Mat image = cv::imread("Lenna.png");
	if (image.empty()) {
		std::cerr << "Could not open or find the image" << std::endl;
		return -1;
	}
	cv::imshow("Display window", image);
	cv::waitKey(0);
	return 0;
}