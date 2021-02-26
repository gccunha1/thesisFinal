#include <opencv2/aruco.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/aruco/dictionary.hpp>

int main(int argc, char const *argv[])
{
	cv::Mat markerImage, boardImage;
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);
	cv::aruco::drawMarker(dictionary, 1, 400, markerImage, 1);
	cv::imwrite("marker1_5_5_50.png", markerImage);
	cv::aruco::drawMarker(dictionary, 2, 400, markerImage, 1);
	cv::imwrite("marker2_5_5_50.png", markerImage);
	dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
	cv::aruco::drawMarker(dictionary, 1, 400, markerImage, 1);
	cv::imwrite("marker1_4_4_50.png", markerImage);
	cv::aruco::drawMarker(dictionary, 2, 400, markerImage, 1);
	cv::imwrite("marker2_4_4_50.png", markerImage);
	/* code */
	return 0;
}