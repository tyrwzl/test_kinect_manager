#include <direct.h>
#include <vector>
#include <string>

#include "kinect_manager.h"
#include "opencv2\opencv.hpp"


bool isZeroMat(cv::Mat &DepthMat)
{
	int sum_pixel = cv::sum(DepthMat)[0];
	if (sum_pixel < 10 * 0.5 * DepthMat.rows * DepthMat.cols)
		return true;
	else
		return false;
}


int main(int argc, char** argv)
{
	// if use all source, you use like below.
	// KinectManager km(depth_flag | color_flag | infrared_flag | body_flag);
	KinectManager km(DEPTH_FLAG);
	const int depth_width = km.getDepthWidth();
	const int depth_height = km.getDepthHeight();

	std::string fileid = "test";
	std::string filepath = fileid;

	_mkdir(filepath.c_str());
	filepath = fileid + "//depth";
	_mkdir(filepath.c_str());
	filepath = fileid + "//color";
	_mkdir(filepath.c_str());

	INT64 now_time_stamp = 0;
	INT64 past_time_stamp = 0;
	std::vector<INT64> too_long_time_stamps;

	cv::Mat depth_data(depth_height, depth_width, CV_16UC1);

	std::vector<int> compression_params;
	compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
	compression_params.push_back(0);

	while (1) {
		if (km.getDepthData(reinterpret_cast<UINT16*>(depth_data.data), &now_time_stamp)) {
			INT64 diff_time_stamp = now_time_stamp - past_time_stamp;
			if (diff_time_stamp > 400000) too_long_time_stamps.push_back(now_time_stamp);

			std::string filename = fileid + "//depth//" + std::to_string(now_time_stamp) + ".png";
			cv::imwrite(filename, depth_data, compression_params);
			past_time_stamp = now_time_stamp;
		}

		if (isZeroMat(depth_data)) break;
		if (cv::waitKey(10) == 'q') break;
	}

	std::ofstream ofs(fileid + "//" + fileid + ".log", std::ios::out);

	for (int time_i = 0; time_i < too_long_time_stamps.size(); ++time_i) {
		ofs << too_long_time_stamps[time_i] << std::endl;
	}
	ofs.close();


	return 0;
}