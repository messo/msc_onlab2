/*
 * Magic.cpp
 *
 *  Created on: 2014.02.28.
 *      Author: Balint
 */

#include "Magic.h"

Magic::Magic() :
		imageSize(640, 480), sbm(StereoBM::BASIC_PRESET,
				16 * 5 /**< Range of disparity */,
				21 /**< Size of the block window. Must be odd */) {
	cameraMatrix[0] = Mat::eye(3, 3, CV_64F);
	cameraMatrix[1] = Mat::eye(3, 3, CV_64F);

	// save intrinsic parameters
	FileStorage fs("intrinsics.yml", CV_STORAGE_READ);
	if (fs.isOpened()) {
		fs["M1"] >> cameraMatrix[0];
		fs["D1"] >> distCoeffs[0];
		fs["M2"] >> cameraMatrix[1];
		fs["D2"] >> distCoeffs[1];
		fs.release();
	} else
		cout << "Error: can not load the intrinsic parameters\n";

	Mat R1, R2, P1, P2;
	// Rect validRoi[2];

	fs.open("extrinsics.yml", CV_STORAGE_READ);
	if (fs.isOpened()) {
		fs["R"] >> R;
		fs["T"] >> T;
		fs["R1"] >> R1;
		fs["R2"] >> R2;
		fs["P1"] >> P1;
		fs["P2"] >> P2;
		fs["Q"] >> Q;
		fs["validRoiLeft"] >> validRoiLeft;
		fs["validRoiRight"] >> validRoiRight;
		fs.release();
	} else
		cout << "Error: can not load the intrinsic parameters\n";

	//Precompute maps for cv::remap()
	initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize,
	CV_16SC2, rmap[0][0], rmap[0][1]);
	initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize,
	CV_16SC2, rmap[1][0], rmap[1][1]);
}

Mat Magic::readAndRemap(const string& filename, int cam) {
	Mat remapped;
	remap(imread(filename), remapped, cam);
	return remapped;

//	std::cout << remapped.cols << ", " << remapped.rows << "\n";
//	std::cout << validRoiLeft.x << ", " << validRoiLeft.y << "; "
//			<< validRoiLeft.width << ", " << validRoiLeft.height << "\n\n";
//	std::cout.flush();
//	Mat result = remapped((cam == LEFT) ? validRoiLeft : validRoiRight);
//	return result;
}

void Magic::remap(const Mat& input, Mat& output, int cam) {
	cv::remap(input, output, rmap[cam][0], rmap[cam][1], CV_INTER_LINEAR);
}

void Magic::drawEpipolarLines(Mat& canvas) {
	for (int j = 0; j < canvas.rows; j += 16)
		line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1,
				8);
}

void Magic::reprojectTo3D(const Mat& left, const Mat& right, Mat& xyz) {
	Mat imgDisparity16S = Mat(left.rows, left.cols, CV_16S);
	Mat imgDisparity8U = Mat(imgDisparity16S.rows, imgDisparity16S.cols,
	CV_8UC1);
	double minVal;
	double maxVal;

	// ---------
	// DISPARITY
	// ---------
	Mat imgLeftGray, imgRightGray;
	cvtColor(left, imgLeftGray, CV_BGR2GRAY);
	cvtColor(right, imgRightGray, CV_BGR2GRAY);
	sbm(imgLeftGray, imgRightGray, imgDisparity16S, CV_16S);

	filterSpeckles(imgDisparity16S, 0, 100, 5);

	minMaxLoc(imgDisparity16S, &minVal, &maxVal);
	imgDisparity16S.convertTo(imgDisparity8U, CV_8UC1, 255 / (maxVal - minVal));
	// imshow("disparity", imgDisparity8U);

	// ----------
	// 3D
	// ----------

	reprojectImageTo3D(imgDisparity16S, xyz, Q, true);
}

void Magic::filter3DCoordinates(Mat& xyz) {
	objectPoints.clear();
	imagePoints.clear();
	for (int y = 0; y < xyz.rows; y++) {
		for (int x = 0; x < xyz.cols; x++) {
			Vec3f& point = xyz.at<Vec3f>(y, x);
			if (fabs(point[2] - MAX_Z) < FLT_EPSILON
					|| fabs(point[2]) > MAX_Z) {
				point[2] = -1.0f;
			} else {
				objectPoints.push_back(Point3f(point));
				imagePoints.push_back(Point2f(x, y));
			}
		}
	}
}

void Magic::getCameraPose(Mat& rvec, Mat& tvec) {
	solvePnPRansac(objectPoints, imagePoints, cameraMatrix[LEFT],
			distCoeffs[LEFT], rvec, tvec);
}

void Magic::getPointsForTransformation(vector<Point3f>& objs,
		vector<Point2f>& pixels, const Camera& camera) {
	unsigned int size = objectPoints.size();
	vector<Point2f> _imagePoints;
	projectPoints(camera, _imagePoints);

	std::cout << "All: " << size << " " << _imagePoints.size() << "\n";

	for (unsigned int i = 0; i < _imagePoints.size(); i++) {
		int y = (int) _imagePoints[i].y;
		int x = (int) _imagePoints[i].x;
		if (x >= 0 && y >= 0 && y < imageSize.width && x < imageSize.height) {
			objs.push_back(objectPoints[i]);
			pixels.push_back(imagePoints[i]);
		}
	}

	std::cout << "Értékes: " << objs.size() << "\n";

	std::cout.flush();
}

void Magic::projectPoints(const vector<Point3f>& objectPoints,
		const Camera& camera, vector<Point2f>& imagePoints) {
	cv::projectPoints(objectPoints, camera.getRVec(), camera.getTVec(),
			cameraMatrix[LEFT], distCoeffs[LEFT], imagePoints);
}

void Magic::projectPoints(const Camera& camera, vector<Point2f>& imagePoints) {
	cv::projectPoints(objectPoints, camera.getRVec(), camera.getTVec(),
			cameraMatrix[LEFT], distCoeffs[LEFT], imagePoints);
}

void Magic::drawTest(Mat& img, const Mat& left, const Camera& camera) {
	vector<Point2f> _imagePoints;
	projectPoints(camera, _imagePoints);
	for (unsigned int i = 0; i < _imagePoints.size(); i++) {
		int y = (int) _imagePoints[i].y;
		int x = (int) _imagePoints[i].x;
		if (x >= 0 && y >= 0 && y < img.rows && x < img.cols)
			img.at<Vec3b>(y, x) = left.at<Vec3b>(imagePoints[i]);
	}
}

Magic::~Magic() {
	// TODO Auto-generated destructor stub
}

