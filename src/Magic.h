/*
 * Magic.h
 *
 *  Created on: 2014.02.28.
 *      Author: Balint
 */

#ifndef MAGIC_H_
#define MAGIC_H_

#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/operations.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "Camera.h"

using namespace cv;
using namespace std;

const int LEFT = 0;
const int RIGHT = 1;
const double MAX_Z = 2.5;

class Magic {
	Size imageSize;
	StereoBM sbm;
	Mat cameraMatrix[2], distCoeffs[2];
	Mat R, T, E, F, Q;
	Mat rmap[2][2];
	Rect validRoiLeft, validRoiRight;
	vector<Point3f> objectPoints;
	vector<Point2f> imagePoints;
	RNG rng;
public:
	Magic();

	virtual ~Magic();

	Mat readAndRemap(const string& filename, int cam);

	void remap(const Mat& input, Mat& output, int cam);

	void drawEpipolarLines(Mat& canvas);

	void reprojectTo3D(const Mat& left, const Mat& right, Mat& xyz);

	void filter3DCoordinates(Mat& xyz);

	void getCameraPose(Mat& rvec, Mat& tvec);

	void getPointsForTransformation(vector<Point3f>& objs, vector<Point2f>& pixels, const Camera& camera);

	void projectPoints(const vector<Point3f>& objectPoints, const Camera& camera, vector<Point2f>& imagePoints);

	void projectPoints(const Camera& camera, vector<Point2f>& imagePoints);

	void drawTest(Mat& img, const Mat& left, const Camera& camera);
};

#endif /* MAGIC_H_ */
