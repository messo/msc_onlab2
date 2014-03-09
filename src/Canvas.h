/*
 * Canvas.h
 *
 *  Created on: 2014.03.09.
 *      Author: Balint
 */

#ifndef CANVAS_H_
#define CANVAS_H_

#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

class Canvas {
	Size imageSize;
	int imgs;
	Mat canvas;
public:
	Canvas(Size imageSize, int imgs);
	virtual ~Canvas();

	void show(const string& title) const {
		imshow(title, canvas);
	}

	void put(const Mat& img, int index);
};

#endif /* CANVAS_H_ */
