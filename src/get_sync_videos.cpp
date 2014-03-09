#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/operations.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

using namespace cv;
using namespace std;

const double BLACK_DELTA = 10;

double myMean(Scalar s) {
	return (s.val[0] + s.val[1] + s.val[2])/3;
}

int mainaaaa() {
	VideoCapture cam1;
	cam1.open("../105.mp4");

	VideoCapture cam2;
	cam2.open("../114.mp4");

	int i = 0;
	Mat view1, view2;
	bool black1 = false;
	bool black2 = false;
	bool detected = false;
	int first = 0;

	VideoWriter output1;
	Size S = Size(640, 480);
	output1.open("output1.avi", CV_FOURCC('X', 'V', 'I', 'D'), 10, S, true);

	VideoWriter output2;
	output2.open("output2.avi", CV_FOURCC('X', 'V', 'I', 'D'), 10, S, true);

	if (!output1.isOpened()) {
		cout << "Could not open the output video for write..";
		return -1;
	}

	if (!output2.isOpened()) {
		cout << "Could not open the output video for write..";
		return -1;
	}

	while (true) {
		++i;

		cam1.grab();
		cam2.grab();

		cam1.retrieve(view1);
		cam2.retrieve(view2);

		Scalar mean1 = mean(view1);
		Scalar mean2 = mean(view2);

		black1 = (myMean(mean1) < BLACK_DELTA);
		black2 = (myMean(mean2) < BLACK_DELTA);

		if ((black1 ^ black2) && !detected) {
			first = black1 ? 1 : 2;
			i = 0;
			detected = true;
		}

		if (black1 && black2) {
			break;
		}

		ostringstream convert;
		convert << i << "  " << myMean(mean1);
		putText(view1, convert.str().c_str(), Point2f(10, 20), 1, 1,
				Scalar(0, 255, 0), 2);

		convert.str("");
		convert << myMean(mean2);
		putText(view2, convert.str().c_str(), Point2f(10, 20), 1, 1,
				Scalar(0, 255, 0), 2);

		imshow("result1", view1);
		imshow("result2", view2);

		waitKey(100);

		continue;
	}

	while (i != 0) {
		if (first == 1) {
			cam2.grab();
		} else {
			cam1.grab();
		}
		i--;
	}

	int k=1;

	while (true) {
		cam1.grab();
		cam2.grab();

		cam1.retrieve(view1);
		cam2.retrieve(view2);

		imshow("result1", view1);
		imshow("result2", view2);

		output1 << view1;
		output2 << view2;

		int ch = waitKey(100);
		if(ch == 'c') {
			ostringstream convert;

			convert << "left0" << k << ".jpg";

			imwrite(convert.str(), view1);

			ostringstream convert2;

			convert2 << "right0" << k << ".jpg";

			imwrite(convert2.str(), view2);
			k++;
		}
	}

	output1.release();
	output2.release();

	return 0;
}
