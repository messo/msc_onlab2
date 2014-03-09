#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <cwchar>
#include <string>

#include "Magic.h"
#include "Canvas.h"

using namespace cv;
using namespace std;

const string leftImgs[] = { "left06.jpg", "left02.jpg", "left03.jpg",
		"left04.jpg", "left05.jpg", "left06.jpg", "left07.jpg", "left08.jpg",
		"left09.jpg", "left10.jpg", "left11.jpg", "left12.jpg", "left13.jpg",
		"left14.jpg" };

const string rightImgs[] = { "right06.jpg", "right02.jpg", "right03.jpg",
		"right04.jpg", "right05.jpg", "right06.jpg", "right07.jpg",
		"right08.jpg", "right09.jpg", "right10.jpg", "right11.jpg",
		"right12.jpg", "right13.jpg", "right14.jpg" };

void writePLY(const Mat& xyz, const Mat& img) {
	FILE* fp = fopen("magic.ply", "wt");
	fprintf(fp, "%s", "ply\n");
	fprintf(fp, "%s", "format ascii 1.0\n");

	int count = 0;
	for (int y = 0; y < xyz.rows; y++) {
		for (int x = 0; x < xyz.cols; x++) {
			Vec3f point = xyz.at<Vec3f>(y, x);
			if (point[2] == -1.0f)
				continue;
			count++;
		}
	}

	fprintf(fp, "element vertex %d\n", count);
	fprintf(fp, "%s", "property float x\n");
	fprintf(fp, "%s", "property float y\n");
	fprintf(fp, "%s", "property float z\n");
	fprintf(fp, "%s", "property uchar b\n");
	fprintf(fp, "%s", "property uchar g\n");
	fprintf(fp, "%s", "property uchar r\n");
	fprintf(fp, "%s", "end_header\n");

	for (int y = 0; y < xyz.rows; y++) {
		for (int x = 0; x < xyz.cols; x++) {
			Vec3f point = xyz.at<Vec3f>(y, x);
			if (point[2] == -1.0f)
				continue;
			Vec3b s = img.at<Vec3b>(y, x);
			fprintf(fp, "%f %f %f %d %d %d\n", point[0], point[1], point[2],
					s[0], s[1], s[2]);
		}
	}

	std::cout << xyz.rows << " -- " << xyz.cols << "\n";
	std::cout.flush();

	fclose(fp);
}

int main(int argc, char** argv) {
	Magic magic;
	Canvas canvas(Size(640, 480), 3);

	Camera camera;
	camera.setTranslation(0.0, 0.0, 0.5);

	int picIdx = 0;
	bool drawEpi = false;

	while (true) {
		// ---------------------------
		// BAL és JOBB OLDAL
		// ---------------------------
		Mat imgLeftRM = magic.readAndRemap(leftImgs[picIdx], LEFT);
		Mat imgRightRM = magic.readAndRemap(rightImgs[picIdx], RIGHT);

		// ---------------------------
		// 3D projekció
		// ---------------------------
		Mat xyz;
		magic.reprojectTo3D(imgLeftRM, imgRightRM, xyz);

		// ---------------------------
		// Túl távoli képek kiszûrése
		// ---------------------------
		magic.filter3DCoordinates(xyz);

		// ---------------------------
		Mat test(imgLeftRM.size(), imgLeftRM.type());
		magic.drawTest(test, imgLeftRM, camera);

		// ---------------------------
		// TRAFÓ!
		// ---------------------------
		// Random pont sorsolás
		vector<Point3f> objectPoints;
		vector<Point2f> imagePoints;
		magic.getPointsForTransformation(objectPoints, imagePoints, camera);
		std::cout << imagePoints.size() << "\n";
		std::cout.flush();

		// 1. pontok vetítése
		vector<Point2f> output;
		magic.projectPoints(objectPoints, camera, output);

		// 2. trafó
		Mat warped;
		Mat homographyMat = findHomography(imagePoints, output, CV_RANSAC);
		warpPerspective(imgLeftRM, warped, homographyMat, imgLeftRM.size());

		// --- MEGJELENÍTÉS
		canvas.put(imgLeftRM, 0);
		canvas.put(test, 1);
		canvas.put(warped, 2);
		canvas.show("left -- point-cloud -- warped");

		int c = waitKey();
		if (c == ' ') {
			picIdx = ((picIdx + 1) % 8);
		} else if (c == 'f') {
			writePLY(xyz, imgLeftRM);
		} else if (c == 'l') {
			drawEpi = !drawEpi;
		} else if (c == 27) {
			break;
		} else if (c == 'e') {
			camera.rotX(10);
		} else if (c == 'q') {
			camera.rotX(-10);
		} else if (c == 'd') {
			camera.rotY(10);
		} else if (c == 'a') {
			camera.rotY(-10);
		} else if (c == 'c') {
			camera.rotZ(10);
		} else if (c == 'y') {
			camera.rotZ(-10);
		} else if (c == 'x') {
			camera.setRotation(0, 0, 0);
			camera.setTranslation(0.0, 0.0, 0.5);
		} else if (c == 'w') {
			camera.addZ(-0.5);
		} else if (c == 's') {
			camera.addZ(0.5);
		} else if (c == 2424832) { // left
			camera.addX(0.5);
		} else if (c == 2555904) { // right
			camera.addX(-0.5);
		} else if (c == 2490368) { // up
			camera.addY(0.5);
		} else if (c == 2621440) { // down
			camera.addY(-0.5);
		}
	}

	return 0;
}
