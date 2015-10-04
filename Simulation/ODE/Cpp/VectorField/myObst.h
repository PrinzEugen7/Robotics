#include <iostream>
#include <vector>
#include "opencv2\opencv.hpp"

using namespace cv;

void obstDetection(Mat im){
    hsv, mask;					        // 画像オブジェクトの宣言
    cvtColor(im, hsv, CV_BGR2HSV);      // 画像をRGBからHSVに変換
    inRange(hsv, Scalar(150, 70, 70), Scalar(360, 255, 255), mask);	// 色検出でマスク画像の作成
    imshow("Mask", mask);               // マスク画像の作成
    waitKey(0);
	int h = mask.rows;
	int w = mask.cols;
	for (int y = 0; y < h; y++){
		for (int x = 0; x < w; x++){
			if (mask.data[y * h + x] == 255){

			}
		}
	}
}

int main(int argc, char *argv[])
{
	Mat im = imread("map.jpg");
	obstDetection(im);
	return(0);
}
