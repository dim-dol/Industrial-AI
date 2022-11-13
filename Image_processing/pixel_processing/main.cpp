#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <malloc.h>
#include <vector>

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/types_c.h"


using namespace cv;
using namespace std;

typedef struct {
	int r, g, b;
}int_rgb;

#define SQ(x) ((x)*(x))

int** IntAlloc2(int height, int width)
{
	int** tmp;
	tmp = (int**)calloc(height, sizeof(int*));
	for (int i = 0; i < height; i++)
		tmp[i] = (int*)calloc(width, sizeof(int));
	return(tmp);
}

void IntFree2(int** image, int height, int width)
{
	for (int i = 0; i < height; i++)
		free(image[i]);

	free(image);
}


float** FloatAlloc2(int height, int width)
{
	float** tmp;
	tmp = (float**)calloc(height, sizeof(float*));
	for (int i = 0; i < height; i++)
		tmp[i] = (float*)calloc(width, sizeof(float));
	return(tmp);
}

void FloatFree2(float** image, int height, int width)
{
	for (int i = 0; i < height; i++)
		free(image[i]);

	free(image);
}

int_rgb** IntColorAlloc2(int height, int width)
{
	int_rgb** tmp;
	tmp = (int_rgb**)calloc(height, sizeof(int_rgb*));
	for (int i = 0; i < height; i++)
		tmp[i] = (int_rgb*)calloc(width, sizeof(int_rgb));
	return(tmp);
}

void IntColorFree2(int_rgb** image, int height, int width)
{
	for (int i = 0; i < height; i++)
		free(image[i]);

	free(image);
}

int** ReadImage(char* name, int* height, int* width)
{
	Mat img = imread(name, IMREAD_GRAYSCALE);
	int** image = (int**)IntAlloc2(img.rows, img.cols);

	*width = img.cols;
	*height = img.rows;

	for (int i = 0; i < img.rows; i++)
		for (int j = 0; j < img.cols; j++)
			image[i][j] = img.at<unsigned char>(i, j);

	return(image);
}

void WriteImage(char* name, int** image, int height, int width)
{
	Mat img(height, width, CV_8UC1);
	for (int i = 0; i < height; i++)
		for (int j = 0; j < width; j++)
			img.at<unsigned char>(i, j) = (unsigned char)image[i][j];

	imwrite(name, img);
}


void ImageShow(char* winname, int** image, int height, int width)
{
	Mat img(height, width, CV_8UC1);
	for (int i = 0; i < height; i++)
		for (int j = 0; j < width; j++)
			img.at<unsigned char>(i, j) = (unsigned char)image[i][j];
	imshow(winname, img);
	waitKey(0);
}



int_rgb** ReadColorImage(char* name, int* height, int* width)
{
	Mat img = imread(name, IMREAD_COLOR);
	int_rgb** image = (int_rgb**)IntColorAlloc2(img.rows, img.cols);

	*width = img.cols;
	*height = img.rows;

	for (int i = 0; i < img.rows; i++)
		for (int j = 0; j < img.cols; j++) {
			image[i][j].b = img.at<Vec3b>(i, j)[0];
			image[i][j].g = img.at<Vec3b>(i, j)[1];
			image[i][j].r = img.at<Vec3b>(i, j)[2];
		}

	return(image);
}

void WriteColorImage(char* name, int_rgb** image, int height, int width)
{
	Mat img(height, width, CV_8UC3);
	for (int i = 0; i < height; i++)
		for (int j = 0; j < width; j++) {
			img.at<Vec3b>(i, j)[0] = (unsigned char)image[i][j].b;
			img.at<Vec3b>(i, j)[1] = (unsigned char)image[i][j].g;
			img.at<Vec3b>(i, j)[2] = (unsigned char)image[i][j].r;
		}

	imwrite(name, img);
}

void ColorImageShow(char* winname, int_rgb** image, int height, int width)
{
	Mat img(height, width, CV_8UC3);
	for (int i = 0; i < height; i++)
		for (int j = 0; j < width; j++) {
			img.at<Vec3b>(i, j)[0] = (unsigned char)image[i][j].b;
			img.at<Vec3b>(i, j)[1] = (unsigned char)image[i][j].g;
			img.at<Vec3b>(i, j)[2] = (unsigned char)image[i][j].r;
		}
	imshow(winname, img);

}

void DrawHistogram(char* comments, int* Hist)
{
	int histSize = 256; /// Establish the number of bins
						// Draw the histograms for B, G and R
	int hist_w = 512; int hist_h = 512;
	int bin_w = cvRound((double)hist_w / histSize);

	Mat histImage(hist_h, hist_w, CV_8UC3, Scalar(255, 255, 255));
	Mat r_hist(histSize, 1, CV_32FC1);
	for (int i = 0; i < histSize; i++)
		r_hist.at<float>(i, 0) = (float)Hist[i];
	/// Normalize the result to [ 0, histImage.rows ]
	normalize(r_hist, r_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());

	/// Draw for each channel
	for (int i = 1; i < histSize; i++)
	{
		line(histImage, Point(bin_w * (i - 1), hist_h - cvRound(r_hist.at<float>(i - 1))),
			Point(bin_w * (i), hist_h - cvRound(r_hist.at<float>(i))),
			Scalar(255, 0, 0), 2, 8, 0);
	}

	/// Display
	namedWindow(comments, WINDOW_AUTOSIZE);
	imshow(comments, histImage);

	waitKey(0);

}

template <typename _TP>
void ConnectedComponentLabeling(_TP** seg, int height, int width, int** label, int* no_label)
{

	//Mat bw = threshval < 128 ? (img < threshval) : (img > threshval);
	Mat bw(height, width, CV_8U);

	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++)
			bw.at<unsigned char>(i, j) = (unsigned char)seg[i][j];
	}
	Mat labelImage(bw.size(), CV_32S);
	*no_label = connectedComponents(bw, labelImage, 8); // 0源뚯� �ы븿�� 媛�닔��

	(*no_label)--;

	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++)
			label[i][j] = labelImage.at<int>(i, j);
	}
}


#define GetMax(x, y) ((x)>(y) ? x : y)
#define GetMin(x, y) ((x)<(y) ? x : y)

void AddGaussianNoise(float mean, float std, int** img, int height, int width, int** img_out)
{
	Mat bw(height, width, CV_16SC1);

	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++)
			bw.at<short>(i, j) = (short)img[i][j];
	}

	Mat noise_img(height, width, CV_16SC1);
	randn(noise_img, mean, std);

	addWeighted(bw, 1.0, noise_img, 1.0, 0.0, bw);

	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++)
			img_out[i][j] = GetMin(GetMax(bw.at<short>(i, j), 0), 255);
	}
}


int ex_0307()
{
	int height, width;
	int** image = (int**)ReadImage((char*)"barbara.png", &height, &width);

	for (int j = 0; j < 200; j++)
	{
		for (int i = 0; i < 100; i++)
		{
			image[j][i] += 100;
			if (image[j][i] > 255)
				image[j][i] = 255;

		}
	}

	ImageShow((char*)"test", image, height, width);

	return(0);
}

void SetImageColor(int** img, int height, int width, int value) //ex0314
{
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			img[y][x] = value;
		}
	}

}


void SetBlockValue(int** img, int y0, int x0, int y1, int x1, int value) //ex0314
{
	for (int y = y0; y < y1; y++)
	{
		for (int x = x0; x < x1; x++)
		{
			img[y][x] = value;
		}
	}
}

int ex0314_1()
{
	int height = 128;
	int width = 256;
	int test=1;
	int** img_ptr = (int**)IntAlloc2(height, width);

	SetImageColor(img_ptr, height, width, 255);

	SetBlockValue(img_ptr, 32, 64, 96, 96, 128);

	ImageShow((char*)"영상보기", img_ptr, height, width);
	
	IntFree2(img_ptr, 128, 256);
	
	return(0);

}


int ImgReadWrite() // ex0314
{
	int height, width;
	int** image = (int**)ReadImage((char*)"barbara.png", &height, &width);

	WriteImage((char*)"barbara3.jpg", image, height, width);

	IntFree2(image, height, width);

	return(0);
}


void binarization2(int** input, int**output, int y0, int y1, int x0, int x1, int thresh)
{ // Using 4 area
	for (int y = y0; y < y1; y++)
	{
		for (int x = x0; x < x1; x++)
		{
			if (input[y][x] >= thresh)
				output[y][x] = 255;
			else
				output[y][x] = 0;
		}
	}
}

void binarization(int** input, int** output, int height, int width, int thresh)
{
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			if (input[y][x] >= thresh)
				output[y][x] = 255;
			else
				output[y][x] = 0;
		}
	}
}


int ex0314_2()
{
	int height, width;
	int** image = (int**)ReadImage((char*)"barbara.png", &height, &width);
	int** image_out = (int**)IntAlloc2(height, width);

	binarization(image, image_out, height, width, 128);

	ImageShow((char*)"input", image, height, width);
	ImageShow((char*)"output", image_out, height, width);

	IntFree2(image, height, width);
	IntFree2(image_out, height, width);

	return(0);
}

int ex0314_3()
{

	int height, width;
	int** image = (int**)ReadImage((char*)"barbara.png", &height, &width);
	int** image_out = (int**)IntAlloc2(height, width);

	for (int thresh = 50; thresh <= 200; thresh += 50)
	{
		binarization(image, image_out, height, width, thresh);
		ImageShow((char*)"input", image_out, height, width);
	}

	
	IntFree2(image, height, width);
	IntFree2(image_out, height, width);

	return(0);
}



int main()
{
	int height, width;
	int** image = (int**)ReadImage((char*)"barbara.png", &height, &width);
	int** image_out = (int**)IntAlloc2(height, width);

	binarization2(image, image_out, 0, height / 2, 0, width / 2, 50);
	binarization2(image, image_out, 0, height / 2, width / 2, width, 100);
	binarization2(image, image_out, height / 2, height, 0, width / 2, 150);
	binarization2(image, image_out, height / 2, height, width / 2, width, 200);

	ImageShow((char*)"input", image, height, width);
	ImageShow((char*)"output", image_out, height, width);

	IntFree2(image, height, width);
	IntFree2(image_out, height, width);

	return(0);
}
