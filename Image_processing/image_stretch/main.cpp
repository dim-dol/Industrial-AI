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


int project1_0314()
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

/*

220321 

Clipping and Fusion



*/
void addvalue(int height, int width, int value, int** image, int** image_out)
{
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			image_out[y][x] = image[y][x] + value;
		}
	}
}

void clipping(int** image, int** image_out2, int height, int width)
{
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			if (image[y][x] > 255)
				image_out2[y][x] = 255;
			else if (image[y][x] < 0)
				image_out2[y][x] = 0;
			else
				image_out2[y][x] = image[y][x];

		}
	}
}



#define imax(x,y) ((x>y)? x:y)
#define imin(x,y) ((x<y)? x:y)

void clipping2(int** image, int** image_out, int height, int width)
{
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			image_out[y][x] = imin(imax(image[y][x], 0), 255);

		}
	}
}

void fusion(int height, int width, float alpha, int** image_out, int** image1, int** image2)
{
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			image_out[y][x] = alpha * image1[y][x] + (1.0 - alpha) * image2[y][x];
		}
	}
}


int main_0321() //increase
{
	int height, width;
	int** image1 = (int**)ReadImage((char*)"barbara.png", &height, &width);
	int** image2 = (int**)ReadImage((char*)"lena.png", &height, &width);
	int** image_out = (int**)IntAlloc2(height, width);

	//float alpha = 0.3;
	
	
	for (float alpha = 0.1; alpha < 1.0; alpha += 0.1)
	{
		fusion(height, width, alpha, image_out, image1, image2);

		ImageShow((char*)"output", image_out, height, width);
	}

	//ImageShow((char*)"input1", image1, height, width);
	//ImageShow((char*)"input2", image2, height, width);
	//ImageShow((char*)"output", image_out, height, width);

	IntFree2(image1, height, width);
	IntFree2(image2, height, width);
	IntFree2(image_out, height, width);

	return(0);
}

int main_0321_2() // decrease
{
	int height, width;
	int** image1 = (int**)ReadImage((char*)"barbara.png", &height, &width);
	int** image2 = (int**)ReadImage((char*)"lena.png", &height, &width);
	int** image_out = (int**)IntAlloc2(height, width);

	//float alpha = 0.3;
	ImageShow((char*)"input1", image1, height, width);
	ImageShow((char*)"input2", image2, height, width);

	for (float alpha = 1.0; alpha > 0; alpha -= 0.1)
	{
		fusion(height, width, alpha, image_out, image1, image2);

		ImageShow((char*)"output", image_out, height, width);
	}


	IntFree2(image1, height, width);
	IntFree2(image2, height, width);
	IntFree2(image_out, height, width);

	return(0);
}


void copyblock(int dx, int dy, int x0, int y0, int** image, int** image_out)
{
	for (int x = 0; x < dx; x++)
	{
		for (int y = 0; y < dy; y++)
		{
			image_out[y][x] = image[y0 + y][x0 + x];

		}
	}
}



int main_0321_3()
{
	int height, width;
	int** image = (int**)ReadImage((char*)"barbara.png", &height, &width);
	
	int x0 = 200;
	int y0 = 160;
	int dx = 230;
	int dy = 300;

	int** image_out = (int**)IntAlloc2(dy, dx);

	copyblock(dx, dy, x0, y0, image, image_out);

	ImageShow((char*)"input", image, height, width);
	//ImageShow((char*)"output", image2, height, width);
	ImageShow((char*)"output", image_out, dy, dx);

	IntFree2(image, height, width);
	IntFree2(image_out, dy, dx);

	return(0);
}


int main_220321()
{
	int height, width;
	int** image1 = (int**)ReadImage((char*)"barbara.png", &height, &width);
	int** image2 = (int**)ReadImage((char*)"lena.png", &height, &width);
	int y0 = 0, x0 = width / 2;
	int dy = height / 2, dx = width / 2;
	
	int** image_out1 = (int**)IntAlloc2(dy, dx);
	int** image_out2 = (int**)IntAlloc2(dy, dx);
	int** image_out_out = (int**)IntAlloc2(dy, dx);

	copyblock(dx, dy, x0, y0, image1, image_out1);
	copyblock(dx, dy, x0, y0, image2, image_out2);
	fusion(dy, dx, 0.5, image_out_out, image_out1, image_out2);

	ImageShow((char*)"input1", image_out1, dy, dx);
	ImageShow((char*)"input2", image_out2, dy, dx);
	ImageShow((char*)"output", image_out_out, dy, dx);

	IntFree2(image1, height, width);
	IntFree2(image2, height, width);
	IntFree2(image_out1, dy, dx);
	IntFree2(image_out2, dy, dx);
	IntFree2(image_out_out, dy, dx);

	return(0);
}

/*

220328

Stretch and gamma corr, draw histogram


*/


void stretch_img(int a, int b, int c, int d, int** img, int** img_out, int width, int height)
{
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			if (img[y][x] < a)
				img_out[y][x] = (float)c / a * img[y][x] + 0.5;
			else if (a <= img[y][x] && img[y][x] < b)
				img_out[y][x] = ((float)d - c) / (b - a) * (img[y][x] - a) + c + 0.5;
			else
				img_out[y][x] = (255.0 - d) / (255 - b) * (img[y][x] - b) + d + 0.5;
		}
	}
}

// y=x^{1/r} : x <- img[y][x]/255.0, y<- y*255.0
void gamma_corr_img(float gamma, int** img, int** img_out, int width, int height)
{
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			float output = pow(img[y][x] / 255.0, 1.0 / gamma);
			img_out[y][x] = 255.0 * output + 0.5;

		}
	}
}


int main_stretch_gamma_0328()
{
	int height, width;
	int** img = (int**)ReadImage((char*)"barbara.png", &height, &width);
	int** img_out = (int**)IntAlloc2 (height, width);

	int a = 100, b = 140, c = 200, d = 100;
	float gamma = 2;
	//c = a / b;
	//d = b / a;

	//stretch_img(a, b, c, d, img, img_out, width, height);
	gamma_corr_img(gamma, img, img_out, width, height);
	
#if 0
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{

			if (img[y][x] < a)
				img_out[y][x] = (float)c / a * img[y][x] + 0.5;
			else if (a <= img[y][x] && img[y][x] < b)
				img_out[y][x] = ((float)d - c) / (b - a) * (img[y][x] - a) + c + 0.5;
			else
				img_out[y][x] = (255.0 - d) / (255 - b) * (img[y][x] - b) + d + 0.5;

		}
	}
#endif

	ImageShow((char*)"input", img, height, width);
	ImageShow((char*)"output", img_out, height, width);

	IntFree2(img, height, width);
	IntFree2(img_out, height, width);

	return 0;
}


void findhistogram(int* hist, int** img, int width, int height)
{

	//	for (int i = 0; i < 256; i++)
		//	hist[i] = 0;


	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			hist[img[y][x]]++;
			
			/*
			for (int pixel = 0; pixel < 256; pixel++)
			{
				if (img[y][x] == pixel)
					hist[pixel]++;
			}
			*/

		}
	}

}

void sum_hist(int* hist, int*sum)
{
	sum[0] = hist[0];

	for (int i = 1; i < 256; i++)
	{
		sum[i] = sum[i - 1] + hist[i];
	}
}


int main()
{
	int height, width;
	int** img = (int**)ReadImage((char*)"barbara.png", &height, &width);

	int hist[256] = { 0 };
	int sum[256];

	findhistogram(hist, img, width, height);

	sum_hist(hist, sum);

#if 0
	sum[0] = hist[0];

	for (int i = 1; i < 256; i++)
	{
		sum[i] = sum[i - 1] + hist[i];
	}
#endif

#if 0
//	for (int i = 0; i < 256; i++)
	//	hist[i] = 0;


	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			//hist[img[y][x]]++;

			for(int pixel=0; pixel<256;pixel++)
			{
				if (img[y][x] == pixel)
					hist[pixel]++;
			}
	
		}
	}
#endif

	for (int i = 0; i < 256;i++)
	{
		printf("hist[%d] = %d\n", i, hist[i]);
	}
	ImageShow((char*)"input", img, height, width);

	DrawHistogram((char*)"histogram", hist);
	DrawHistogram((char*)"histogram_sum", sum);

	IntFree2(img, height, width);

	return 0;
}