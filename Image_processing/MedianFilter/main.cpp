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
			else  (a <= img[y][x] && img[y][x] < b)
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


int main_220328()
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






/*

2022/04/04 
김원우

*/

void avg3x3(int** img, int** img_out, int height, int width)
{
	for (int y = 1; y < height-1; y++)
	{
		for (int x = 1; x < width-1; x++)
		{
			int sum = 0;
			for (int m = -1; m <= 1; m++)
			{
				for (int n = -1; n <= 1; n++)
				{
					sum = sum + img[y + m][x + n];
				}
			}
			img_out[y][x] = sum / 9.0 + 0.5;
		}
	}

	int x, y;
	// 1
	y = 0;
	for (x = 0; x < width; x++)	img_out[y][x] = img[y][x];
	// 2
	y = height - 1;
	for (x = 0; x < width; x++)		img_out[y][x] = img[y][x];
	// 3
	x = 0;
	for (y = 0; y < height; y++)	img_out[y][x] = img[y][x];
	//4
	x = width - 1;
	for (y = 0; y < height; y++)	img_out[y][x] = img[y][x];
}

void avg5x5(int** img, int** img_out, int height, int width)
{
	for (int y = 2; y < height - 2; y++)
	{
		for (int x = 2; x < width - 2; x++)
		{
			int sum = 0;
			for (int m = -2; m <= 2; m++)
			{
				for (int n = -2; n <= 2; n++)
				{
					sum = sum + img[y + m][x + n];
				}
			}
			img_out[y][x] = sum / 25.0 + 0.5;
		}
	}

	int x, y;
	// 1
	for (y = 0; y <= 1; y++)
	{
		for (x = 0; x < width; x++)	img_out[y][x] = img[y][x];
	}
	
	// 2
	for (y = height - 2; y < height; y++)
	{
		for (x = 0; x < width; x++)		img_out[y][x] = img[y][x];
	}

	
	// 3
	for (x = 0; x <= 1; x++)
	{
		for (y = 0; y < height; y++)	img_out[y][x] = img[y][x];
	}
	
	//4
	for (x = width - 2; x < width; x++)
	{
		for (y = 0; y < height; y++)	img_out[y][x] = img[y][x];
	}


}

void avgnxn(int N, int** img, int** img_out, int height, int width)
{
	int delta = (N-1)/2;

	for (int y = delta; y < height-delta; y++)
	{
		for (int x = delta; x < width-delta; x++)
		{
			int sum = 0;
			for (int m = -delta; m <= delta; m++)
			{
				for (int n = -delta; n <= delta; n++)
				{
					sum = sum + img[y + m][x + n];
				}
			}
			img_out[y][x] = (float)sum / (N*N) + 0.5;
		}
	}


	int x, y;
	// 1
	for (y = 0; y <= delta; y++)
	{
		for (x = 0; x < width; x++)	img_out[y][x] = img[y][x];
	}

	// 2
	for (y = height - delta; y < height; y++)
	{
		for (x = 0; x < width; x++)		img_out[y][x] = img[y][x];
	}


	// 3
	for (x = 0; x <= delta; x++)
	{
		for (y = 0; y < height; y++)	img_out[y][x] = img[y][x];
	}

	//4
	for (x = width - delta; x < width; x++)
	{
		for (y = 0; y < height; y++)	img_out[y][x] = img[y][x];
	}


}

int main_220404() //220404
{
	int height, width;
	int** img = (int**)ReadImage((char*)"barbara.png", &height, &width);
	int** img_out3 = (int**)IntAlloc2(height, width);
	int** img_out5 = (int**)IntAlloc2(height, width);

	avgnxn(3, img, img_out3, height, width);
	avgnxn(5, img, img_out5, height, width);

//	avg3x3(img, img_out3, height, width);
//	avg5x5(img, img_out5, height, width);

	ImageShow((char*)"input", img, height, width);
	ImageShow((char*)"output 3x3", img_out3, height, width);
	ImageShow((char*)"output 5x5", img_out5, height, width);

	IntFree2(img, height, width);
	IntFree2(img_out3, height, width);
	IntFree2(img_out5, height, width);
	
	return (0);
}





/*
220418

gradient
laplacian
sobel

*/

// 미분의 값이 가장 크거나 작은 부분은 영상의 edge 일 수 있음

void Gradient(int** img, int** img_out, int height, int width)
{
	for (int y = 0; y < height - 1; y++)
	{
		for (int x = 0; x < width - 1; x++)
		{
			int fx = img[y][x + 1] - img[y][x];
			int fy = img[y + 1][x] - img[y][x];

			img_out[y][x] = abs(fx) + abs(fy);
			//img_out_x[y][x] = abs(fx);
			//img_out_y[y][x] = abs(fy);
		}
	}
}

void Masking(float** mask, int** img, int** img_out, int height, int width)
{
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {

			float sum = 0.0;
			for (int m = -1; m <= 1; m++) {
				for (int n = -1; n <= 1; n++) {
					sum += mask[m + 1][n + 1] * img[imin(imax(y + m, 0), height - 1)][imin(imax(x + n, 0), width - 1)];
				}
			}

			img_out[y][x] = imin(imax(sum + 0.5, 0), 255);
		}
	}
}

int FindMaxvalue(int** img, int height, int width)
{
	int value = img[0][0];

	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			value = imax(img[y][x], value);
		}
	}

	return(value);
}

int FindMinvalue(int** img, int height, int width)
{
	int value = img[0][0];

	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			value = imin(img[y][x], value);
		}
	}

	return(value);
}

void NormalizedByMaxvalue(int** img, int height, int width)
{
	int max_grad = FindMaxvalue(img, height, width);
	//int min_grad = FindMinvalue(img_out_grad, height, width);

	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			img[y][x] = 255 * img[y][x] / max_grad;
		}
	}
}

int main_220418()
{
	int height, width;
	int** img = (int**)ReadImage((char*)"lena.png", &height, &width);
	//int** img_out_x = (int**)IntAlloc2(height, width);
	//int** img_out_y = (int**)IntAlloc2(height, width);
	int** img_out_grad = (int**)IntAlloc2(height, width);
	int** img_out_grad2 = (int**)IntAlloc2(height, width);

// #1
	Gradient(img, img_out_grad, height, width);
	NormalizedByMaxvalue(img_out_grad, height, width);
		
// #2	
	int** img_out_Laplace = (int**)IntAlloc2(height, width);
	
	float** mask = (float**)FloatAlloc2(3, 3);
	mask[0][0] = -1;	mask[0][1] = -1;	mask[0][2] = -1;
	mask[1][0] = -1;	mask[1][1] = 8;		mask[1][2] = -1;
	mask[2][0] = -1;	mask[2][1] = -1;	mask[2][2] = -1;

	Masking(mask, img, img_out_Laplace, height, width);
	NormalizedByMaxvalue(img_out_Laplace, height, width);
	//clipping(img_out_Laplace, img_out_Laplace, height, width);

// #3
	int** img_out_Sobel_x = (int**)IntAlloc2(height, width);
	int** img_out_Sobel_y = (int**)IntAlloc2(height, width);

	mask[0][0] = -1;	mask[0][1] = -2;	mask[0][2] = -1;
	mask[1][0] = 0;		mask[1][1] = 0;		mask[1][2] = 0;
	mask[2][0] = 1;		mask[2][1] = 2;		mask[2][2] = 1;

	Masking(mask, img, img_out_Sobel_x, height, width);
	NormalizedByMaxvalue(img_out_Sobel_x, height, width);

	mask[0][0] = -1;		mask[0][1] = 0;		mask[0][2] = 1;
	mask[1][0] = -2;		mask[1][1] = 0;		mask[1][2] = 2;
	mask[2][0] = -1;		mask[2][1] = 0;		mask[2][2] = 1;

	Masking(mask, img, img_out_Sobel_y, height, width);
	NormalizedByMaxvalue(img_out_Sobel_y, height, width);


	ImageShow((char*)"input", img, height, width);
	ImageShow((char*)"output_grad", img_out_grad, height, width);
	//ImageShow((char*)"output_grad2", img_out_grad2, height, width);
	//ImageShow((char*)"output_x", img_out_x, height, width);
	//ImageShow((char*)"output_y", img_out_y, height, width);
	ImageShow((char*)"output_laplace", img_out_Laplace, height, width);

	ImageShow((char*)"img_out_Sobel_x", img_out_Sobel_x, height, width);
	ImageShow((char*)"img_out_Sobel_y", img_out_Sobel_y, height, width);


	IntFree2(img, height, width);
	IntFree2(img_out_grad, height, width);
	//IntFree2(img_out_x, height, width);
	//IntFree2(img_out_y, height, width);
	//IntFree2(img_out_grad2, height, width);
	IntFree2(img_out_Laplace, height, width);
	IntFree2(img_out_Sobel_x, height, width);
	IntFree2(img_out_Sobel_y, height, width);



	return (0);
}

int canny_example()
{
	// Reading image
	Mat img = imread("lena.png");
	// Display original image
	imshow("original Image", img);
	waitKey(0);

	// Convert to graycsale
	Mat img_gray;
	cvtColor(img, img_gray, COLOR_BGR2GRAY);
	// Blur the image for better edge detection
	Mat img_blur;
	GaussianBlur(img_gray, img_blur, Size(3, 3), 0);

	// Sobel edge detection
	Mat sobelx, sobely, sobelxy;
	Sobel(img_blur, sobelx, CV_64F, 1, 0, 5);
	Sobel(img_blur, sobely, CV_64F, 0, 1, 5);
	Sobel(img_blur, sobelxy, CV_64F, 1, 1, 5);
	// Display Sobel edge detection images
	imshow("Sobel X", sobelx);
	waitKey(0);
	imshow("Sobel Y", sobely);
	waitKey(0);
	imshow("Sobel XY using Sobel() function", sobelxy);
	waitKey(0);

	// Canny edge detection
	Mat edges;
	Canny(img_blur, edges, 100, 200, 3, false);
	// Display canny edge detected image
	imshow("Canny edge detection", edges);
	waitKey(0);

	destroyAllWindows();
	return 0;
}



int main_0425_1() // 선명화 마스크
{
	int height, width;
	//int** img = (int**)ReadImage((char*)"lena.png", &height, &width);
	//int** img = (int**)ReadImage((char*)"IR_1472.png", &height, &width);
	
	int** img = (int**)ReadImage((char*)"lenaGN10.png", &height, &width);
	int** img_out = (int**)IntAlloc2(height, width);

	int N = 3;
	float** mask = (float**)FloatAlloc2(N, N);
	mask[0][0] = -1; mask[0][1] = -1; mask[0][2] = -1;
	mask[1][0] = -1; mask[1][1] = 9; mask[1][2] = -1;
	mask[2][0] = -1; mask[2][1] = -1; mask[2][2] = -1;


	mask[0][0] = 0; mask[0][1] = -1; mask[0][2] = 0;
	mask[1][0] = -1; mask[1][1] = 5; mask[1][2] = -1;
	mask[2][0] = 0; mask[2][1] = -1; mask[2][2] = 0;


	float alpha = 0.7;
	mask[0][0] = 0; mask[0][1] = -alpha; mask[0][2] = 0;
	mask[1][0] = -alpha; mask[1][1] = 1 + 4 * alpha; mask[1][2] = -alpha;
	mask[2][0] = 0; mask[2][1] = -alpha; mask[2][2] = 0;


	Masking(mask, img, img_out, height, width);
	
	clipping(img_out, img_out, height, width); // 음수 및 255 초과 값을 범위내로 클리핑
	ImageShow((char*)"input", img, height, width);
	ImageShow((char*)"output", img_out, height, width);

	return(0);
}

void Swap(int* a, int* b)
{
	int buff = *a;
	*a = *b;
	*b = buff;
}


void Bubbling(int* A, int num)
{
	for (int i = 0; i < num - 1; i++) {
		if (A[i] > A[i + 1]) Swap(&A[i], &A[i + 1]); // 바로 이웃한 값끼리 위치 바꾸기
	}
}

void BubbleSort(int* A, int N)
{
	for (int i = 0; i < N - 1; i++) // 버블링 반복, 맨처음은 N개에 대해, 두번째는 (N-1)개에 대해
		Bubbling(A, N - i);
}

void Get9Pixels(int** img, int y, int x, int* C)
{
	int index = 0;
	for (int i = -1; i <= 1; i++)
	{
		for (int j = -1; j <= 1; j++)
		{
			C[index] = img[y + i][x + j];
			index++;
		}
	}
}

void Get25Pixels(int** img, int y, int x, int* C)
{
	int index = 0;
	for (int i = -2; i <= 2; i++)
	{
		for (int j = -2; j <= 2; j++)
		{
			C[index] = img[y + i][x + j];
			index++;
		}
	}
}


void Get_NxN_Pixels(int N, int** img, int y, int x, int* C)
{
	int hN = (N - 1) / 2;
	int index = 0;
	for (int i = -hN; i <= hN; i++)
	{
		for (int j = -hN; j <= hN; j++)
		{
			C[index] = img[y + i][x + j];
			index++;
		}
	}
}

void MedianFilteringNxN(int N, int** img, int** img_out, int height, int width)
{
	int* C = (int*)malloc(N * N * sizeof(int)); // == int* C = new int[N*N]

	int hN = (N - 1) / 2;

	for (int y = hN; y < height - hN; y++)
	{
		for (int x = hN; x < width - hN; x++)
		{
			Get_NxN_Pixels(N, img, y, x, C);
			BubbleSort(C, N*N);
			img_out[y][x] = C[(N * N - 1) / 2];
		}
	}

	free(C); // == delete[] C
}


void MedianFiltering5x5(int** img, int** img_out, int height, int width)
{
	int C[25];

	for (int y = 2; y < height - 2; y++)
	{
		for (int x = 2; x < width - 2; x++)
		{
			Get_NxN_Pixels(5,img, y, x, C);
			BubbleSort(C, 25);
			img_out[y][x] = C[12];
		}
	}
}

void MedianFiltering3x3(int** img, int** img_out, int height, int width)
{
	int C[9];

	for (int y = 1; y < height - 1; y++)
	{
		for (int x = 1; x < width - 1; x++)
		{
			Get_NxN_Pixels(3,img, y, x, C);
			BubbleSort(C, 9);
			img_out[y][x] = C[4];
		}
	}
}


int main_0425_2() // 중간값 필터
{
	int height, width;
	
	int** img = (int**)ReadImage((char*)"lenaSP5.png", &height, &width);
	int** img_out = (int**)IntAlloc2(height, width);
	
	int N = 3;

	MedianFilteringNxN(N, img, img_out, height, width);


	ImageShow((char*)"input", img, height, width);
	ImageShow((char*)"output", img_out, height, width);

	IntFree2(img, height, width);
	IntFree2(img_out, height, width);

	return(0);
}


int main_0425_3() // Pointer
{
	int A[5] = { 10,20,30,40,50 };

	printf("%x %x %d %d", &A[1], A + 1, *(A + 1), A[1]);

	int* B = A;
	// B = &A[2];

	printf("%x %x %d %d", &B[1], B + 1, *(B + 1), B[1]);

	int* C = &A[2];

	printf("%x %x %d %d", &C[1], C + 1, *(C + 1), C[-1]);


	return(0);
}