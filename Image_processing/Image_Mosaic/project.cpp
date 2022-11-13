#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <malloc.h>
#include <vector>
#include <iostream>
#include <algorithm>

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/types_c.h"

#define DB_SIZE 510

using namespace cv;
using namespace std;

typedef struct {
	int r, g, b;
}int_rgb;


int** IntAlloc2(int height, int width)
{
	int** tmp;
	tmp = (int**)calloc(height, sizeof(int*));
	for (int i = 0; i < height; i++)
		tmp[i] = (int*)calloc(width, sizeof(int));
	return(tmp);
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

void IntFree2(int** image, int height, int width)
{
	for (int i = 0; i < height; i++)
		free(image[i]);

	free(image);
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


void ReadBlock(int** img, int y, int x, int dy, int dx, int** block)
{
	for (int i = 0; i < dy; i++) {
		for (int j = 0; j < dx; j++) {
			block[i][j] = img[y + i][x + j];
		}
	}
}

void WriteBlock(int** img, int y, int x, int dy, int dx, int** block)
{
	for (int i = 0; i < dy; i++) {
		for (int j = 0; j < dx; j++) {
			img[y + i][x + j] = block[i][j];
		}
	}
}


void ReadAllDBimages_32(char* filename, int*** tplate, int* dy_out, int* dx_out)
{
	int dy, dx;
	for (int i = 0; i < DB_SIZE; i++) {
		sprintf_s(filename, 100, ".\\db4mosaic\\dbs%04d.jpg", i);
		tplate[i] = (int**)ReadImage(filename, &dy, &dx); // 32x32로 고정되어 있음

	}

	*dy_out = dy;
	*dx_out = dx;

}

void ReadAllDBimages_16(char* filename, int*** tplate, int* dy_out, int* dx_out)
{
	int dy, dx;
	for (int i = 0; i < DB_SIZE; i++) {
		sprintf_s(filename, 100, ".\\db4mosaic\\s_dbs%04d.jpg", i);
		tplate[i] = (int**)ReadImage(filename, &dy, &dx); // 16x16로 고정되어 있음

	}

	*dy_out = dy;
	*dx_out = dx;

}

void ReadAllDBimages_8(char* filename, int*** tplate, int* dy_out, int* dx_out)
{
	int dy, dx;
	for (int i = 0; i < DB_SIZE; i++) {
		sprintf_s(filename, 100, ".\\db4mosaic\\s_s_dbs%04d.jpg", i);
		tplate[i] = (int**)ReadImage(filename, &dy, &dx); // 8x8로 고정되어 있음

	}

	*dy_out = dy;
	*dx_out = dx;

}

void ReadAllDBimages(char* filename, int*** tplate, int* dy_out, int* dx_out)
{
	int dy, dx;
	for (int i = 0; i < DB_SIZE; i++) {
		sprintf_s(filename, 100, ".\\db4mosaic\\dbs%04d.jpg", i);
		tplate[i] = (int**)ReadImage(filename, &dy, &dx); // 32x32로 고정되어 있음

	}

	*dy_out = dy;
	*dx_out = dx;

}


float MAD(int** img1, int** img2, int height, int width)
{
	int diff = 0;
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			diff += abs(img1[y][x] - img2[y][x]);
		}
	}

	float mad = (float)diff / (height * width);

	return mad;
}


void FindBestTemplate(int** block, int*** tplate, int dy, int dx, int* idx_min, float* mad_min)
{
	*mad_min = FLT_MAX;
	//int idx_min = 0;
	for (int idx = 0; idx < DB_SIZE; idx++)
	{
		float mad = MAD(block, tplate[idx], dy, dx);
		if (mad < *mad_min)
		{
			*mad_min = mad;
			*idx_min = idx;
		}
	}
	printf("idx_min = %d\n", *idx_min);
	printf("mad_min = %.2f\n", *mad_min);

}


typedef struct value {
	int idx;
	float mad;
	int x;
	int y;
};

struct value p[256] = { 0, };
struct value p2[512] = { 0, };

void my_print(struct value p, int i)
{
	printf("p[%d].mad = %.2f, p[%d].x = %d, p[%d].y = %d\n",i,p.mad,i,p.x,i,p.y);
}

void swap(value* p1, value* p2) {
	value tmp;
	tmp = *p1;
	*p1 = *p2;
	*p2 = tmp;
}

void sort(value p[], int n) {
	int i, j;

	for (i = 0; i < n; i++) {
		for (j = 0; j < n - i - 1; j++) {
			if (p[j].mad > p[j + 1].mad) {
				swap(&p[j], &p[j + 1]);
			}
		}
	}
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
	waitKey(0);
}

void record_struct(int** img, int y, int x, int** block, int*** tplate, int dy, int dx, int num, int* idx_min, float* mad_min)
{
	ReadBlock(img, y, x, dy, dx, block);
	// 510개의 mad 중 최소의 mad 를 가지는 tplate를 결정하기
	//idx_min

	FindBestTemplate(block, tplate, dy, dx, idx_min, mad_min); // idx_min 과 mad_min 가져오기

	p2[num].idx = *idx_min;
	p2[num].mad = *mad_min;
	p2[num].y = y;
	p2[num].x = x;
}

void MakeMosaicImage8(int** img, int** img_out, int height, int width, int*** tplate, int dy, int dx)
{
	int** block = (int**)IntAlloc2(dy, dx);
	int num = 0;

	for (int k = 256; k < 512; k++)
	{
		int idx_min;
		float mad_min;

		int x1 = p2[k].x;
		int y1 = p2[k].y;
		int x2 = x1 + dx;
		int y2 = y1 + dy;

		record_struct(img, y1, x1, block, tplate, dy, dx, num, &idx_min, &mad_min);
		WriteBlock(img_out, p2[num].y, p2[num].x, dy, dx, tplate[p2[num].idx]);

		record_struct(img, y1, x2, block, tplate, dy, dx, num, &idx_min, &mad_min);
		WriteBlock(img_out, p2[num].y, p2[num].x, dy, dx, tplate[p2[num].idx]);

		record_struct(img, y2, x1, block, tplate, dy, dx, num, &idx_min, &mad_min);
		WriteBlock(img_out, p2[num].y, p2[num].x, dy, dx, tplate[p2[num].idx]);

		record_struct(img, y2, x2, block, tplate, dy, dx, num, &idx_min, &mad_min);
		WriteBlock(img_out, p2[num].y, p2[num].x, dy, dx, tplate[p2[num].idx]);


	}
	

	IntFree2(block, dy, dx);
}

void MakeMosaicImage16(int** img, int** img_out, int height, int width, int*** tplate, int dy, int dx)
{
	int** block = (int**)IntAlloc2(dy, dx);
	int num = 0;

	int idx = (height / dy) * (width / dx); // 256, 512, 1024

	
	for (int k = 128; k < 256; k++)
	{
		int idx_min;
		float mad_min;

		int x1 = p[k].x;
		int y1 = p[k].y;
		int x2 = x1 + dx;
		int y2 = y1 + dy;
		
		record_struct(img, y1, x1, block, tplate, dy, dx, num, &idx_min, &mad_min);
		num++;

		record_struct(img, y1, x2, block, tplate, dy, dx, num, &idx_min, &mad_min);
		num++;

		record_struct(img, y2, x1, block, tplate, dy, dx, num, &idx_min, &mad_min);
		num++;

		record_struct(img, y2, x2, block, tplate, dy, dx, num, &idx_min, &mad_min);
		num++;
		
	}

	sort(p2, 512);
	for (int i = 0; i < 256; i++)
	{
		WriteBlock(img_out, p2[i].y, p2[i].x, dy, dx, tplate[p2[i].idx]);
	}
	
	/* mad, idx, 좌표 */
	/*for (int i = 0; i < 512; i++)
	{
		printf("p2[%d].mad = %.2f, p2[%d].idx = %d, p2[%d].x = %d, p2[%d].y = %d\n", i, p2[i].mad, i,p2[i].idx, p2[i].x, i, p2[i].y);
	}*/

	IntFree2(block, dy, dx);
}


void MakeMosaicImage32(int** img, int** img_out, int height, int width, int*** tplate, int dy, int dx)
{
	int** block = (int**)IntAlloc2(dy, dx);
	int num = 0;
	//int mad_arr[256] = { 0 };

	int idx = (height / dy) * (width / dx); // 256, 512, 1024

	for (int y = 0; y < height; y += dy)
	{
		for (int x = 0; x < width; x += dx)
		{

			int idx_min;
			float mad_min;

			printf(" y = %d, x = %d\n", y, x);

			ReadBlock(img, y, x, dy, dx, block);
			// 510개의 mad 중 최소의 mad 를 가지는 tplate를 결정하기
			//idx_min

			FindBestTemplate(block, tplate, dy, dx, &idx_min, &mad_min); // idx_min 과 mad_min 가져오기

			p[num].idx = idx_min;
			p[num].mad = mad_min;
			p[num].y = y;
			p[num].x = x;

			num++;


		}
	}

	sort(p, 256);

	for (int i = 0; i < idx / 2; i++)
	{
		WriteBlock(img_out, p[i].y, p[i].x, dy, dx, tplate[p[i].idx]);
		p[i].mad = -1;


	}

	for (int i = 0; i < idx; i++)
	{
		my_print(p[i], i);
	}

	IntFree2(block, dy, dx);
}

int main()
{
	//int height1, width1;
	char filename[100];
	int** tplate32[DB_SIZE];
	int** tplate16[DB_SIZE];
	int** tplate8[DB_SIZE];
	int dy32, dx32;
	int dy16, dx16;
	int dy8, dx8;

	ReadAllDBimages_32(filename, tplate32, &dx32, &dy32);
	ReadAllDBimages_16(filename, tplate16, &dx16, &dy16);
	ReadAllDBimages_8(filename, tplate8, &dx8, &dy8);

	int height, width;
	int** img = (int**)ReadImage((char*)"lena.png", &height, &width);

	int** img_out = (int**)IntAlloc2(height, width);


	// mad_min 을 sorting하여 큰 순서대로 50%에 대해 32x32으로 template
	MakeMosaicImage32(img, img_out, height, width, tplate32, dy32, dx32);

	// mad_min 을 sorting하여 큰 순서대로 50%에 대해 16x16으로 template
	MakeMosaicImage16(img, img_out, height, width, tplate16, dy16, dx16);

	// mad_min 을 sorting하여 큰 순서대로 50%에 대해 8x8으로 template
	MakeMosaicImage8(img, img_out, height, width, tplate8, dy8, dx8);


	WriteImage((char*)"out_img.png", img_out, height, width);

	ImageShow((char*)"input", img, height, width);
	ImageShow((char*)"img_out", img_out, height, width);

	return 0;
}