#include "Image.h"


void Image::set(float patchAreaRatio,int patchDiv,int tileNum,int oriNum,int iNu,float dSigma,float dF)
{
	this->patchAreaRatio = patchAreaRatio;
	this->patchDiv = patchDiv;
	this->tileNum = tileNum;
	this->oriNum = oriNum;
	this->iNu = iNu;
	this->dSigma = dSigma;
	this->dF = dF;
}

float Image::queryAVG(CvPoint2D32f &left_up, CvPoint2D32f &right_down)
{
	int x_start = (left_up.x > 0)? ceil(left_up.x) : 0;
	int x_end = (right_down.x < filteredImage->width)? floor(right_down.x) : filteredImage->width-1;
	int y_start = (left_up.y > 0)? ceil(left_up.y) : 0;
	int y_end = (right_down.y < filteredImage->height)? floor(right_down.y) : filteredImage->height-1;
	int sum = 0;
	for(int x=x_start;x<=x_end;x++)
	{
		for(int y=y_start;y<=y_end;y++)
		{
			//sum += cvGet2D(filteredImage,y,x).val[0];
			sum += filteredImage->imageData[y*filteredImage->width+x] & 255;
		}
	}
	return sum/float((x_end-x_start+1)*(y_end-y_start+1));
	return 0;
}

CvMat* Image::getFeatMat()
{
	int tnSqr = tileNum*tileNum;
	int row_length = oriNum*tnSqr;
	CvMat *featMat = cvCreateMat(patchDiv*patchDiv,row_length,CV_32FC1);
	float patchLineRatio=sqrt(patchAreaRatio);
	float patch_width=iplImage->width*patchLineRatio;
	float patch_height=iplImage->height*patchLineRatio;
	float x_step = float(iplImage->width-patch_width)/(patchDiv-1);
	float y_step = float(iplImage->height-patch_height)/(patchDiv-1);
	CvPoint2D32f center;
	bool *accList = new bool[featMat->rows];
	for(int i=0;i<featMat->rows;i++)
		accList[i] = false;
	int rowCount = 0;
	for(int oriIndex=0;oriIndex<oriNum;oriIndex++)
	{
		float ori = PI*oriIndex/oriNum;
		CvGabor *gabor = new CvGabor(ori,iNu,dSigma,dF);
		filteredImage = cvCreateImage(cvSize(iplImage->width,iplImage->height),IPL_DEPTH_8U,1);
		
		char buf[200];

		gabor->conv_img(iplImage,filteredImage,CV_GABOR_MAG);

		CvPoint2D32f left_up,right_down;
		int rowIndex=0;
		for(int cx=0;cx<patchDiv;cx++)
		{
			for(int cy=0;cy<patchDiv;cy++)
			{
				center.x=patch_width/2 + x_step*cx;
				center.y=patch_height/2 + y_step*cy;
				for(int s=0;s<tileNum;s++)
				{
					for(int t=0;t<tileNum;t++)
					{
						left_up.x=center.x-patch_width/2.0+patch_width*s/tileNum;
						left_up.y=center.y-patch_height/2.0+patch_height*t/tileNum;
						right_down.x=left_up.x+patch_width/tileNum;
						right_down.y=left_up.y+patch_height/tileNum;

						float featVal=queryAVG(left_up,right_down);
						
						
						if(featVal > 1.0 && !accList[rowIndex])
						{
							rowCount++;
							accList[rowIndex] = true;
						}
						featMat->data.fl[rowIndex*row_length + oriIndex*tnSqr + t*tileNum + s] = featVal;
					}
				}
				rowIndex++;
			}
		}
		cvReleaseImage(&filteredImage);
		delete gabor;
	}
	CvMat *retMat = cvCreateMat(rowCount,featMat->cols,CV_32FC1);
	for(int i=0,ri=0;i<featMat->rows;i++)
	{
		if(accList[i])
		{
			for(int c=0;c<row_length;c++)
				retMat->data.fl[ri*row_length+c] = featMat->data.fl[i*row_length+c];
			ri++;
		}
	}
	delete []accList;
	cvReleaseMat(&featMat);
	return retMat;
}

Image::~Image()
{
	cvReleaseImage(&filteredImage);
	cvReleaseImage(&iplImage);
}