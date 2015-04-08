#include "sketch_gen.h"
#include <limits>
#include <opencv\highgui.h>
#include <iostream>

using namespace std;

#define PI 3.14159265358979

Basis* genOri(int phiDiv,int thetaDiv)
{
	Basis* ori = new Basis[phiDiv*thetaDiv];
	for(int i=0;i<phiDiv*thetaDiv;i++)
		ori[i] = getOri(i,phiDiv,thetaDiv);
	return ori;
}

Basis getOri(int index,int phiDiv,int thetaDiv)
{
	Basis ori;
	int pIdx = index/thetaDiv;
	int tIdx = index%thetaDiv;
	float phi = PI*(pIdx+1)/phiDiv+0.0001;
	float theta = 2*PI*tIdx/thetaDiv+0.0001;
	ori.front = sbsr_Vec(-sin(phi)*cos(theta),-sin(phi)*sin(theta),-cos(phi));
	ori.up = (sbsr_Vec(0,0,1).cross(ori.front));
	ori.up.normalize();
	ori.right = ori.front.cross(ori.up);
	return ori;
}

sbsr_Vec coordTrans(Basis &basis,sbsr_Vec vec)
{
	float r = basis.right.dot(vec);
	float u = basis.up.dot(vec);
	float f = basis.front.dot(vec);
	return sbsr_Vec(r,u,f);
}

void sweepTriangle(CvMat *depthMap,sbsr_Vec *point)
{
	int upMost,downMost;
	bool updated = false;
	int margin_x[2];
	float margin_z[2];
	for(int i=0;i<3;i++)
	{
		if(!updated)
		{
			updated = true;
			upMost = downMost = point[i][1];
		}
		else
		{
			upMost = (point[i][1] < upMost)? point[i][1] : upMost;
			downMost = (point[i][1] > downMost)? point[i][1] : downMost;
		}
	}
	for(int y=upMost;y<=downMost;y++)
	{
		int mIdx = 0;
		for(int i=0;i<3;i++)
		{
			sbsr_Vec &p1 = point[i];
			sbsr_Vec &p2 = point[(i+1)%3];
			float dy1 = p1[1] - y;
			float dy2 = p2[1] - y;
			if(dy1*dy2 <= 0)
			{
				margin_x[mIdx] = (dy2-dy1==0) ? p1[0] : p1[0]-dy1/(dy2-dy1)*(p2[0]-p1[0]);
				margin_z[mIdx] = (dy2-dy1==0) ? p1[2] : ceil(p1[2]-dy1/(dy2-dy1)*(p2[2]-p1[2]));
				mIdx++;
			}
		}
		if(mIdx!=2)
		{
			continue;
		}
		if(margin_x[0] == margin_x[1])
		{
			if(depthMap ->data.fl[y*depthMap->width+margin_x[0]] > margin_z[0])
				depthMap->data.fl[y*depthMap->width+margin_x[0]] = margin_z[0];
			continue;
		}
		int mx_s = (margin_x[0] < margin_x[1])? margin_x[0] : margin_x[1];
		int mx_e = (margin_x[0] >= margin_x[1])? margin_x[0] : margin_x[1];
		for(int x=mx_s;x<=mx_e;x++)
		{
			//printf("(%d,%d)\n",x,y);
			float z = (margin_x[1] == margin_x[0]) ? margin_z[0] : margin_z[0]+(x-margin_x[0])/(margin_x[1]-margin_x[0])*(margin_z[1]-margin_z[0]);
			if(depthMap ->data.fl[y*depthMap->width+x] > z)
				depthMap ->data.fl[y*depthMap->width+x] = z;
		}
	}
}

CvMat* genDepthMap(Basis &ori,GLMmodel *model,int length)
{
	float ratio = 0.9;
	CvMat *depthMap = cvCreateMat(length,length,CV_32FC1);
	float leftMost,rightMost,upMost,downMost;
	bool updated = false;
	for(int vi=1;vi<=model->numvertices;vi++)
	{
		sbsr_Vec point = coordTrans(ori,sbsr_Vec(model->vertices[vi*3],model->vertices[vi*3+1],model->vertices[vi*3+2]));
		if(!updated)
		{
			updated = true;
			leftMost = rightMost = point[0];
			upMost = downMost = point[1];
		}
		else
		{
			leftMost = (point[0] < leftMost)? point[0] : leftMost;
			rightMost = (point[0] > rightMost)? point[0] : rightMost;
			downMost = (point[1] < downMost)? point[1] : downMost;
			upMost = (point[1] > upMost)? point[1] : upMost;
		}
	}
	for(int i=0;i<length*length;i++)
		depthMap->data.fl[i] = std::numeric_limits<float>::infinity();
	float delta_w = rightMost - leftMost;
	float delta_h = upMost - downMost;
	float whMax = (delta_w > delta_h)? delta_w : delta_h;
	float ds = whMax/ratio*(1-ratio)/2;
	sbsr_Vec shift(-leftMost+(whMax-delta_w)/2+ds,-upMost-(whMax-delta_h)/2-ds,0);
	for(int fi=0;fi<model->numtriangles;fi++)
	{
		sbsr_Vec point[3];
		for(int vi=0;vi<3;vi++)
		{
			int vn = model->triangles[fi].vindices[vi];
			point[vi] = coordTrans(ori,sbsr_Vec(model->vertices[3*vn],model->vertices[3*vn+1],model->vertices[3*vn+2]));
			point[vi] = point[vi]+shift;
			point[vi][0] *= ratio*length/whMax;
			point[vi][1] *= -ratio*length/whMax;
		}
		sweepTriangle(depthMap,point);
	}
	return depthMap;
}

IplImage* depthMap_to_sketch(CvMat *depthMap)
{
	IplImage* image = cvCreateImage(cvGetSize(depthMap),8,1);
	for(int y=0;y<depthMap->rows;y++)
	{
		for(int x=0;x<depthMap->cols;x++)
		{
			CV_IMAGE_ELEM(image,char,y,x)=255;
			if(depthMap->data.fl[y*depthMap->cols+x] < std::numeric_limits<float>::infinity())
			{
				if(depthMap->data.fl[y*depthMap->cols+x-1] == std::numeric_limits<float>::infinity() ||
					depthMap->data.fl[y*depthMap->cols+x+1] == std::numeric_limits<float>::infinity() ||
					depthMap->data.fl[(y-1)*depthMap->cols+x] == std::numeric_limits<float>::infinity() ||
					depthMap->data.fl[(y+1)*depthMap->cols+x] == std::numeric_limits<float>::infinity())
				{
					CV_IMAGE_ELEM(image,char,y,x)=0;
				}
			}
		}
	}
	cvReleaseMat(&depthMap);
	/*cvNamedWindow("Generated Sketch", 1);  
	cvShowImage("Generated Sketch",image);  
	cvWaitKey(0);  
	cvDestroyWindow("Generated Sketch");*/
	return image;
}

