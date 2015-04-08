#include "SilhouetteOperation.h"
#include <opencv\cv.h>
#include <opencv\highgui.h>
#include <QFileInfo>

SilhouetteOperation::SilhouetteOperation(void)
{
	retriever = NULL;
}


SilhouetteOperation::~SilhouetteOperation(void)
{
}

void SilhouetteOperation::Initialization(QString filefold)
{
	QString tmpFile = filefold;
	int phi = 25, tha = 25;
	QFileInfo  info(tmpFile + "/data_" + QString::number(phi) + "_" + QString::number(tha) + ".bin");
	if(info.exists())
	{
		tmpFile = tmpFile + "/data_" + QString::number(phi) + "_" + QString::number(tha) + ".bin";
		tmpFile.replace("/","\\");
		retriever = new Retriever(tmpFile.toStdString());
		return;
	}
	else
	{
		tmpFile.replace("/","\\");
		retriever = new Retriever(200,10000,256,0.05,32,4,4,3,1.7,1.414,phi,tha);
		retriever->loadAndBuild(tmpFile.toStdString());
		tmpFile = tmpFile + "\\data_" + QString::number(phi) + "_" + QString::number(tha) + ".bin";
		retriever->saveData(tmpFile.toStdString());
		return;
	}
}

IplImage* SilhouetteOperation::rotateImage(IplImage* img,int degree)  
{  
    //旋转中心为图像中心  
    CvPoint2D32f center;    
    center.x=float (img->width/2.0+0.5);  
    center.y=float (img->height/2.0+0.5);  
    //计算二维旋转的仿射变换矩阵  
    float m[6];              
    CvMat M = cvMat( 2, 3, CV_32F, m );  
    cv2DRotationMatrix( center, degree,1, &M);  
    //变换图像，并用黑色填充其余值
	IplImage * img_rotate = cvCreateImage(cvSize(img->width, img->height),8,1);
    cvWarpAffine(img,img_rotate, &M,CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS,cvScalarAll(0) );
	return img_rotate;
}

void SilhouetteOperation::FindMatchSilhouette(QString image)
{
	QString tmpFile = image;
	tmpFile.replace("/","\\");
	IplImage* input = cvLoadImage(tmpFile.toLocal8Bit().data(),CV_LOAD_IMAGE_GRAYSCALE);
	cvCanny(input, input, 1, 1 * 3, 3);
	for (int i=0; i<input->height; i++)
		for(int j=0; j<input->width; j++)
			for (int k=0; k<input->nChannels; k++)
				input->imageData[i*input->widthStep + j*input->nChannels + k] = 255 - input->imageData[i*input->widthStep + j*input->nChannels + k];
	IplImage* input_right = NULL;
	IplImage* input_left = NULL;
	IplImage* input_down = NULL;
	input_right = rotateImage(input,90);
	input_down = rotateImage(input_right,90);
	input_left = rotateImage(input_down,90);

	double maxScore = 0;
	idx_rotation = 0;

	retriever->rankAll(input);
	maxScore = retriever->getScore(0);

	retriever->rankAll(input_right);
	if(retriever->getScore(0) > maxScore)
	{
		maxScore = retriever->getScore(0);
		idx_rotation = 1;
	}

	retriever->rankAll(input_down);
	if(retriever->getScore(0) > maxScore)
	{
		maxScore = retriever->getScore(0);
		idx_rotation = 2;
	}

	retriever->rankAll(input_left);
	if(retriever->getScore(0) > maxScore)
	{
		maxScore = retriever->getScore(0);
		idx_rotation = 3;
	}

	switch(idx_rotation)
	{
	case 0:
		retriever->rankAll(input);
		break;
	case 1:
		retriever->rankAll(input_right);
		break;
	case 2:
		retriever->rankAll(input_down);
		break;
	}
}

camera_setting SilhouetteOperation::GetBestCameraSetting()
{
	camera_setting tmp_result;
	Basis basis = retriever->getView(0);
	tmp_result.front_direction[0] = basis.front[0];
	tmp_result.front_direction[1] = basis.front[1];
	tmp_result.front_direction[2] = basis.front[2];
	tmp_result.up_direction[0] = basis.up[0];
	tmp_result.up_direction[1] = basis.up[1];
	tmp_result.up_direction[2] = basis.up[2];
	tmp_result.right_direction[0] = basis.right[0];
	tmp_result.right_direction[1] = basis.right[1];
	tmp_result.right_direction[2] = basis.right[2];
	tmp_result.rotation_index = idx_rotation;
	return tmp_result;
}

QString SilhouetteOperation::GetBestModel()
{
	std::string path = retriever->getPath(0);
	QString tmp = QString::fromStdString(path);
	return tmp;
}