#include "cvgabor.h"

class Image
{
public:
	IplImage *iplImage,*filteredImage;
	void set(float patchAreaRatio,int patchDiv,int tileNum,int oriNum,int iNu,float dSigma,float dF);
	CvMat* getFeatMat();
	~Image();
private:
	float patchAreaRatio;
	int patchDiv;
	int tileNum;
	int oriNum;
	int iNu;
	float dSigma;
	float dF;
	float queryAVG(CvPoint2D32f &left_up, CvPoint2D32f &right_down);
};