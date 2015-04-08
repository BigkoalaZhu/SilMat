#include <string>
#include<io.h>
#include "sketch_gen.h"
#include <vector>

using namespace std;

struct BestView
{
	int mid;
	Basis viewIndex;
	double score;
};

class Retriever{
private:
	int viewWidth;
	int centerNum;
	int rowCount;
	int midIndex;
	int sampleNum;
	int featNum;
	float patchAreaRatio;
	int patchDiv;
	int tileNum;
	int oriNum;
	int iNu;
	float dSigma;
	float dF;
	int phiDiv;
	int thetaDiv;
	CvMat *sampleMat,*label,*featCenter,*vocabulary,*modelID;
	void loadPass(string &path,int layer,int pass);//pass=0 : get centers   pass=1 : build histogram lib 
	void loadPassSketch(string &path,int layer,int pass);
	void decomposeToHistogram(int *arr,CvMat *featMat);
	string libPath;
	vector<string> modelFilePath;
	BestView* modelRank;
public:
	int getLibSize();
	string getPath(int rank);
	Basis getView(int rank);
	double getScore(int rank);
	Retriever(){};
	Retriever(string dataPath,string libPath="");
	Retriever(int centerNum,int sampleNum,int viewWidth,
		float patchAreaRatio,int patchDiv,int tileNum,int oriNum,
		int iNu,float dSigma,float dF,int phiDiv,int thetaDiv);
	void rankAll(IplImage *inputImage);
	void loadAndBuild(string path);
	void loadAndBuildSketch(string path);
	void saveData(string path);
	void loadData(string path,string libPath="");
	~Retriever();
};