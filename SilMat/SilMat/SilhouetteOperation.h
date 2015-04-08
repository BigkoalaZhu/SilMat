#pragma once
#include "QString.h"
#include "Eigen\dense"
#include "sbsr\Retriever.h"

struct camera_setting{
	Eigen::Vector3f up_direction;
	Eigen::Vector3f front_direction;
	Eigen::Vector3f right_direction;

	int rotation_index;
};


class SilhouetteOperation
{
public:
	SilhouetteOperation(void);
	~SilhouetteOperation(void);
	void Initialization(QString filefold);
	void FindMatchSilhouette(QString image);
	camera_setting GetBestCameraSetting();
	QString GetBestModel();

private:
	IplImage* rotateImage(IplImage* img ,int degree);

protected:
	Retriever *retriever;
	int idx_rotation;


};

