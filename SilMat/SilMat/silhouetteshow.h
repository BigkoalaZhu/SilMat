#ifndef SILHOUETTESHOW_H
#define SILHOUETTESHOW_H

#include <QWidget>
#include <QMainWindow>
#include <QPaintEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPoint>
#include <QMouseEvent>
#include <QPolygon>
#include <QRegion>
#include <QMessageBox>
#include <QDir>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "TriMesh.h"
#include "SilhouetteOperation.h"

class SilhouetteShow : public QWidget
{
	Q_OBJECT

public:
	SilhouetteShow(QWidget *parent);
	~SilhouetteShow();

	void LoadSilhouetteImage(QString filename);
	void findBestView(QString lib);
	camera_setting GetCameraSetting(){return settings;};
	void SetCameraSetting(camera_setting s){settings = s;};
	void CalcProjectedSilhouette();
	QString GetMeshPath(){return mesh_path;};
	void SetMeshPath(QString name){mesh_path = name;};
	void GetCorrespondence();


private:
	void paintEvent(QPaintEvent *);
	Eigen::Vector3f GetIndividualCorrespondence(QPoint *pos, double sc, double off_x, double off_y, Eigen::Vector2f translation, Eigen::Matrix2f rotation, int height);
	bool inTri(Eigen::Vector2d A,Eigen::Vector2d B, Eigen::Vector2d C, Eigen::Vector2d P, double &u, double &v);

protected:
	QString *image_name;
	QImage *image;

	QString Filefolder_lib;
	SilhouetteOperation *analysis;
	camera_setting settings;

	TriMesh * mesh;
	QString mesh_path;
	QVector<QPoint *> Silhouettes_normalized;
	QVector<QPoint *> Silhouettes_sketch;
	QVector<QVector<Eigen::Vector3f>> Silhouettes;
	QVector<bool> Silhouettes_visible;
	double scale;
	double offset_x;
	double offset_y;
	Eigen::Vector2f Silhouettes_length;
	Eigen::Vector2f Silhouettes_center;
	double length;

public slots:

	
};

#endif // SILHOUETTESHOW_H
