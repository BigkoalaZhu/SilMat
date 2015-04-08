#ifndef SILHOUETTEPAINT_H
#define SILHOUETTEPAINT_H

#include<QWidget>
#include<QMainWindow>
#include<QPaintEvent>
#include<QPainter>
#include<QPainterPath>
#include<QPoint>
#include<QMouseEvent>
#include<QPolygon>
#include<QRegion>
#include<QMessageBox>
#include<QDir>

#include "TriMesh.h"
#include "Eigen\dense"
#include "silhouetteoperation.h"

class SilhouettePaint : public QWidget
{
	Q_OBJECT

public:
	SilhouettePaint(QWidget *parent);
	~SilhouettePaint();

	void ReadMeshFile(QString filename);
	void saveSilhouettes(QString file_dic);
	void loadCorrespondence(QString filename, int index);
	void loadMeshList(QString filename);
	TriMesh* GetMesh(int idx){return meshes[idx];};

private:
	void paintEvent(QPaintEvent *);
	void resizeEvent(QResizeEvent *);
	void mousePressEvent( QMouseEvent * event );

	void findCorrespondence(QPoint pos);
	QVector<QPoint> findCorrespondence_all(QPoint pos, int silhouette_idx);
	bool inTri(Eigen::Vector2d A,Eigen::Vector2d B, Eigen::Vector2d C, Eigen::Vector2d P , double &u, double &v);
	bool collisionSegment(Eigen::Vector2d A1,Eigen::Vector2d B1,Eigen::Vector2d A2,Eigen::Vector2d B2);
	bool collisionTris(Eigen::Vector2d A1,Eigen::Vector2d B1, Eigen::Vector2d C1,Eigen::Vector2d A2,Eigen::Vector2d B2, Eigen::Vector2d C2);

	void loadCameras();
	void writeSilhoutte(QString path, int idx_mesh, int idx_camera, bool normal_w);
	void readSilhoutte(QString path, int idx_mesh, int idx_camera, bool normal_r);
	void normalizeSilhouette();

	void findCorrespondencePoints();

protected:
	QString workspace_path;
	QVector<QVector<Eigen::Vector3f>> tmp_Silhouette;
	QVector<QPoint *> tmp_Silhouettes_normalized;
	double tmp_scale;
	double tmp_offset_x;
	double tmp_offset_y;
	QVector<QPair<QPoint,int>> tmp_select_point;
	QVector<QPoint> tmp_display_select_point;
	int pre_display_num;

	SilhouetteOperation *silOp;

	QVector<TriMesh *> meshes;
	QVector<QString> meshes_names;
	QVector<QVector<Eigen::VectorXf>> meshes_corr;
	QVector<double> length;

	int camera_num;
	Eigen::Vector3f *camera_direction;
	Eigen::Vector3f *camera_up;

	double *scale;
	double *offset_x;
	double *offset_y;

	int display_num;
	QVector<QVector<QPoint *>> Silhouettes_normalized;
	QVector<QVector<bool>> Silhouettes_visible;
	QVector<QVector<QVector<Eigen::Vector3f>>> Silhouettes;
	QVector<Eigen::Vector2f> Silhouettes_length;
	QVector<Eigen::Vector2f> Silhouettes_center;
	QVector<QVector<QPoint>> select_point;
	QVector<QVector<bool>> select_point_flag;

public slots:
	void calcProjected();
	void changeDisplayIdx(int idx){display_num = idx-1;update();};

signals:
	void mesh_num_changed(int);
};

#endif // SILHOUETTEPAINT_H
