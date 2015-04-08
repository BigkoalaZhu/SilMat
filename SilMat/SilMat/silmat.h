#ifndef SILMAT_H
#define SILMAT_H

#include <QtWidgets/QMainWindow>
#include "ui_silmat.h"
#include "meshgl.h"

class SilMat : public QMainWindow
{
	Q_OBJECT

public:
	SilMat(QWidget *parent = 0);
	~SilMat();

private:
	Ui::SilMatClass ui;

	QVector<QString> filelist;

public slots:
	void LoadAgentMesh();
	void SaveSilhouettes();
	void mesh_num_set(int num);
	void DisplayModel();

	void LoadSilhoueeteA();
	void FindBestView();
	void LoadSilhoueeteB();
	void FindBestView_2();

};

#endif // SILMAT_H
