#ifndef MESHGL_H
#define MESHGL_H

#include <QtOpenGL>
#include "GL/GLU.h"
#include <QObject>
#include "TriMesh.h"
#include "Eigen\dense"

class MeshGL : public QGLWidget
{
	Q_OBJECT

public:
	MeshGL(QWidget * parent = 0);
	~MeshGL();

	void ReadMeshFile(QString filename);
	void LoadMesh(TriMesh *m);
private:
	

protected:
	void LightOn();
	void paintGL();
	void initializeGL();
	void resizeGL(int width, int height);
	void mousePressEvent( QMouseEvent * event );
	void mouseMoveEvent( QMouseEvent * event );
	void wheelEvent(QWheelEvent *event);


	bool valid;
	TriMesh *mesh;
	double length;
	double scale;
	/////////////////////////////////////////
	GLdouble xRot,yRot;
	GLdouble tranx,trany;
	QPoint lastPos;
};

#endif // MESHGL_H
