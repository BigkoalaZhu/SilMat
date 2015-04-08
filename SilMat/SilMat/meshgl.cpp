#include "meshgl.h"
#include <QtGui>

GLfloat LightAmbient[]= { 0.05f, 0.05f, 0.05f, 0.5f };
GLfloat LightDiffuse[]= { 0.05f, 0.05f, 0.05f, 0.5f };
GLfloat Zero[]= { 0.0f, 0.0f, 0.0f, 0.0f };
GLfloat LightPosition[]= { 0.0f, -60.0f, 30.0f, 1.0f };

MeshGL::MeshGL(QWidget * parent)
{
	xRot = yRot = 0;
	scale = 1;
	tranx = trany = 0;
	valid = false;
}

void MeshGL::LoadMesh(TriMesh *m)
{
	mesh = m;
	Eigen::Vector3d L = Eigen::Vector3d((mesh->bbox.max - mesh->bbox.min)[0],(mesh->bbox.max - mesh->bbox.min)[1],(mesh->bbox.max - mesh->bbox.min)[2]);
	length = L.norm();
	valid = true;
}

void MeshGL::ReadMeshFile(QString filename)
{
	mesh = TriMesh::read(filename.toLocal8Bit().data());
	mesh->need_bbox();
	mesh->need_normals();
	Eigen::Vector3d L = Eigen::Vector3d((mesh->bbox.max - mesh->bbox.min)[0],(mesh->bbox.max - mesh->bbox.min)[1],(mesh->bbox.max - mesh->bbox.min)[2]);
	length = L.norm();
	valid = true;
}

void MeshGL::initializeGL()
{
	glShadeModel(GL_SMOOTH);							// Enable Smooth Shading
	glClearColor(0.8f, 0.8f, 0.8f, 0.0f);				// grey Background
	glClearDepth(1.0f);									// Depth Buffer Setup
	glEnable(GL_DEPTH_TEST);							// Enables Depth Testing
	glDepthFunc(GL_LEQUAL);								// The Type Of Depth Testing To Do
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);	// Really Nice Perspective Calculations
}

void MeshGL::resizeGL(int width, int height)
{
	if (height==0)										// Prevent A Divide By Zero By
	{
		height=1;										// Making Height Equal One
	}

	glViewport(0,0,width,height);						// Reset The Current Viewport

	glMatrixMode(GL_PROJECTION);						// Select The Projection Matrix
	glLoadIdentity();									// Reset The Projection Matrix

	// Calculate The Aspect Ratio Of The Window
	gluPerspective(45.0f,(GLfloat)width/(GLfloat)height,0.1f,10000.0f);

	glMatrixMode(GL_MODELVIEW);							// Select The Modelview Matrix
	glLoadIdentity();									// Reset The Modelview Matrix	
}

void MeshGL::mousePressEvent( QMouseEvent * event )
{
	if(!valid)
		return;
	setFocus();
	lastPos = event->pos();
}

void MeshGL::wheelEvent(QWheelEvent *event)
{
	if(event->orientation() == Qt::Vertical)
	{
		scale += (double)(event->delta()) / 10000;
		event->Quit;
		update();
	}
}

void MeshGL::mouseMoveEvent( QMouseEvent * event )
{
	if(!valid)
		return;
	if(event->buttons()&Qt::LeftButton)
	{
		GLfloat dx = GLfloat(event->x() - lastPos.x())/width();
		GLfloat dy = GLfloat(event->y() - lastPos.y())/height();

		xRot += 180*dy;
		yRot += 180*dx;

		update();
	}
	if(event->buttons()&Qt::RightButton)
	{
		GLfloat dx = GLfloat(event->x() - lastPos.x())/width();
		GLfloat dy = GLfloat(event->y() - lastPos.y())/height();

		tranx += dx;
		trany += dy;

		update();
	}
	lastPos = event->pos();
}

void MeshGL::LightOn()
{
	// Set up a light
	LightPosition[0] = mesh->bbox.max[0]*100;
	LightPosition[1] = mesh->bbox.max[1]*100;
	LightPosition[2] = mesh->bbox.max[2]*100;
	glLightfv(GL_LIGHT1, GL_AMBIENT, LightAmbient);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, LightDiffuse);
	glLightfv(GL_LIGHT1, GL_SPECULAR, Zero);
	glLightfv(GL_LIGHT1, GL_POSITION, LightPosition);
	glEnable(GL_LIGHT1);

	LightPosition[0] = mesh->bbox.min[0]*100;
	LightPosition[1] = mesh->bbox.min[1]*100;
	LightPosition[2] = mesh->bbox.min[2]*100;
	glLightfv(GL_LIGHT2, GL_AMBIENT, LightAmbient);
	glLightfv(GL_LIGHT2, GL_DIFFUSE, LightDiffuse);
	glLightfv(GL_LIGHT2, GL_SPECULAR, Zero);
	glLightfv(GL_LIGHT2, GL_POSITION, LightPosition);
	glEnable(GL_LIGHT2);

	LightPosition[0] = mesh->bbox.min[0]*100;
	LightPosition[1] = mesh->bbox.max[1]*100;
	LightPosition[2] = mesh->bbox.min[2]*100;
	glLightfv(GL_LIGHT3, GL_AMBIENT, LightAmbient);
	glLightfv(GL_LIGHT3, GL_DIFFUSE, LightDiffuse);
	glLightfv(GL_LIGHT3, GL_SPECULAR, Zero);
	glLightfv(GL_LIGHT3, GL_POSITION, LightPosition);
	glEnable(GL_LIGHT3);

	LightPosition[0] = mesh->bbox.max[0]*100;
	LightPosition[1] = mesh->bbox.min[1]*100;
	LightPosition[2] = mesh->bbox.max[2]*100;
	glLightfv(GL_LIGHT4, GL_AMBIENT, LightAmbient);
	glLightfv(GL_LIGHT4, GL_DIFFUSE, LightDiffuse);
	glLightfv(GL_LIGHT4, GL_SPECULAR, Zero);
	glLightfv(GL_LIGHT4, GL_POSITION, LightPosition);
	glEnable(GL_LIGHT4);

	LightPosition[0] = mesh->bbox.min[0]*100;
	LightPosition[1] = mesh->bbox.max[1]*100;
	LightPosition[2] = mesh->bbox.max[2]*100;
	glLightfv(GL_LIGHT5, GL_AMBIENT, LightAmbient);
	glLightfv(GL_LIGHT5, GL_DIFFUSE, LightDiffuse);
	glLightfv(GL_LIGHT5, GL_SPECULAR, Zero);
	glLightfv(GL_LIGHT5, GL_POSITION, LightPosition);
	glEnable(GL_LIGHT5);

	LightPosition[0] = mesh->bbox.max[0]*100;
	LightPosition[1] = mesh->bbox.min[1]*100;
	LightPosition[2] = mesh->bbox.min[2]*100;
	glLightfv(GL_LIGHT6, GL_AMBIENT, LightAmbient);
	glLightfv(GL_LIGHT6, GL_DIFFUSE, LightDiffuse);
	glLightfv(GL_LIGHT6, GL_SPECULAR, Zero);
	glLightfv(GL_LIGHT6, GL_POSITION, LightPosition);
	glEnable(GL_LIGHT6);

	LightPosition[0] = mesh->bbox.max[0]*100;
	LightPosition[1] = mesh->bbox.max[1]*100;
	LightPosition[2] = mesh->bbox.min[2]*100;
	glLightfv(GL_LIGHT7, GL_AMBIENT, LightAmbient);
	glLightfv(GL_LIGHT7, GL_DIFFUSE, LightDiffuse);
	glLightfv(GL_LIGHT7, GL_SPECULAR, Zero);
	glLightfv(GL_LIGHT7, GL_POSITION, LightPosition);
	glEnable(GL_LIGHT7);
}

void MeshGL::paintGL()
{
	if(!valid)
		return;
	
	LightOn();

	glEnable(GL_LIGHTING);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);	// Clear Screen And Depth Buffer
	glLoadIdentity();									// Reset The Current Modelview Matrix

	gluLookAt(0,0,scale*length*1.5,0,0,0,0,1,0);
	glTranslatef( tranx*length-mesh->bbox.center()[0],  -trany*length-mesh->bbox.center()[1], -mesh->bbox.center()[2] );
	glRotatef(xRot,1,0,0);
	glRotatef(yRot,0,1,0);

	glBegin(GL_TRIANGLES);
	for (int i = 0; i < mesh->faces.size(); i++)
	{
		for (int j = 0; j < 3; j++)
		{
			int index = mesh->faces[i][j];

			glNormal3fv(mesh->normals[index]);
			glVertex3fv(mesh->vertices[index]);
		}
	}
	glEnd();
	glFlush();
}


MeshGL::~MeshGL()
{

}
