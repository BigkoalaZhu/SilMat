#include "silhouetteshow.h"

SilhouetteShow::SilhouetteShow(QWidget *parent)
{
	image_name == NULL;
	image = NULL;
	mesh = NULL;
	analysis = new SilhouetteOperation();
}

SilhouetteShow::~SilhouetteShow()
{

}

void SilhouetteShow::LoadSilhouetteImage(QString filename)
{
	image_name = new QString(filename);
	image = new QImage(filename);
	update();
}

void SilhouetteShow::paintEvent(QPaintEvent *event)
{
	if(image == NULL && Silhouettes_normalized.size() == 0)
		return;
	QPainter paint(this);
	if(image != NULL)
		paint.drawImage(0,0,*image);
	if(Silhouettes_normalized.size() != 0)
	{
		QBrush bruch;
		bruch.setColor(Qt::black);
		bruch.setStyle(Qt::SolidPattern);
		paint.setBrush(bruch);
		paint.setPen(Qt::black);
		for (int i = 0; i < Silhouettes_normalized.size(); i++)
		{
			paint.drawPolygon(Silhouettes_normalized[i],3);
		}
	}

}

void SilhouetteShow::findBestView(QString lib)
{
	Filefolder_lib = lib;
	analysis->Initialization(Filefolder_lib);
	analysis->FindMatchSilhouette(*image_name);
	settings = analysis->GetBestCameraSetting();
	mesh_path = analysis->GetBestModel();
}

void SilhouetteShow::CalcProjectedSilhouette()
{
	if(mesh == NULL)
	{
		mesh = TriMesh::read(mesh_path.toLocal8Bit().data());
		mesh->need_bbox();
		mesh->need_normals();
		Eigen::Vector3d L = Eigen::Vector3d((mesh->bbox.max - mesh->bbox.min)[0],(mesh->bbox.max - mesh->bbox.min)[1],(mesh->bbox.max - mesh->bbox.min)[2]);
		length = L.norm();
	}

	Silhouettes_visible.resize(mesh->faces.size());
	Silhouettes_visible.fill(true);
	Silhouettes.clear();

	double maxX = -1e9;
	double maxY = -1e9;
	double minX = 1e9;
	double minY = 1e9;

	for (int j = 0; j < mesh->faces.size(); j++)
	{
		QVector<Eigen::Vector3f> tri;
		tri.resize(3);
		for (int k = 0; k < 3; k++)
		{
			int index = mesh->faces[j][k];

			Eigen::Vector3f vertex = Eigen::Vector3f(mesh->vertices[index][0],mesh->vertices[index][1],mesh->vertices[index][2]);

			double tmpx = (vertex - settings.front_direction*length).dot(settings.up_direction);
			double tmpy = (vertex - settings.front_direction*length).dot(settings.right_direction);
			double tmpz = (vertex - settings.front_direction*length).dot(-settings.front_direction);
			tri[k] = Eigen::Vector3f(tmpx,tmpy,tmpz);
			if (maxX < tmpx)
				maxX = tmpx;
			if (maxY < tmpy)
				maxY = tmpy;
			if (minX > tmpx)
				minX = tmpx;
			if (minY > tmpy)
				minY = tmpy;
		}
		Silhouettes.push_back(tri);
	}

	int w = width();
	int h = height();
	Silhouettes_length[0] = maxX - minX;
	Silhouettes_length[1] = maxY - minY;
	Silhouettes_center[0] = (maxX + minX)/2;
	Silhouettes_center[1] = (maxY + minY)/2;
	scale = Silhouettes_length[0]/h > Silhouettes_length[1]/w ? 1.3*Silhouettes_length[0]/h:1.3*Silhouettes_length[1]/w;
	offset_x = h/2.0f - Silhouettes_center[0]/scale;
	offset_y = w/2.0f - Silhouettes_center[1]/scale;

	Eigen::Matrix2f rotation = Eigen::Matrix2f::Identity();
	double degree = M_PI/2*settings.rotation_index;
	rotation(0,0) = cos(degree);
	rotation(0,1) = -sin(degree);
	rotation(1,0) = sin(degree);
	rotation(1,1) = cos(degree);

	Eigen::Vector2f translation;
	translation[0] = -Silhouettes_center[0]/scale - offset_x;
	translation[1] = -Silhouettes_center[1]/scale - offset_y;

	for (int j = 0; j < Silhouettes.size(); j++)
	{
		QPoint *tri = new QPoint[3]; 
		for (int k = 0; k < 3; k++)
		{
			int tmpx = ( Silhouettes[j][k][0] )/scale + offset_x ;
			int tmpy = ( Silhouettes[j][k][1] )/scale + offset_y ;
			Eigen::Vector2f tmp_point(tmpy,h-tmpx);
			tmp_point = tmp_point + translation;
			tmp_point = rotation*tmp_point;
			tmp_point = tmp_point - translation;
			tri[k].setX(tmp_point[0]);
			tri[k].setY(tmp_point[1]);
		}
		Silhouettes_normalized.push_back(tri);
	}
	update();
}

void SilhouetteShow::GetCorrespondence()
{

}