#include "silhouetteshow.h"

SilhouetteShow::SilhouetteShow(QWidget *parent)
{
	image_name = NULL;
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

bool SilhouetteShow::inTri(Eigen::Vector2d A,Eigen::Vector2d B, Eigen::Vector2d C, Eigen::Vector2d P, double &u, double &v)
{
	u = -1;
	v = -1;

	Eigen::Vector2d v0 = C - A ;
    Eigen::Vector2d v1 = B - A ;
    Eigen::Vector2d v2 = P - A ;

    float dot00 = v0.dot(v0) ;
    float dot01 = v0.dot(v1) ;
    float dot02 = v0.dot(v2) ;
    float dot11 = v1.dot(v1) ;
    float dot12 = v1.dot(v2) ;

    float inverDeno = 1 / (dot00 * dot11 - dot01 * dot01) ;

    u = (dot11 * dot02 - dot01 * dot12) * inverDeno ;
    if (u < 0 - 1e-3 || u > 1 + 1e-3) // if u out of range, return directly
    {
        return false ;
    }

    v = (dot00 * dot12 - dot01 * dot02) * inverDeno ;
    if (v < 0 - 1e-3 || v > 1 + 1e-3) // if v out of range, return directly
    {
        return false ;
    }

    return u + v <= 1 ;
}

Eigen::Vector3f SilhouetteShow::GetIndividualCorrespondence(QPoint *pos, double sc, double off_x, double off_y, Eigen::Vector2f translation, Eigen::Matrix2f rotation, int height)
{
	Eigen::Vector2f P;
	Eigen::Vector2d Q;
	P[0] = pos->x();
	P[1] = pos->y();
	P = P + translation;
	P = rotation.inverse()*P;
	P = P - translation;

	Q[1] = (P[0] - off_y)*sc;
	Q[0] = (height - P[1] - off_x)*sc;

	int Tri_num = Silhouettes.size();
	double depth = 1e9;
	double tmpu,tmpv,u,v;
	int idx = -1;
	for (int i = 0; i < Tri_num; i++)
	{
		Eigen::Vector2d A = Eigen::Vector2d(Silhouettes[i][0][0],Silhouettes[i][0][1]);
		Eigen::Vector2d B = Eigen::Vector2d(Silhouettes[i][1][0],Silhouettes[i][1][1]);
		Eigen::Vector2d C = Eigen::Vector2d(Silhouettes[i][2][0],Silhouettes[i][2][1]);

		if(inTri(A,B,C,Q,tmpu,tmpv) && Silhouettes[i][0][2] < depth)
		{
			idx = i;
			depth = Silhouettes[i][0][2];
			u = tmpu;
			v = tmpv;
		}
	}

	Eigen::Vector3f result(idx, u, v);
	return result;
}

void SilhouetteShow::GetCorrespondence()
{
	int h = 256;
	int w = 256;
	double scale_sketch = Silhouettes_length[0]/h > Silhouettes_length[1]/w ? 1.3*Silhouettes_length[0]/h:1.3*Silhouettes_length[1]/w;
	double offset_x_sketch = h/2.0f - Silhouettes_center[0]/scale_sketch;
	double offset_y_sketch = w/2.0f - Silhouettes_center[1]/scale_sketch;

	Eigen::Vector2f translation;
	translation[0] = -Silhouettes_center[0]/scale_sketch - offset_x_sketch;
	translation[1] = -Silhouettes_center[1]/scale_sketch - offset_y_sketch;

	Eigen::Matrix2f rotation = Eigen::Matrix2f::Identity();
	double degree = M_PI/2*settings.rotation_index;
	rotation(0,0) = cos(degree);
	rotation(0,1) = -sin(degree);
	rotation(1,0) = sin(degree);
	rotation(1,1) = cos(degree);

	for (int j = 0; j < Silhouettes.size(); j++)
	{
		QPoint *tri = new QPoint[3]; 
		for (int k = 0; k < 3; k++)
		{
			int tmpx = ( Silhouettes[j][k][0] )/scale_sketch + offset_x_sketch ;
			int tmpy = ( Silhouettes[j][k][1] )/scale_sketch + offset_y_sketch ;
			Eigen::Vector2f tmp_point(tmpy,h-tmpx);
			tmp_point = tmp_point + translation;
			tmp_point = rotation*tmp_point;
			tmp_point = tmp_point - translation;
			tri[k].setX(tmp_point[0]);
			tri[k].setY(tmp_point[1]);
		}
		Silhouettes_sketch.push_back(tri);
	}

	QPixmap *pix = new QPixmap(w,h);
	QPainter *paint = new QPainter(pix);
	QBrush bk;
	paint->fillRect(0,0,w,h,Qt::white);

	QBrush bruch;
	bruch.setColor(Qt::black);
	bruch.setStyle(Qt::SolidPattern);
	paint->setBrush(bruch);
	paint->setPen(Qt::black);
	for (int j = 0; j < Silhouettes_sketch.size(); j++)
	{
		paint->drawPolygon(Silhouettes_sketch[j],3);
	}
	paint->end();
	pix->save("tmp.png");

	delete pix;
	delete paint;

	QVector<QVector<Eigen::Vector3f>> index_label;
	index_label.resize(w);
	for (int i = 0; i < w; i++)
		index_label[i].resize(h);

	IplImage* input = cvLoadImage("tmp.png",CV_LOAD_IMAGE_GRAYSCALE);
	double color;
	for(int i = 0;i < input->height; i++)
	{
		for(int j = 0; j < input->width; j++)
		{
			color = cvGet2D(input,i,j).val[0];
			if(color == 255)
				index_label[j][i] = Eigen::Vector3f(-1,-1,-1);
			else
			{
				QPoint *pos = new QPoint(j,i);
				index_label[j][i] = GetIndividualCorrespondence(pos,scale_sketch,offset_x_sketch,offset_y_sketch,translation,rotation,h);
			}
		}
	}

	std::ofstream ifs("corres.txt");
	for(int i = 0;i < w; i++)
	{
		for(int j = 0; j < h; j++)
		{
			ifs << index_label[j][i][0] << " ";
		}
		ifs << "\n";
	}
	ifs.close();

}