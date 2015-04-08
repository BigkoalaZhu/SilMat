#include "silhouettepaint.h"
const int MAXS = 60*1024*1024;



SilhouettePaint::SilhouettePaint(QWidget *parent)
{
	display_num = 0;
	pre_display_num = -1;
	loadCameras();
}

void SilhouettePaint::loadCameras()
{
	camera_num = 600;
	camera_direction = new Eigen::Vector3f[camera_num];
	camera_up = new Eigen::Vector3f[camera_num];
	///////////////////////////////////////////////////////
	int vertex_num = 20;
	QVector<Eigen::Vector3i> directions;
	std::ifstream ifs("cameras/edges.txt");
	for (int i = 0; i < vertex_num; i++)
	{
		int n,a,b,c;
		ifs >> n >> a >> b >> c;
		Eigen::Vector3i tmp(a-1,b-1,c-1);
		directions.push_back(tmp);
	}
	ifs.close();
	///////////////////////////////////////////////////////
	int filenums = 10;
	int index = 0;
	for (int i = 0; i < filenums; i++)
	{
		QString filename = "cameras/12_" + QString::number(i) + ".obj";
		ifs.open(filename.toLocal8Bit());
		QVector<Eigen::Vector3f> vertices;
		for (int j = 0; j < vertex_num; j++)
		{
			char tmp;
			float x,y,z;
			ifs >> tmp >> x >> y >> z;
			Eigen::Vector3f pos(x,y,z);
			vertices.push_back(pos);
		}
		for (int j = 0; j < vertex_num; j++)
		{
			camera_direction[index] = vertices[j];
			camera_direction[index+1] = vertices[j];
			camera_direction[index+2] = vertices[j];
			camera_up[index] = (vertices[directions[j][0]] - vertices[j]).normalized();
			camera_up[index+1] = (vertices[directions[j][1]] - vertices[j]).normalized();
			camera_up[index+2] = (vertices[directions[j][2]] - vertices[j]).normalized();
			index += 3;
		}
		ifs.close();
	}
	
}

SilhouettePaint::~SilhouettePaint()
{

}

void SilhouettePaint::ReadMeshFile(QString filename)
{
	TriMesh *mesh = new TriMesh;
	mesh = TriMesh::read(filename.toLocal8Bit().data());
	mesh->need_bbox();
	mesh->need_normals();
	Eigen::Vector3d L = Eigen::Vector3d((mesh->bbox.max - mesh->bbox.min)[0],(mesh->bbox.max - mesh->bbox.min)[1],(mesh->bbox.max - mesh->bbox.min)[2]);
	meshes.push_back(mesh);
	length.push_back(L.norm());
	emit mesh_num_changed(meshes.size());
}

void SilhouettePaint::paintEvent(QPaintEvent *event)
{
	if(meshes.size()!=0&&pre_display_num != display_num)
	{
		readSilhoutte(workspace_path + "work_dir/", display_num%meshes.size(), display_num/meshes.size(), true);
	}
	if (tmp_Silhouettes_normalized.size()==0)
		return;
	if (pre_display_num != display_num)
		findCorrespondencePoints();
	QPainter paint(this);
	QBrush bruch;
	bruch.setColor(Qt::black);
	bruch.setStyle(Qt::SolidPattern);
	paint.setBrush(bruch);
	paint.setPen(Qt::black);
	for (int i = 0; i < tmp_Silhouettes_normalized.size(); i++)
	{
		paint.drawPolygon(tmp_Silhouettes_normalized[i],3);
	}
	if(tmp_display_select_point.size() != 0)
	{
		for (int i = 0; i < tmp_display_select_point.size(); i++)
		{
			paint.setPen(Qt::GlobalColor(7+i));
			bruch.setColor(Qt::GlobalColor(7+i));
			bruch.setStyle(Qt::SolidPattern);
			paint.setBrush(bruch);
			paint.drawEllipse(tmp_display_select_point[i],2,2);
		}
	}
}

void SilhouettePaint::mousePressEvent( QMouseEvent * event )
{
	setFocus();
	QPair<QPoint,int> tmp_pair;
	tmp_pair.first = event->pos();
	tmp_pair.second = display_num;
	tmp_select_point.push_back(tmp_pair);

	findCorrespondencePoints();

	update();
}

void SilhouettePaint::findCorrespondencePoints()
{
	if (tmp_select_point.size() == 0)
		return;
	tmp_display_select_point.resize(tmp_select_point.size());

	pre_display_num = display_num;

	for (int i = 0; i < tmp_select_point.size(); i++)
	{
		readSilhoutte(workspace_path+"work_dir/",tmp_select_point[i].second%meshes.size(),tmp_select_point[i].second/meshes.size(),true);
		readSilhoutte(workspace_path+"work_dir/",tmp_select_point[i].second%meshes.size(),tmp_select_point[i].second/meshes.size(),false);

		Eigen::Vector2d P;
		P[1] = (tmp_select_point[i].first.x() - tmp_offset_y)*tmp_scale;
		P[0] = (height() - tmp_select_point[i].first.y() - tmp_offset_x)*tmp_scale;

		int Tri_num = tmp_Silhouette.size();
		double depth = 1e9;
		double tmpu,tmpv,u,v;
		int idx = -1;
		for (int j = 0; j < Tri_num; j++)
		{
			Eigen::Vector2d A = Eigen::Vector2d(tmp_Silhouette[j][0][0],tmp_Silhouette[j][0][1]);
			Eigen::Vector2d B = Eigen::Vector2d(tmp_Silhouette[j][1][0],tmp_Silhouette[j][1][1]);
			Eigen::Vector2d C = Eigen::Vector2d(tmp_Silhouette[j][2][0],tmp_Silhouette[j][2][1]);

			if(inTri(A,B,C,P,tmpu,tmpv) && tmp_Silhouette[j][0][2] < depth)
			{
				idx = j;
				depth = tmp_Silhouette[j][0][2];
				u = tmpu;
				v = tmpv;
			}
		}

		int tmp_idx[3];
		tmp_idx[0] = meshes[tmp_select_point[i].second%meshes.size()]->faces[idx][0];
		tmp_idx[1] = meshes[tmp_select_point[i].second%meshes.size()]->faces[idx][1];
		tmp_idx[2] = meshes[tmp_select_point[i].second%meshes.size()]->faces[idx][2];

		readSilhoutte(workspace_path+"work_dir/",display_num%meshes.size(),display_num/meshes.size(),true);
		readSilhoutte(workspace_path+"work_dir/",display_num%meshes.size(),display_num/meshes.size(),false);

		if (display_num%meshes.size() == tmp_select_point[i].second%meshes.size())
		{
			tmp_display_select_point[i] = tmp_Silhouettes_normalized[idx][0];
			tmp_display_select_point[i] += u*(tmp_Silhouettes_normalized[idx][2] - tmp_Silhouettes_normalized[idx][0]);
			tmp_display_select_point[i] += v*(tmp_Silhouettes_normalized[idx][1] - tmp_Silhouettes_normalized[idx][0]);
			continue;
		}

		int corres_mesh = display_num%meshes.size();
		int silhouette_idx = tmp_select_point[i].second%meshes.size();
		QPoint corres_pos[3];
		int flag = 0;


		for (int j = 0; j < meshes_corr[silhouette_idx].size(); j++)
		{
			if (meshes_corr[silhouette_idx][j][0] != corres_mesh)
				continue;
			if (meshes_corr[silhouette_idx][j][1] == tmp_idx[0])
			{
				corres_pos[0] = tmp_Silhouettes_normalized[meshes_corr[silhouette_idx][j][2]][0]*meshes_corr[silhouette_idx][j][3]
				+tmp_Silhouettes_normalized[meshes_corr[silhouette_idx][j][2]][1]*meshes_corr[silhouette_idx][j][4]
				+tmp_Silhouettes_normalized[meshes_corr[silhouette_idx][j][2]][2]*meshes_corr[silhouette_idx][j][5];
//				if(!Silhouettes_visible[i][meshes_corr[silhouette_idx][j][2]])
//					flag++;
			}
			if (meshes_corr[silhouette_idx][j][1] == tmp_idx[1])
			{
				corres_pos[1] = tmp_Silhouettes_normalized[meshes_corr[silhouette_idx][j][2]][0]*meshes_corr[silhouette_idx][j][3]
				+tmp_Silhouettes_normalized[meshes_corr[silhouette_idx][j][2]][1]*meshes_corr[silhouette_idx][j][4]
				+tmp_Silhouettes_normalized[meshes_corr[silhouette_idx][j][2]][2]*meshes_corr[silhouette_idx][j][5];
//				if(!Silhouettes_visible[i][meshes_corr[silhouette_idx][j][2]])
//					flag++;
			}
			if (meshes_corr[silhouette_idx][j][1] == tmp_idx[2])
			{
				corres_pos[2] = tmp_Silhouettes_normalized[meshes_corr[silhouette_idx][j][2]][0]*meshes_corr[silhouette_idx][j][3]
				+tmp_Silhouettes_normalized[meshes_corr[silhouette_idx][j][2]][1]*meshes_corr[silhouette_idx][j][4]
				+tmp_Silhouettes_normalized[meshes_corr[silhouette_idx][j][2]][2]*meshes_corr[silhouette_idx][j][5];
//				if(!Silhouettes_visible[i][meshes_corr[silhouette_idx][j][2]])
//					flag++;
			}
		}

		tmp_display_select_point[i] = corres_pos[0];
		tmp_display_select_point[i] += u*(corres_pos[2] - corres_pos[0]);
		tmp_display_select_point[i] += v*(corres_pos[1] - corres_pos[0]);
	}
}

void SilhouettePaint::loadMeshList(QString filename)
{
	QFile file(filename);
	if(!file.open(QIODevice::ReadOnly | QIODevice::Text)) {  
        return;  
    }

	workspace_path = filename.mid(0,filename.lastIndexOf("/")+1);
	
	while(!file.atEnd()) {
		QByteArray line = file.readLine();
		QString str(line);
		if (str[str.size()-1] == '\n')
			str = str.mid(0,str.size()-1);
		meshes_names.push_back(str);
	}

	for (int i = 0; i < meshes_names.size(); i++)
	{
		ReadMeshFile(workspace_path + meshes_names[i] + ".off");
		loadCorrespondence(workspace_path + meshes_names[i],i);
	}
}

bool SilhouettePaint::collisionTris(Eigen::Vector2d A1,Eigen::Vector2d B1, Eigen::Vector2d C1,Eigen::Vector2d A2,Eigen::Vector2d B2, Eigen::Vector2d C2)
{
	double u,v;
	if(collisionSegment(A1,B1,A2,B2))
		return true;
	if(collisionSegment(A1,B1,A2,C2))
		return true;
	if(collisionSegment(A1,B1,C2,B2))
		return true;
	if(collisionSegment(A1,C1,A2,B2))
		return true;
	if(collisionSegment(A1,C1,A2,C2))
		return true;
	if(collisionSegment(A1,C1,C2,B2))
		return true;
	if(collisionSegment(C1,B1,A2,B2))
		return true;
	if(collisionSegment(C1,B1,A2,C2))
		return true;
	if(collisionSegment(C1,B1,C2,B2))
		return true;
	if(inTri(A1,B1,C1,A2,u,v))
		return true;
	if(inTri(A1,B1,C1,B2,u,v))
		return true;
	if(inTri(A1,B1,C1,C2,u,v))
		return true;
	if(inTri(A2,B2,C2,A1,u,v))
		return true;
	if(inTri(A2,B2,C2,B1,u,v))
		return true;
	if(inTri(A2,B2,C2,C1,u,v))
		return true;
	return false;
}

bool SilhouettePaint::collisionSegment(Eigen::Vector2d v1,Eigen::Vector2d v2,Eigen::Vector2d v3,Eigen::Vector2d v4)
{
	double d = (v4[1]-v3[1])*(v2[0]-v1[0])-(v4[0]-v3[0])*(v2[1]-v1[1]);
    double u = (v4[0]-v3[0])*(v1[1]-v3[1])-(v4[1]-v3[1])*(v1[0]-v3[0]);
    double v = (v2[0]-v1[0])*(v1[1]-v3[1])-(v2[1]-v1[1])*(v1[0]-v3[0]);

	if (d < 0)
	{
		d = -d;
		u = -u;
		v = -v;
	}

	return (0 < u)&&(u <= d)&&(0 < v)&&(v <= d);
}

void SilhouettePaint::loadCorrespondence(QString filename, int index)
{
	int vnum = meshes[index]->vertices.size();
	QVector<Eigen::VectorXf> corres;
	for (int i = 0; i < meshes_names.size(); i++)
	{
		if(i == index)
			continue;
		QString path = filename + "/Blended_DenseMapPreceise_" + meshes_names[index] + "_to_" + meshes_names[i] + ".dense.preceise.map";
		std::ifstream ifs(path.toLocal8Bit().data());
		int v_index;
		float vx,vy,vz;
		Eigen::VectorXf tmpc(6);
		for (int j = 0; j < vnum; j++)
		{
			ifs >> v_index >> vx >> vy >> vz;
			tmpc[0] = i;
			tmpc[1] = j;
			tmpc[2] = v_index;
			tmpc[3] = vx;
			tmpc[4] = vy;
			tmpc[5] = vz;
			corres.push_back(tmpc);
		}
		ifs.close();
	}
	meshes_corr.push_back(corres);
}

QVector<QPoint> SilhouettePaint::findCorrespondence_all(QPoint pos, int silhouette_idx)
{
	QVector<QPoint> results;
	results.resize(Silhouettes.size()-1);

	Eigen::Vector2d P;
	P[1] = (pos.x() - offset_y[silhouette_idx])*scale[silhouette_idx];
	P[0] = (height() - pos.y() - offset_x[silhouette_idx])*scale[silhouette_idx];

	int Tri_num = Silhouettes[silhouette_idx].size();
	double depth = 1e9;
	double tmpu,tmpv,u,v;
	int idx = -1;
	for (int i = 0; i < Tri_num; i++)
	{
		Eigen::Vector2d A = Eigen::Vector2d(Silhouettes[silhouette_idx][i][0][0],Silhouettes[silhouette_idx][i][0][1]);
		Eigen::Vector2d B = Eigen::Vector2d(Silhouettes[silhouette_idx][i][1][0],Silhouettes[silhouette_idx][i][1][1]);
		Eigen::Vector2d C = Eigen::Vector2d(Silhouettes[silhouette_idx][i][2][0],Silhouettes[silhouette_idx][i][2][1]);

		if(inTri(A,B,C,P,tmpu,tmpv) && Silhouettes[silhouette_idx][i][0][2] < depth)
		{
			idx = i;
			depth = Silhouettes[silhouette_idx][i][0][2];
			u = tmpu;
			v = tmpv;
		}
	}

	int tmp_idx[3];
	tmp_idx[0] = meshes[silhouette_idx]->faces[idx][0];
	tmp_idx[1] = meshes[silhouette_idx]->faces[idx][1];
	tmp_idx[2] = meshes[silhouette_idx]->faces[idx][2];

	for (int i = 0; i < Silhouettes.size(); i++)
	{
		if (i%meshes.size() == silhouette_idx%meshes.size())
		{
			results[i] = Silhouettes_normalized[i][idx][0];
			results[i] += u*(Silhouettes_normalized[i][idx][2] - Silhouettes_normalized[i][idx][0]);
			results[i] += v*(Silhouettes_normalized[i][idx][1] - Silhouettes_normalized[i][idx][0]);
			continue;
		}

		int corres_mesh = i%meshes.size();
		QPoint corres_pos[3];
		int flag = 0;
		for (int j = 0; j < meshes_corr[silhouette_idx].size(); j++)
		{
			if (meshes_corr[silhouette_idx][j][0] != corres_mesh)
				continue;
			if (meshes_corr[silhouette_idx][j][1] == tmp_idx[0])
			{
				corres_pos[0] = Silhouettes_normalized[i][meshes_corr[silhouette_idx][j][2]][0]*meshes_corr[silhouette_idx][j][3]
								+Silhouettes_normalized[i][meshes_corr[silhouette_idx][j][2]][1]*meshes_corr[silhouette_idx][j][4]
								+Silhouettes_normalized[i][meshes_corr[silhouette_idx][j][2]][2]*meshes_corr[silhouette_idx][j][5];
				if(!Silhouettes_visible[i][meshes_corr[silhouette_idx][j][2]])
					flag++;
			}
			if (meshes_corr[silhouette_idx][j][1] == tmp_idx[1])
			{
				corres_pos[1] = Silhouettes_normalized[i][meshes_corr[silhouette_idx][j][2]][0]*meshes_corr[silhouette_idx][j][3]
								+Silhouettes_normalized[i][meshes_corr[silhouette_idx][j][2]][1]*meshes_corr[silhouette_idx][j][4]
								+Silhouettes_normalized[i][meshes_corr[silhouette_idx][j][2]][2]*meshes_corr[silhouette_idx][j][5];
				if(!Silhouettes_visible[i][meshes_corr[silhouette_idx][j][2]])
					flag++;
			}
			if (meshes_corr[silhouette_idx][j][1] == tmp_idx[2])
			{
				corres_pos[2] = Silhouettes_normalized[i][meshes_corr[silhouette_idx][j][2]][0]*meshes_corr[silhouette_idx][j][3]
								+Silhouettes_normalized[i][meshes_corr[silhouette_idx][j][2]][1]*meshes_corr[silhouette_idx][j][4]
								+Silhouettes_normalized[i][meshes_corr[silhouette_idx][j][2]][2]*meshes_corr[silhouette_idx][j][5];
				if(!Silhouettes_visible[i][meshes_corr[silhouette_idx][j][2]])
					flag++;
			}
		}

		if(flag > 1)
		{
			results[i] = QPoint(-1,-1);
			break;
		}

		results[i] = corres_pos[0];
		results[i] += u*(corres_pos[2] - corres_pos[0]);
		results[i] += v*(corres_pos[1] - corres_pos[0]);
	}
	return results;
}

void SilhouettePaint::findCorrespondence(QPoint pos)
{
	QPoint tmp_pos;
	for(int i = 0; i < select_point_flag.size(); i++)
	{
		select_point[i].push_back(tmp_pos);
		select_point_flag[i].push_back(true);
	}

	Eigen::Vector2d P;
	P[1] = (pos.x() - offset_y[display_num])*scale[display_num];
	P[0] = (height() - pos.y() - offset_x[display_num])*scale[display_num];

	int Tri_num = Silhouettes[display_num].size();
	double depth = 1e9;
	double tmpu,tmpv,u,v;
	int idx = -1;
	for (int i = 0; i < Tri_num; i++)
	{
		Eigen::Vector2d A = Eigen::Vector2d(Silhouettes[display_num][i][0][0],Silhouettes[display_num][i][0][1]);
		Eigen::Vector2d B = Eigen::Vector2d(Silhouettes[display_num][i][1][0],Silhouettes[display_num][i][1][1]);
		Eigen::Vector2d C = Eigen::Vector2d(Silhouettes[display_num][i][2][0],Silhouettes[display_num][i][2][1]);

		if(inTri(A,B,C,P,tmpu,tmpv) && Silhouettes[display_num][i][0][2] < depth)
		{
			idx = i;
			depth = Silhouettes[display_num][i][0][2];
			u = tmpu;
			v = tmpv;
		}
	}

	for (int i = 0; i < Silhouettes.size(); i++)
	{
		select_point[i][select_point[i].size()-1] = Silhouettes_normalized[i][idx][0];
		select_point[i][select_point[i].size()-1] += u*(Silhouettes_normalized[i][idx][2] - Silhouettes_normalized[i][idx][0]);
		select_point[i][select_point[i].size()-1] += v*(Silhouettes_normalized[i][idx][1] - Silhouettes_normalized[i][idx][0]);
	}

	for (int i = 0; i < Silhouettes.size(); i++)
	{
		for (int j = 0; j < Silhouettes[i].size(); j++)
		{
			if (j == idx)
				continue;
			TriMesh *mesh = meshes[0];
			if (mesh->faces[j][0] == mesh->faces[idx][0] || mesh->faces[j][0] == mesh->faces[idx][1] ||
				mesh->faces[j][0] == mesh->faces[idx][2] || mesh->faces[j][1] == mesh->faces[idx][0] ||
				mesh->faces[j][1] == mesh->faces[idx][1] || mesh->faces[j][1] == mesh->faces[idx][2] ||
				mesh->faces[j][2] == mesh->faces[idx][0] || mesh->faces[j][2] == mesh->faces[idx][1] ||
				mesh->faces[j][2] == mesh->faces[idx][2] )
				continue;
			Eigen::Vector2d A1 = Eigen::Vector2d(Silhouettes[i][j][0][0],Silhouettes[i][j][0][1]);
			Eigen::Vector2d B1 = Eigen::Vector2d(Silhouettes[i][j][1][0],Silhouettes[i][j][1][1]);
			Eigen::Vector2d C1 = Eigen::Vector2d(Silhouettes[i][j][2][0],Silhouettes[i][j][2][1]);

			Eigen::Vector2d A2 = Eigen::Vector2d(Silhouettes[i][idx][0][0],Silhouettes[i][idx][0][1]);
			Eigen::Vector2d B2 = Eigen::Vector2d(Silhouettes[i][idx][1][0],Silhouettes[i][idx][1][1]);
			Eigen::Vector2d C2 = Eigen::Vector2d(Silhouettes[i][idx][2][0],Silhouettes[i][idx][2][1]);
			if (collisionTris(A1,B1,C1,A2,B2,C2))
			{
				double d1 = Silhouettes[i][j][0][2] + Silhouettes[i][j][1][2] + Silhouettes[i][j][2][2];
				double d2 = Silhouettes[i][idx][0][2] + Silhouettes[i][idx][1][2] + Silhouettes[i][idx][2][2];
				if (d1 < d2)
				{
					select_point_flag[i][select_point_flag[i].size()-1] = false;
					break;
				}
			}
		}
	}
}

bool SilhouettePaint::inTri(Eigen::Vector2d A,Eigen::Vector2d B, Eigen::Vector2d C, Eigen::Vector2d P, double &u, double &v)
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

void SilhouettePaint::resizeEvent(QResizeEvent *event)
{
	if (Silhouettes.size()==0)
		return;
	int w = width();
	int h = height();
	Silhouettes_normalized.clear();
	Silhouettes_normalized.resize(camera_num);
	for (int i = 0; i < camera_num; i++)
	{
		scale[i] = Silhouettes_length[i][0]/h > Silhouettes_length[i][1]/w ? 1.1*Silhouettes_length[i][0]/h:1.1*Silhouettes_length[i][1]/w;
		offset_x[i] = h/2.0f - Silhouettes_center[i][0]/scale[i];
		offset_y[i] = w/2.0f - Silhouettes_center[i][1]/scale[i];

		for (int j = 0; j < Silhouettes[i].size(); j++)
		{
			QPoint *tri = new QPoint[3]; 
			for (int k = 0; k < 3; k++)
			{
				int tmpx = ( Silhouettes[i][j][k][0] )/scale[i] + offset_x[i] ;
				int tmpy = ( Silhouettes[i][j][k][1] )/scale[i] + offset_y[i] ;
				tri[k].setX(tmpy);
				tri[k].setY(h - tmpx);
			}
			Silhouettes_normalized[i].push_back(tri);
		}
	}
	update();
}

void SilhouettePaint::writeSilhoutte(QString path, int idx_mesh, int idx_camera, bool normal_w = false)
{
	if (!normal_w)
	{
		QString filename = path + meshes_names[idx_mesh] + "_" + QString::number(idx_camera);
		std::ofstream ofs(filename.toLocal8Bit().data());
		ofs << tmp_Silhouette.size() << "\n";
		for (int i = 0; i < tmp_Silhouette.size(); i++)
		{
			for (int j = 0; j < 3; j++)
			{
				for (int k = 0; k < 3; k++)
					ofs << tmp_Silhouette[i][j][k] << " ";
			}
			ofs << "\n";
		}
		ofs.close();
	}
	else
	{
		QString filename = path + meshes_names[idx_mesh] + "_" + QString::number(idx_camera) + "_N";
		std::ofstream ofs(filename.toLocal8Bit().data());
		ofs << tmp_Silhouettes_normalized.size() << " "
			<< tmp_scale << " "
			<< tmp_offset_x << " "
			<< tmp_offset_y << "\n";
		for (int i = 0; i < tmp_Silhouettes_normalized.size(); i++)
		{
			for (int j = 0; j < 3; j++)
			{
				ofs << tmp_Silhouettes_normalized[i][j].x() << " ";
				ofs << tmp_Silhouettes_normalized[i][j].y() << " ";
			}
			ofs << "\n";
		}
		ofs.close();
	}
}

void SilhouettePaint::readSilhoutte(QString path, int idx_mesh, int idx_camera, bool normal_r = false)
{
	if (!normal_r)
	{
		QString filename = path + meshes_names[idx_mesh] + "_" + QString::number(idx_camera);
		tmp_Silhouette.clear();

		QFile file( filename );
		if(!file.open(QIODevice::ReadOnly | QIODevice::Text)) {  
			return;  
		}

		int k = 0;
		int vnum = QString(file.readLine()).toInt();
		tmp_Silhouette.resize(vnum);
		while(!file.atEnd()) {
			QByteArray line = file.readLine();
			QStringList nums = QString(line).split(" ");
			QVector<Eigen::Vector3f> tri;
			tri.resize(3);
			for (int i = 0; i < 3; i++)
			{
				Eigen::Vector3f tmp;
				tmp[0] = nums[i*3].toFloat();
				tmp[1] = nums[i*3+1].toFloat();
				tmp[2] = nums[i*3+2].toFloat();
				tri[i] = tmp;
			}
			tmp_Silhouette[k] = tri;
			k++;
		}
		file.close();
	}
	else
	{
		QString filename = path + meshes_names[idx_mesh] + "_" + QString::number(idx_camera) + "_N";
		tmp_Silhouettes_normalized.clear();

		QFile file( filename );
		if(!file.open(QIODevice::ReadOnly | QIODevice::Text)) {  
			return;  
		}

		int k = 0;
		QByteArray line = file.readLine();
		QStringList nums = QString(line).split(" ");
		int vnum = nums[0].toInt();
		tmp_scale = nums[1].toDouble();
		tmp_offset_x = nums[2].toDouble();
		tmp_offset_y = nums[3].toDouble();
		tmp_Silhouettes_normalized.resize(vnum);
		while(!file.atEnd()) {
			line = file.readLine();
			nums = QString(line).split(" ");
			QPoint *tri = new QPoint[3];
			for (int i = 0; i < 3; i++)
			{
				QPoint tmp(nums[i*2].toInt(),nums[i*2+1].toInt());
				tri[i] = tmp;
			}
			tmp_Silhouettes_normalized[k] = tri;
			k++;
		}
		file.close();
	}
}

void SilhouettePaint::calcProjected()
{
	QString path = workspace_path + "work_dir/";
	QDir *work_dir = new QDir;
	bool exist = work_dir->exists(path);
	if(!exist)
		work_dir->mkdir(path);

	Silhouettes_length.resize(camera_num*meshes.size());
	Silhouettes_normalized.resize(camera_num*meshes.size());
	Silhouettes_center.resize(camera_num*meshes.size());
	scale = new double[camera_num*meshes.size()];
	offset_x = new double[camera_num*meshes.size()];
	offset_y = new double[camera_num*meshes.size()];

	for (int i = 0; i < camera_num; i++)
	{
		int mesh_index = 0;
		while (mesh_index < meshes.size())
		{
			QFileInfo  info(path + meshes_names[mesh_index] + "_" + QString::number(i));
			if(info.exists())
			{
				mesh_index++;
				continue;
			}
			TriMesh *mesh = meshes[mesh_index];
			tmp_Silhouette.clear();
			for (int j = 0; j < mesh->faces.size(); j++)
			{
				QVector<Eigen::Vector3f> tri;
				tri.resize(3);
				for (int k = 0; k < 3; k++)
				{
					int index = mesh->faces[j][k];

					Eigen::Vector3f vertex = Eigen::Vector3f(mesh->vertices[index][0],mesh->vertices[index][1],mesh->vertices[index][2]);

					double tmpx = (vertex - camera_direction[i]*length[mesh_index]).dot(camera_up[i]);
					double tmpy = (vertex - camera_direction[i]*length[mesh_index]).dot(camera_direction[i].cross(camera_up[i]));
					double tmpz = (vertex - camera_direction[i]*length[mesh_index]).dot(-camera_direction[i]);
					tri[k] = Eigen::Vector3f(tmpx,tmpy,tmpz);
				}
				tmp_Silhouette.push_back(tri);
			}
			writeSilhoutte(path,mesh_index,i);
			mesh_index++;
		}
	}

	int w = width();
	int h = height();

	////////////////////////////////////////////Visible
	Silhouettes_visible.resize(camera_num*meshes.size());
	for (int i = 0; i < camera_num*meshes.size(); i++)
	{
		QFileInfo  info(path + meshes_names[i%meshes.size()] + "_" + QString::number(i) + "_N");
		if(info.exists())
		{
			continue;
		}
		tmp_Silhouettes_normalized.clear();
		readSilhoutte(path,i%meshes.size(),i/meshes.size());
		Silhouettes_visible[i].resize(tmp_Silhouette.size());
		Silhouettes_visible[i].fill(true);
		double maxX = -1e9;
		double maxY = -1e9;
		double minX = 1e9;
		double minY = 1e9;
		for (int j = 0; j < tmp_Silhouette.size(); j++)
		{
			if (tmp_Silhouette[j][0][0] > maxX)
				maxX = tmp_Silhouette[j][0][0];
			if (tmp_Silhouette[j][1][0] > maxX)
				maxX = tmp_Silhouette[j][1][0];
			if (tmp_Silhouette[j][2][0] > maxX)
				maxX = tmp_Silhouette[j][2][0];
			if (tmp_Silhouette[j][0][1] > maxY)
				maxY = tmp_Silhouette[j][0][1];
			if (tmp_Silhouette[j][1][1] > maxY)
				maxY = tmp_Silhouette[j][1][1];
			if (tmp_Silhouette[j][2][1] > maxY)
				maxY = tmp_Silhouette[j][2][1];
			if (tmp_Silhouette[j][0][0] < minX)
				minX = tmp_Silhouette[j][0][0];
			if (tmp_Silhouette[j][1][0] < minX)
				minX = tmp_Silhouette[j][1][0];
			if (tmp_Silhouette[j][2][0] < minX)
				minX = tmp_Silhouette[j][2][0];
			if (tmp_Silhouette[j][0][1] < minY)
				minY = tmp_Silhouette[j][0][1];
			if (tmp_Silhouette[j][1][1] < minY)
				minY = tmp_Silhouette[j][1][1];
			if (tmp_Silhouette[j][2][1] < minY)
				minY = tmp_Silhouette[j][2][1];
/*			for (int k = 0; k <tmp_Silhouette.size(); k++)
			{
				if(j == k)
					continue;
				Eigen::Vector2d A1 = Eigen::Vector2d(tmp_Silhouette[j][0][0],tmp_Silhouette[j][0][1]);
				Eigen::Vector2d B1 = Eigen::Vector2d(tmp_Silhouette[j][1][0],tmp_Silhouette[j][1][1]);
				Eigen::Vector2d C1 = Eigen::Vector2d(tmp_Silhouette[j][2][0],tmp_Silhouette[j][2][1]);

				Eigen::Vector2d A2 = Eigen::Vector2d(tmp_Silhouette[k][0][0],tmp_Silhouette[k][0][1]);
				Eigen::Vector2d B2 = Eigen::Vector2d(tmp_Silhouette[k][1][0],tmp_Silhouette[k][1][1]);
				Eigen::Vector2d C2 = Eigen::Vector2d(tmp_Silhouette[k][2][0],tmp_Silhouette[k][2][1]);
				if (collisionTris(A1,B1,C1,A2,B2,C2))
				{
					double d1 = tmp_Silhouette[j][0][2] + tmp_Silhouette[j][1][2] + tmp_Silhouette[j][2][2];
					double d2 = tmp_Silhouette[k][0][2] + tmp_Silhouette[k][1][2] + tmp_Silhouette[k][2][2];
					if (d1 < d2)
					{
						Silhouettes_visible[i][j] = false;
						break;
					}
				}
			}*/
		}
		Silhouettes_length[i][0] = maxX - minX;
		Silhouettes_length[i][1] = maxY - minY;
		Silhouettes_center[i][0] = (maxX + minX)/2;
		Silhouettes_center[i][1] = (maxY + minY)/2;

		scale[i] = Silhouettes_length[i][0]/h > Silhouettes_length[i][1]/w ? 1.1*Silhouettes_length[i][0]/h:1.1*Silhouettes_length[i][1]/w;
		offset_x[i] = h/2.0f - Silhouettes_center[i][0]/scale[i];
		offset_y[i] = w/2.0f - Silhouettes_center[i][1]/scale[i];

		for (int j = 0; j < tmp_Silhouette.size(); j++)
		{
			QPoint *tri = new QPoint[3]; 
			for (int k = 0; k < 3; k++)
			{
				int tmpx = ( tmp_Silhouette[j][k][0] )/scale[i] + offset_x[i] ;
				int tmpy = ( tmp_Silhouette[j][k][1] )/scale[i] + offset_y[i] ;
				tri[k].setX(tmpy);
				tri[k].setY(h - tmpx);
			}
			Silhouettes_normalized[i].push_back(tri);
			tmp_Silhouettes_normalized.push_back(tri);
		}
		tmp_scale = scale[i];
		tmp_offset_x = offset_x[i];
		tmp_offset_y = offset_y[i];
		writeSilhoutte(path,i%meshes.size(),i/meshes.size(),true);
	}
	select_point.resize(camera_num*meshes.size());
	select_point_flag.resize(camera_num*meshes.size());
	update();
}

void SilhouettePaint::saveSilhouettes(QString file_dic)
{
	int w = width();
	int h = height();

	for (int i = 0; i < Silhouettes_normalized.size(); i++)
	{
		QPixmap *pix = new QPixmap(w,h);
		QPainter *paint = new QPainter(pix);
		QBrush bk;
		paint->fillRect(0,0,w,h,Qt::white);

		QBrush bruch;
		bruch.setColor(Qt::black);
		bruch.setStyle(Qt::SolidPattern);
		paint->setBrush(bruch);
		paint->setPen(Qt::black);
		for (int j = 0; j < Silhouettes_normalized[i].size(); j++)
		{
			paint->drawPolygon(Silhouettes_normalized[i][j],3);
		}
		paint->end();
		pix->save(file_dic+"/"+QString::number(i)+".png");
		delete pix;
		delete paint;
	}
}