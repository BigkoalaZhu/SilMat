#include "silmat.h"
#include <QFileDialog>

SilMat::SilMat(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
}

SilMat::~SilMat()
{

}

void SilMat::LoadAgentMesh()
{
	QFileDialog dialog(this);
	dialog.setDirectory(QDir::currentPath());
	dialog.setFileMode(QFileDialog::ExistingFile);
	dialog.setNameFilter(tr("List File (*.list)"));

	if (dialog.exec())
	{
		QString filename = dialog.selectedFiles()[0];
//		ui.Silhouette->loadMeshList(filename);
		filelist.push_back(filename);
	}
}

void SilMat::LoadSilhoueeteA()
{
	QFileDialog dialog(this);
	dialog.setDirectory(QDir::currentPath());
	dialog.setFileMode(QFileDialog::ExistingFile);
	dialog.setNameFilter(tr("Png File (*.png)"));

	if (dialog.exec())
	{
		QString filename = dialog.selectedFiles()[0];
		ui.InputSilhouetteA->LoadSilhouetteImage(filename);
	}
}

void SilMat::LoadSilhoueeteB()
{
	QFileDialog dialog(this);
	dialog.setDirectory(QDir::currentPath());
	dialog.setFileMode(QFileDialog::ExistingFile);
	dialog.setNameFilter(tr("Png File (*.png)"));

	if (dialog.exec())
	{
		QString filename = dialog.selectedFiles()[0];
		ui.InputSilhouetteB->LoadSilhouetteImage(filename);
	}
}

void SilMat::FindBestView()
{
	
	QFileDialog dialog(this);
	dialog.setDirectory(QDir::currentPath());
	dialog.setFileMode(QFileDialog::Directory);

	if (dialog.exec())
	{
		QString filename = dialog.selectedFiles()[0];
		ui.InputSilhouetteA->findBestView(filename);
		ui.Silhouette->SetCameraSetting(ui.InputSilhouetteA->GetCameraSetting());
		ui.Silhouette->SetMeshPath(ui.InputSilhouetteA->GetMeshPath());
		ui.Silhouette->CalcProjectedSilhouette();
	}
}

void SilMat::FindBestView_2()
{
	
	QFileDialog dialog(this);
	dialog.setDirectory(QDir::currentPath());
	dialog.setFileMode(QFileDialog::Directory);

	if (dialog.exec())
	{
		QString filename = dialog.selectedFiles()[0];
		ui.InputSilhouetteB->findBestView(filename);
		ui.Silhouette_2->SetCameraSetting(ui.InputSilhouetteB->GetCameraSetting());
		ui.Silhouette_2->SetMeshPath(ui.InputSilhouetteB->GetMeshPath());
		ui.Silhouette_2->CalcProjectedSilhouette();
	}
}

void SilMat::SaveSilhouettes()
{
	QFileDialog dialog(this);
	dialog.setDirectory("silhouettes");
	dialog.setFileMode(QFileDialog::Directory);

	if (dialog.exec())
	{
		QString filename = dialog.directory().absolutePath();
//		ui.Silhouette->saveSilhouettes(filename);
	}
}

void SilMat::mesh_num_set(int num)
{
	ui.mesh_num->setText(QString::number(num));
}

void SilMat::DisplayModel()
{
/*	MeshGL *modelDisplay = new MeshGL(this);
	int modelIndex = ui.model_index->text().toInt() - 1;
	modelDisplay->LoadMesh(ui.Silhouette->GetMesh(modelIndex));
	modelDisplay->show();
/*	QFile file("label.txt");
	std::ofstream ofs("bash.txt");
	if(!file.open(QIODevice::ReadOnly | QIODevice::Text)) {  
        qDebug()<<"Can't open the file!"<<endl;  
    }

//	QByteArray line = file.readLine();
//	line = file.readLine();
//	line = file.readLine();
    while(!file.atEnd()) {  
        QByteArray line = file.readLine();  
        QString str(line);
		if(str == "Elephant 0 24\n")
		{
			QVector<int> indexes;
			for (int i = 0; i < 13; i++)
			{
				line = file.readLine();
				QString tmp(line);
				indexes.push_back(tmp.toInt());
			}
			for (int i = 0; i < 13; i++)
			{
				for (int j = 0; j < 13; j++)
				{
					if(i==j)
						continue;
					QString tmp = "./BlendedIntrinsicMaps ../T" + QString::number(indexes[i]) + ".off ../T" + QString::number(indexes[j]) + ".off\n";
					ofs << tmp.toLocal8Bit().constData();
				}
			}
		}
    }*/
}
