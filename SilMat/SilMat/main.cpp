#include "silmat.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	SilMat w;
	w.show();
	return a.exec();
}
