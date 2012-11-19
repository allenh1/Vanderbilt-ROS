#include "ControlWindow.h"
#include <QApplication>

using namespace server;

int main(int argc, char** argv){
	
    QApplication app(argc, argv);

    ControlWindow s(argc, argv);

	//for(;;)
	//	;
	//system("pause");
	return app.exec();
}

