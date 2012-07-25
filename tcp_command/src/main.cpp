#include "Server.h"
#include <QCoreApplication>

using namespace server;

int main(int argc, char** argv){
	
	QCoreApplication app(argc, argv);

    Server s(argc, argv);

	//for(;;)
	//	;
	//system("pause");
	return app.exec();
}
