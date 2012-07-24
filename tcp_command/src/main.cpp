#include "Server.h"
#include <QCoreApplication>

int main(int argc, char * argv[]){
	
	QCoreApplication app(argc, argv);

    Server server(argc, argv);

	//for(;;)
	//	;
	//system("pause");
	return app.exec();
}
