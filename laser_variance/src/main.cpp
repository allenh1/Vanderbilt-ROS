#include "ViewerWindow.h"
#include <QApplication>

using namespace data_server;

int main(int argc, char** argv){

    QApplication app(argc, argv);

    ViewerWindow s(argc, argv);
    s.show();
    //for(;;)
    //	;
    //system("pause");
    return app.exec();
}
