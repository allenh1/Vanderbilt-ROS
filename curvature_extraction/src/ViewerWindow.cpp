#include "ViewerWindow.h"
#include <QPalette>

namespace data_server{
ViewerWindow::ViewerWindow(int argc, char **argv, QWidget *parent)
    : QWidget(parent),
      m_RobotThread(argc, argv)
{
    /** Set up the Controls **/
    p_closeButton = new QPushButton(tr("&Quit"));
    p_saveButton = new QPushButton(tr("&Save"));

    /** Set up the Position Display **/


    mainLayout = new QVBoxLayout();
    mainLayout->addWidget(p_saveButton);
    mainLayout->addWidget(p_closeButton);
    setLayout(mainLayout);

    setWindowTitle(tr("Data Window"));

    connect(p_closeButton, SIGNAL(clicked()), this, SLOT(close()));
    //connect(p_saveButton, SIGNAL(clicked()), &m_RobotThread, SLOT(saveData()));
    m_RobotThread.init();
    m_RobotThread.start();
}//end constructor

}//namespace server
