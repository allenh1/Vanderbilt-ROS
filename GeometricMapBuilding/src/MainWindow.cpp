#include <QtGui>
#include "MainWindow.h"
namespace server
{
using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
    , robot(argc,argv)
    , m_pRefreshTimer(new QTimer(this))

{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(&robot, SIGNAL(rosShutdown()), this, SLOT(close()));
    ui.F01->setText(QString("test"));

    connect(&robot, SIGNAL(NewPoint()), this,    SLOT(updateDisplay()));
    connect(this, SIGNAL(unlockPoint()), &robot, SLOT(unlock()));
    robot.init();
    robot.start();

    m_pRefreshTimer->setSingleShot(false);
    m_pRefreshTimer->setInterval(5);
    //connect(m_pRefreshTimer,SIGNAL(timeout()), this,SLOT(updateDisplay()));
    m_pRefreshTimer->start();

}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::closeEvent(QCloseEvent *event)
{
	QMainWindow::closeEvent(event);
}

void MainWindow::updateDisplay() {
    robot.LockMutex();
    Matrix2x3d F; Matrix3x3d Cxr;
    Matrix2x2d G; Matrix2x2d Cp;

    F(0, 0) = robot.getFData(0, 0); F(0, 1) = robot.getFData(0, 1); F(0, 2) = robot.getFData(0, 2);
    F(1, 0) = robot.getFData(1, 0); F(1, 1) = robot.getFData(1, 1); F(1, 2) = robot.getFData(1, 2);

    G(0, 0) = robot.getGData(0, 0); G(0, 1) = robot.getGData(0, 1);
    G(1, 0) = robot.getGData(1, 0); G(1, 1) = robot.getGData(1, 1);

    Cxr(0, 0) = robot.getCxrData(0, 0); Cxr(0, 1) = robot.getCxrData(0, 1); Cxr(0, 2) = robot.getCxrData(0, 2);
    Cxr(1, 0) = robot.getCxrData(1, 0); Cxr(1, 1) = robot.getCxrData(1, 1); Cxr(1, 2) = robot.getCxrData(1, 2);
    Cxr(2, 0) = robot.getCxrData(2, 0); Cxr(2, 1) = robot.getCxrData(2, 1); Cxr(2, 2) = robot.getCxrData(2, 2);

    Cp(0, 0) = robot.getCpData(0, 0); Cp(0, 1) = robot.getCpData(0, 1);
    Cp(1, 0) = robot.getCpData(1, 0); Cp(1, 1) = robot.getCpData(1, 1);

    QString F00; QString F01; QString F02;
    QString F10; QString F11; QString F12;

    QString G00; QString G01;
    QString G10; QString G11;

    QString Cxr00; QString Cxr01; QString Cxr02;
    QString Cxr10; QString Cxr11; QString Cxr12;
    QString Cxr20; QString Cxr21; QString Cxr22;

    QString Cp00; QString Cp01;
    QString Cp10; QString Cp11;

    F00.setNum(F(0, 0)); F01.setNum(F(0, 1)); F02.setNum(F(0, 2));
    F10.setNum(F(1, 0)); F11.setNum(F(1, 1)); F12.setNum(F(1, 2));

    G00.setNum(G(0, 0)); G01.setNum(G(0, 1));
    G10.setNum(G(1, 0)); G11.setNum(G(1, 1));

    Cxr00.setNum(Cxr(0, 0)); Cxr01.setNum(Cxr(0, 1)); Cxr02.setNum(Cxr(0, 2));
    Cxr10.setNum(Cxr(1, 0)); Cxr11.setNum(Cxr(1, 1)); Cxr12.setNum(Cxr(1, 2));
    Cxr20.setNum(Cxr(2, 0)); Cxr21.setNum(Cxr(2, 1)); Cxr22.setNum(Cxr(2, 2));

    Cp00.setNum(Cp(0, 0)); Cp01.setNum(Cp(0, 1));
    Cp10.setNum(Cp(1, 0)); Cp11.setNum(Cp(1, 1));

    ui.F00->setText(F00); ui.F01->setText(F01); ui.F02->setText(F02);
    ui.F10->setText(F10); ui.F11->setText(F11); ui.F12->setText(F12);

    ui.G00->setText(G00); ui.G01->setText(G01);
    ui.G10->setText(G10); ui.G11->setText(G11);

    ui.Cxr00->setText(Cxr00); ui.Cxr01->setText(Cxr01); ui.Cxr02->setText(Cxr02);
    ui.Cxr10->setText(Cxr10); ui.Cxr11->setText(Cxr11); ui.Cxr12->setText(Cxr12);
    ui.Cxr20->setText(Cxr20); ui.Cxr21->setText(Cxr21); ui.Cxr22->setText(Cxr22);

    ui.Cp00->setText(Cp00); ui.Cp01->setText(Cp01);
    ui.Cp10->setText(Cp10); ui.Cp11->setText(Cp11);
    //Q_EMIT unlockPoint();
    robot.UnlockMutex();
}//update the GUI Matrix.
}//end namespace
