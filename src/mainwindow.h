#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFileDialog>

#include "RasScene.h"
#include "RasGeneric.h"

const QString alphaSymbol = QString::fromWCharArray(L"\u03B1");
const QString thetaSymbol = QString::fromWCharArray(L"\u03B8");
const QString derivSymbol = QString::fromWCharArray(L"\u2202");
const QString tauSymbol   = QString::fromWCharArray(L"\u03C4");

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    
    // File dialogs variables
    QFileDialog *fileDialog;
    QStringList filterOBJ;
    QStringList filterXML;
    QStringList filterCSV;

public slots:
    void updateRobotInfo();

private slots:
    void on_actionAddRobot_triggered();
    void on_actionDeleteRobot_triggered();

    void on_actionToggleGrid_triggered();
    void on_actionToggleObjectCoordinates_triggered();
    void on_actionToggleLocalAxes_triggered();
    void on_actionToggleGlobalAxis_triggered();
    void on_actionToggleBlend_triggered();
    void on_actionResetCamera_triggered();

    void jointS1_moved(int value);
    void jointS2_moved(int value);
    void jointS3_moved(int value);


private:
    Ui::MainWindow *ui;

    RasScene *scene;

    bool flagObstacleTree;
    bool flagRobotTree;
    bool flagPathTree;

    int robot_index = 0;
    int joint_num = 0;
};

#endif // MAINWINDOW_H
