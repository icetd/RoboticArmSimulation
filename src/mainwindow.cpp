#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QMessageBox>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{

    /*Load qss*/
    QFile qss(":Qss/style1.qss");
    qss.open(QFile::ReadOnly);
    qApp->setStyleSheet(qss.readAll());
    qss.close();
    ui->setupUi(this);
    this->setWindowTitle("RobotArmSimulation");

    scene = new RasScene(this);
    this->setCentralWidget(scene);

    fileDialog = new QFileDialog(this);
    fileDialog->setDirectory(QDir::currentPath());


    connect(ui->jointS1, SIGNAL(valueChanged(int)), this, SLOT(jointS1_moved(int)));
    connect(ui->jointS2, SIGNAL(valueChanged(int)), this, SLOT(jointS2_moved(int)));
    connect(ui->jointS3, SIGNAL(valueChanged(int)), this, SLOT(jointS3_moved(int)));
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::on_actionAddRobot_triggered()
{
    fileDialog->setFileMode(QFileDialog::ExistingFile);
    fileDialog->setOption(QFileDialog::DontUseNativeDialog,true);
    fileDialog->setAcceptMode(QFileDialog::AcceptOpen);
    fileDialog->setDirectory(QDir::currentPath() + "/../robots");
    fileDialog->setNameFilter(tr("File(*.xml)"));

    if(fileDialog->exec())
    {
        scene->addRobot(fileDialog->selectedFiles().at(0));
        updateRobotInfo();
        ui->actionAddRobot->setEnabled(false);
    }
}


void MainWindow::on_actionDeleteRobot_triggered()
{
    robot_index--;
    joint_num = 0;
    if (scene->Robots.length())
        scene->deleteRobot(0);
    ui->actionAddRobot->setEnabled(true);
}

void MainWindow::updateRobotInfo()
{
    robot_index++;
    if (scene->Robots.length())
        joint_num = scene->Robots.last()->nLinks - 1;

    if (joint_num >= 1) {
        ui->jointS1->setMaximum(scene->Robots.last()->Links[1]->qMax);
        ui->jointS1->setMinimum(scene->Robots.last()->Links[1]->qMin);
        ui->jointS1->setValue(scene->Robots.last()->Links[1]->theta);
    }

    if (joint_num >= 2) {
		ui->jointS2->setMaximum(scene->Robots.last()->Links[2]->qMax);
		ui->jointS2->setMinimum(scene->Robots.last()->Links[2]->qMin);
        ui->jointS2->setValue(scene->Robots.last()->Links[2]->theta);
	}

    if (joint_num >= 3) {
		ui->jointS3->setMaximum(scene->Robots.last()->Links[3]->qMax);
		ui->jointS3->setMinimum(scene->Robots.last()->Links[3]->qMin);
        ui->jointS3->setValue(scene->Robots.last()->Links[3]->theta);
    }
}

void MainWindow::on_actionResetCamera_triggered()
{
    scene->resetView();
}

void MainWindow::on_actionToggleGrid_triggered()
{
    scene->toggleGrid(ui->actionToggleGrid->isChecked());
}

void MainWindow::on_actionToggleBlend_triggered()
{
    scene->toggleBlend(ui->actionToggleBlend->isChecked());
}

void MainWindow::on_actionToggleGlobalAxis_triggered()
{
    scene->toggleGlobalAxis(ui->actionToggleGlobalAxis->isChecked());
}

void MainWindow::on_actionToggleLocalAxes_triggered()
{
    scene->toggleLocalAxes(ui->actionToggleLocalAxes->isChecked());
}

void MainWindow::on_actionToggleObjectCoordinates_triggered()
{
    scene->toggleObjectCoordinates(ui->actionToggleObjectCoordinates->isChecked());
}

void MainWindow::jointS1_moved(int value)
{
    if (robot_index && joint_num  >= 1) {
		scene->setRobotLinkQ((double)value, 0, 1, true);
    }
}

void MainWindow::jointS2_moved(int value)
{
	if (robot_index && joint_num >= 2)
		scene->setRobotLinkQ((double)value, 0, 2, true);
}

void MainWindow::jointS3_moved(int value)
{
	if (robot_index && joint_num >= 3)
		scene->setRobotLinkQ((double)value, 0, 3, true);
}
