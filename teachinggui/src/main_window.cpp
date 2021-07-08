#include <QtWidgets> 
#include <QMessageBox>
#include <iostream>
#include <vector>
#include "main_window.hpp"
namespace teachinggui
{

using namespace Qt;
/**
 * Construct a new Main Window:: Main Window object
 */
MainWindow::MainWindow(int argc, char** argv, QWidget *parent) : QMainWindow(parent), qnode(argc,argv)
{
	ui.setupUi(this);

	ui.loggingView->setModel(qnode.loggingModel());

	QObject::connect(ui.startButton, SIGNAL(clicked()), this, SLOT(onStartButtonClicked()));
	QObject::connect(ui.savePoseOnButton, SIGNAL(clicked()), this, SLOT(onSavePoseOnButtonClicked()));
	QObject::connect(ui.savePoseOffButton, SIGNAL(clicked()), this, SLOT(onSavePoseOffButtonClicked()));
	QObject::connect(ui.saveToFileButton, SIGNAL(clicked()), this, SLOT(onSaveToFileButtonClicked()));
	QObject::connect(ui.loadAndExecuteButton, SIGNAL(clicked()), this, SLOT(onLoadAndExecuteButtonClicked()));

	ui.savePoseOnButton->setEnabled(false);
	ui.savePoseOffButton->setEnabled(false);
	ui.saveToFileButton->setEnabled(false);
	ui.loadAndExecuteButton->setEnabled(false);

    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
}
/**
 * Destroy the Main Window:: Main Window object
 */
MainWindow::~MainWindow()
{}
/**
 * if save pose button is clicked and init function was already performed saveposefunction is executed
 */
void MainWindow::onSavePoseOnButtonClicked()
{
	if(initClick != false)
	{
		qnode.savePoseFunction(1);
	}
}
/**
 * if save pose button is clicked and init function was already performed saveposefunction is executed
 */
void MainWindow::onSavePoseOffButtonClicked()
{
	if(initClick != false)
	{
		qnode.savePoseFunction(0);
	}
}
/**
 * if save to file button is clicked and init function was already performed savetofilefunction is executed
 */
void MainWindow::onSaveToFileButtonClicked()
{
	if(initClick != false)
	{
		QString textFileName = ui.txtFileInput->toPlainText();
		qnode.saveToFileFunction(textFileName.toStdString());
	}
}
/**
 * if start button is clicked and init function was already performed qnode.init function is executed
 */
void MainWindow::onStartButtonClicked()
{
	if (initClick == false)
	{	
		ui.startButton->setEnabled(false);
		ui.savePoseOnButton->setEnabled(true);
		ui.savePoseOffButton->setEnabled(true);
		ui.saveToFileButton->setEnabled(true);
		ui.loadAndExecuteButton->setEnabled(true);

		qnode.init();

		initClick = true;
	}
}
/**
 * @brief 
 * 
 */
void MainWindow::onLoadAndExecuteButtonClicked()
{
	if(initClick != false)
	{
		ui.saveToFileButton->setEnabled(false);

		QString textFileName = ui.txtFileInput->toPlainText();
		qnode.startRoutine(textFileName.toStdString());
	}
}
/**
 * @brief 
 * 
 * @param event 
 */
void MainWindow::closeEvent(QCloseEvent *event)
{
	QMainWindow::closeEvent(event);
}

}  // namespace teachinggui

