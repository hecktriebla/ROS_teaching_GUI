#ifndef teachinggui_MAIN_WINDOW_H
#define teachinggui_MAIN_WINDOW_H

#include <QtWidgets/QMainWindow>
#include <QtWidgets> 
#include <QMessageBox>
#include <QModelIndex>
#include "qnode.hpp"
#include <vector>
#include <iostream>
#include "ui_main_window.h"

namespace teachinggui
{

class MainWindow : public QMainWindow 
{
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();
	void closeEvent(QCloseEvent *event); // Overloaded function
public Q_SLOTS:
	void onStartButtonClicked();
	void onSavePoseOnButtonClicked();
	void onSavePoseOffButtonClicked();
	void onSaveToFileButtonClicked();
	void onLoadAndExecuteButtonClicked();
private:
	Ui::MainWindowDesign ui;
	QNode qnode;
	bool initClick = false;
};

}  // namespace teachinggui

#endif // multicontroller_MAIN_WINDOW_H

