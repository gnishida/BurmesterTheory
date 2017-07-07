#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtWidgets/QMainWindow>
#include "ui_MainWindow.h"
#include "Canvas.h"

class MainWindow : public QMainWindow {
	Q_OBJECT

private:
	Ui::MainWindowClass ui;
	Canvas canvas;

public:
	MainWindow(QWidget *parent = 0);
	~MainWindow();

public slots:
	void onOpen();
	void onRun();
	void onStop();
	void onStepForward();
	void onStepBackward();
	void onShowCurveChanged();
	void keyPressEvent(QKeyEvent* e);
	void keyReleaseEvent(QKeyEvent* e);
};

#endif // MAINWINDOW_H
