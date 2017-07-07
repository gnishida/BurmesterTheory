#include "MainWindow.h"
#include <QFileDialog>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
	ui.setupUi(this);
	ui.actionShowCenterPointCurve->setChecked(false);
	ui.actionShowCirclePointCurve->setChecked(true);

	connect(ui.actionOpen, SIGNAL(triggered()), this, SLOT(onOpen()));
	connect(ui.actionExit, SIGNAL(triggered()), this, SLOT(close()));
	connect(ui.actionRun, SIGNAL(triggered()), this, SLOT(onRun()));
	connect(ui.actionStop, SIGNAL(triggered()), this, SLOT(onStop()));
	connect(ui.actionStepForward, SIGNAL(triggered()), this, SLOT(onStepForward()));
	connect(ui.actionStepBackward, SIGNAL(triggered()), this, SLOT(onStepBackward()));
	connect(ui.actionShowCenterPointCurve, SIGNAL(triggered()), this, SLOT(onShowCurveChanged()));
	connect(ui.actionShowCirclePointCurve, SIGNAL(triggered()), this, SLOT(onShowCurveChanged()));

	setCentralWidget(&canvas);
}

MainWindow::~MainWindow() {
}

void MainWindow::onOpen() {
	QString filename = QFileDialog::getOpenFileName(this, tr("Open Pose file..."), "", tr("Pose Files (*.xml)"));
	if (filename.isEmpty()) return;

	canvas.open(filename);
}

void MainWindow::onRun() {
	canvas.run();
}

void MainWindow::onStop() {
	canvas.stop();
}

void MainWindow::onStepForward() {
	canvas.stepForward();
}

void MainWindow::onStepBackward() {
	canvas.stepBackward();
}

void MainWindow::onShowCurveChanged() {
	canvas.showCenterPointCurve = ui.actionShowCenterPointCurve->isChecked();
	canvas.showCirclePointCurve = ui.actionShowCirclePointCurve->isChecked();
	update();
}

void MainWindow::keyPressEvent(QKeyEvent* e) {
	canvas.keyPressEvent(e);
}

void MainWindow::keyReleaseEvent(QKeyEvent* e) {
	canvas.keyReleaseEvent(e);
}

