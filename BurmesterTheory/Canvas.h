#ifndef CANVAS_H
#define CANVAS_H

#include <QWidget>
#include <QKeyEvent>
#include <glm/glm.hpp>
#include <boost/shared_ptr.hpp>
#include <kinematics.h>
#include <QTimer>

class Canvas : public QWidget {
Q_OBJECT

public:
	bool ctrlPressed;
	bool shiftPressed;

	kinematics::Kinematics kinematics;
	QTimer* animation_timer;
	bool collision_check;
	QPoint prev_mouse_pt;
	QPoint origin;
	double scale;
	std::vector<std::vector<std::vector<glm::dvec2>>> solutions;
	std::vector<std::vector<std::vector<glm::dvec2>>> poles;
	std::vector<std::vector<std::vector<kinematics::SpecialPoint>>> pole_intersections;
	std::vector<std::vector<kinematics::SpecialPoint>> UTs;
	std::vector<glm::dvec2> Bs;
	std::vector<std::vector<std::tuple<int, int, int>>> extreme_poses;
	double alpha;
	std::pair<int, int> selectedSolution;
	boost::shared_ptr<kinematics::Joint> selectedJoint;
	std::vector<std::vector<glm::dvec2>> body_pts;
	std::vector<glm::dmat4x4> poses;
	int linkage_type;
	bool grashofDefect;
	bool orderDefect;
	bool branchDefect;
	bool showCenterPointCurve;
	bool showCirclePointCurve;

public:
	Canvas(QWidget *parent = NULL);
    ~Canvas();

	void open(const QString& filename);
	void run();
	void stop();
	void speedUp();
	void speedDown();
	void invertSpeed();
	void stepForward();
	void stepBackward();
	void showAssemblies(bool flag);
	void showLinks(bool flag);
	void showBodies(bool flag);

	std::pair<int, int> findSolution(bool center_point_curve, const glm::dvec2& pt);
	int getGrashofType();
	bool checkGrashofDefect();
	bool checkOrderDefect();
	bool checkBranchDefect();

public slots:
	void animation_update();

protected:
	void paintEvent(QPaintEvent* e);
	void mousePressEvent(QMouseEvent* e);
	void mouseMoveEvent(QMouseEvent* e);
	void mouseReleaseEvent(QMouseEvent* e);
	void mouseDoubleClickEvent(QMouseEvent* e);
	void wheelEvent(QWheelEvent* e);
	void resizeEvent(QResizeEvent *e);

public:
	void keyPressEvent(QKeyEvent* e);
	void keyReleaseEvent(QKeyEvent* e);
};

#endif // CANVAS_H
