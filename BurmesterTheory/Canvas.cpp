#include "Canvas.h"
#include <QPainter>
#include <iostream>
#include <QFileInfoList>
#include <QDir>
#include <QMessageBox>
#include <QTextStream>
#include <QDomDocument>
#include <QResizeEvent>
#include <QtWidgets/QApplication>
#include <glm/gtx/string_cast.hpp>

Canvas::Canvas(QWidget *parent) : QWidget(parent) {
	ctrlPressed = false;
	shiftPressed = false;

	origin = QPoint(0, height());
	scale = 10.0;
	animation_timer = NULL;
	simulation_speed = 0.01;

	linkage_type = -1;
	grashofDefect = false;
	orderDefect = false;
	branchDefect = false;

	showCenterPointCurve = false;
	showCirclePointCurve = true;
	
	// read solution curve
	/*
	QFile file("solution_curve_ex1.txt");
	file.open(QIODevice::ReadOnly);
	QTextStream in(&file);
	while (!in.atEnd()) {
		QStringList list = in.readLine().split(",");
		glm::dvec2 p1(list[0].toDouble(), list[1].toDouble());
		glm::dvec2 p2(list[2].toDouble(), list[3].toDouble());
		solutions.push_back({ p1, p2 });
	}
	*/
	selectedSolution = { -1, -1 };

	// poses
	/*
	poses.resize(4);
	poses[0] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 1, 0, 1 };
	poses[1] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 2, 0.5, 0, 1 };
	poses[2] = { 0.707107, 0.707107, 0, 0, -0.707107, 0.707107, 0, 0, 0, 0, 1, 0, 3, 1.5, 0, 1 };
	poses[3] = { 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1, 0, 2, 2, 0, 1 };
	poles = kinematics::calculatePoles(poses);
	*/
	/*
	poses[0] = { 0.37955, 0.92516, 0, 0, -0.92516, 0.37955, 0, 0, 0, 0, 1, 0, 4.2, 1.2, 0, 1 };
	poses[1] = { 0.94385, 0.33035, 0, 0, -0.33035, 0.94385, 0, 0, 0, 0, 1, 0, 1.52, 1.02, 0, 1 };
	poses[2] = { 0.667119, -0.74495, 0, 0, 0.74495, 0.667119, 0, 0, 0, 0, 1, 0, 0.6, 3.52, 0, 1 };
	poses[3] = { 0.97648, -0.21558, 0, 0, 0.21558, 0.97648, 0, 0, 0, 0, 1, 0, 0.63, 1.3, 0, 1 };

	poles = kinematics::calculatePoles(poses);
	*/
	alpha = 0.3;

	// body geometry
	body_pts.resize(poses.size());
	for (int i = 0; i < poses.size(); i++) {
		body_pts[i].push_back(glm::dvec2(poses[i] * glm::dvec4(0, 0, 0, 1)));
		body_pts[i].push_back(glm::dvec2(poses[i] * glm::dvec4(1, 0, 0, 1)));
		body_pts[i].push_back(glm::dvec2(poses[i] * glm::dvec4(1, 0.3, 0, 1)));
		body_pts[i].push_back(glm::dvec2(poses[i] * glm::dvec4(0, 0.3, 0, 1)));
	}

	// initialize kinematics structure
	kinematics.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(0, true, glm::dvec2(0, 0))));
	kinematics.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(1, true, glm::dvec2(2, 0))));
	kinematics.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(2, false, glm::dvec2(0, 2))));
	kinematics.diagram.addJoint(boost::shared_ptr<kinematics::PinJoint>(new kinematics::PinJoint(3, false, glm::dvec2(2, 2))));
	kinematics.diagram.addLink(true, kinematics.diagram.joints[0], kinematics.diagram.joints[2]);
	kinematics.diagram.addLink(false, kinematics.diagram.joints[1], kinematics.diagram.joints[3]);
	kinematics.diagram.addLink(false, kinematics.diagram.joints[2], kinematics.diagram.joints[3]);

	// update the geometry
	//kinematics.diagram.addBody(kinematics.diagram.joints[2], kinematics.diagram.joints[3], body_pts[0]);

	// setup the kinematic system
	kinematics.diagram.initialize();
}

Canvas::~Canvas() {
}

void Canvas::open(const QString& filename) {
	QFile file(filename);
	if(!file.open(QFile::ReadOnly | QFile::Text)) throw "File cannot open.";

	QDomDocument doc;
	doc.setContent(&file);
	QDomElement root = doc.documentElement();
	if (root.tagName() != "poses")	throw "Invalid file format.";

	poses.clear();

	QDomNode node = root.firstChild();
	while (!node.isNull()) {
		if (node.toElement().tagName() == "pose") {
			std::vector<glm::dvec2> pts;
			QDomNode point_node = node.firstChild();
			while (!point_node.isNull()) {
				if (point_node.toElement().tagName() == "point") {
					double x = point_node.toElement().attribute("x").toDouble();
					double y = point_node.toElement().attribute("y").toDouble();
					pts.push_back(glm::dvec2(x, y));
				}
				point_node = point_node.nextSibling();
			}

			if (pts.size() == 2) {
				double theta = atan2(pts[1].y - pts[0].y, pts[1].x - pts[0].x);
				poses.push_back({ cos(theta), sin(theta), 0, 0, -sin(theta), cos(theta), 0, 0, 0, 0, 1, 0, pts[0].x, pts[0].y, 0, 1 });
			}
		}

		node = node.nextSibling();
	}

	if (poses.size() != 4) {
		throw "Invalid number of poses was specified.";
	}

	// body geometry
	body_pts.clear();
	body_pts.resize(poses.size());
	for (int i = 0; i < poses.size(); i++) {
		body_pts[i].push_back(glm::dvec2(poses[i] * glm::dvec4(0, -0.25, 0, 1)));
		body_pts[i].push_back(glm::dvec2(poses[i] * glm::dvec4(0.843, -0.25, 0, 1)));
		body_pts[i].push_back(glm::dvec2(poses[i] * glm::dvec4(0.843, 0.25, 0, 1)));
		body_pts[i].push_back(glm::dvec2(poses[i] * glm::dvec4(0, 0.25, 0, 1)));
	}

	// calculate the circle point curve and center point curve
	kinematics::calculateSolutionCurve(poses, solutions);

	poles = kinematics::calculatePoles(poses);
	pole_intersections = kinematics::calculatePoleIntersections(poses, solutions);

	UTs = kinematics::calculateUTs(solutions[1], poles);
	Bs.clear();
	for (int i = 0; i < solutions[1].size(); i++) {
		Bs.push_back(solutions[1][i][0]);
	}

	extreme_poses = kinematics::findExtremePoses(poses, solutions[1], poles[1], pole_intersections[1], UTs);




	std::vector<std::vector<glm::dvec2>> best_solution = kinematics::findValidSolution(poses, solutions);
	kinematics.diagram.joints[0]->pos = best_solution[0][0];
	kinematics.diagram.joints[1]->pos = best_solution[0][1];
	kinematics.diagram.joints[2]->pos = best_solution[1][0];
	kinematics.diagram.joints[3]->pos = best_solution[1][1];

	// update the geometry
	kinematics.diagram.bodies.clear();
	kinematics.diagram.addBody(kinematics.diagram.joints[2], kinematics.diagram.joints[3], body_pts[0]);

	// setup the kinematic system
	kinematics.diagram.initialize();
	update();

	grashofDefect = checkGrashofDefect();
	orderDefect = checkOrderDefect();
	branchDefect = checkBranchDefect();


	update();
}

void Canvas::run() {
	if (animation_timer == NULL) {
		animation_timer = new QTimer(this);
		connect(animation_timer, SIGNAL(timeout()), this, SLOT(animation_update()));
		animation_timer->start(10);
	}
}

void Canvas::stop() {
	if (animation_timer != NULL) {
		animation_timer->stop();
		delete animation_timer;
		animation_timer = NULL;
	}
}

void Canvas::speedUp() {
	simulation_speed *= 2;
}

void Canvas::speedDown() {
	simulation_speed *= 0.5;
}

void Canvas::invertSpeed() {
	simulation_speed = -simulation_speed;
}

void Canvas::stepForward() {
	if (animation_timer == NULL) {
		try {
			kinematics.stepForward(simulation_speed);
		}
		catch (char* ex) {
			simulation_speed = -simulation_speed;
			std::cerr << "Animation is stopped by error:" << std::endl;
			std::cerr << ex << std::endl;
		}
		update();
	}
}

void Canvas::stepBackward() {
	if (animation_timer == NULL) {
		try {
			kinematics.stepForward(-simulation_speed);
		}
		catch (char* ex) {
			simulation_speed = -simulation_speed;
			std::cerr << "Animation is stopped by error:" << std::endl;
			std::cerr << ex << std::endl;
		}
		update();
	}
}

void Canvas::showAssemblies(bool flag) {
	kinematics.showAssemblies(flag);
	update();
}

void Canvas::showLinks(bool flag) {
	kinematics.showLinks(flag);
	update();
}

void Canvas::showBodies(bool flag) {
	kinematics.showBodies(flag);
	update();
}

std::pair<int, int> Canvas::findSolution(bool center_point_curve, const glm::dvec2& pt) {
	int offset = 1;
	if (center_point_curve) {
		offset = 0;
	}

	std::pair<int, int> ans = { -1, -1 };
	double min_dist = std::numeric_limits<double>::max();
	for (int i = 0; i < solutions[offset].size(); i++) {
		for (int j = 0; j < solutions[offset][i].size(); j++) {
			double dist = glm::length(solutions[offset][i][j] - pt);
			if (dist < min_dist) {
				min_dist = dist;
				ans = { i, j };
			}
		}
	}

	return ans;
}

/**
 * Return the Grashof type.
 *    
 * 0 -- Grashof (Drag-link)
 * 1 -- Grashof (Crank-rocker)
 * 2 -- Grashof (Rocker-crank)
 * 3 -- Grashof (Double-rocker)
 * 4 -- Non-Grashof (0-0 Rocker)
 * 5 -- Non-Grashof (pi-pi Rocker)
 * 6 -- Non-Grashof (pi-0 Rocker)
 * 7 -- Non-Grashof (0-pi Rocker)
 */
int Canvas::getGrashofType() {
	double g = glm::length(kinematics.diagram.joints[0]->pos - kinematics.diagram.joints[1]->pos);
	double a = glm::length(kinematics.diagram.joints[0]->pos - kinematics.diagram.joints[2]->pos);
	double b = glm::length(kinematics.diagram.joints[1]->pos - kinematics.diagram.joints[3]->pos);
	double h = glm::length(kinematics.diagram.joints[2]->pos - kinematics.diagram.joints[3]->pos);

	double T1 = g + h - a - b;
	double T2 = b + g - a - h;
	double T3 = b + h - a - g;

	if (T1 < 0 && T2 < 0 && T3 >= 0) {
		return 0;
	}
	else if (T1 >= 0 && T2 >= 0 && T3 >= 0) {
		return 1;
	}
	else if (T1 >= 0 && T2 < 0 && T3 < 0) {
		return 2;
	}
	else if (T1 < 0 && T2 >= 0 && T3 < 0) {
		return 3;
	}
	else if (T1 < 0 && T2 < 0 && T3 < 0) {
		return 4;
	}
	else if (T1 < 0 && T2 >= 0 && T3 >= 0) {
		return 5;
	}
	else if (T1 >= 0 && T2 < 0 && T3 >= 0) {
		return 6;
	}
	else if (T1 >= 0 && T2 >= 0 && T3 < 0) {
		return 7;
	}
	else {
		return -1;
	}
}

/**
 * Check if the linkage has Grashof defect.
 * If the following conditions are not satisified, the linkage has Grashof defect, and true is returned.
 * - The sum of the shortest and longest link is less than the sum of the remaining links (i.e., s + l <= p + q).
 * - The shortest link is either a driving link or a ground link.
 * If both conditions are satisfied, there is no Grashof defect, and false is returned.
 */
bool Canvas::checkGrashofDefect() {
	linkage_type = getGrashofType();
	
	if (linkage_type == 0 || linkage_type == 1) {
		return false;
	}
	else {
		return true;
	}
}

/**
 * Check if the linkage has order defect.
 * If there is an order defect, true is returned.
 * Otherwise, false is returned.
 */
bool Canvas::checkOrderDefect() {
	linkage_type = getGrashofType();

	glm::dvec2 inv_W = glm::dvec2(glm::inverse(poses[0]) * glm::dvec4(kinematics.diagram.joints[2]->pos, 0, 1));

	double total_cw = 0;
	double total_ccw = 0;
	double prev = 0;
	//int ccw = 1;
	for (int i = 0; i < poses.size(); i++) {
		// calculate the coordinates of the circle point of the driving crank in the world coordinate system
		glm::dvec2 X = glm::dvec2(poses[i] * glm::dvec4(inv_W, 0, 1));
		//std::cout << X.x << "," << X.y << std::endl;

		// calculate the direction from the ground pivot (center point) of the driving crank to the circle point
		glm::dvec2 dir = X - kinematics.diagram.joints[0]->pos;

		// calculate its angle
		double theta = atan2(dir.y, dir.x);

		if (i >= 1) {
			if (theta >= prev) {
				total_cw += kinematics::M_PI * 2 - theta + prev;
				total_ccw += theta - prev;
			}
			else {
				total_cw += prev - theta;
				total_ccw += kinematics::M_PI * 2 - prev + theta;
			}
		}

		prev = theta;
	}

	if (total_cw > kinematics::M_PI * 2 + 0.1 && total_ccw > kinematics::M_PI * 2 + 0.1) return true;
	else return false;
}

/**
* Check if all the poses are in the same branch.
* If there is an branch defect, true is returned.
* Otherwise, false is returned.
*/
bool Canvas::checkBranchDefect() {
	int type = getGrashofType();
	
	if (type == 0) {	// Grashof (Drag-link)
		int sign1 = 1;
		int sign2 = 1;

		glm::dvec2 inv_W1 = glm::dvec2(glm::inverse(poses[0]) * glm::dvec4(kinematics.diagram.joints[2]->pos, 0, 1));
		glm::dvec2 inv_W2 = glm::dvec2(glm::inverse(poses[0]) * glm::dvec4(kinematics.diagram.joints[3]->pos, 0, 1));

		for (int i = 0; i < poses.size(); i++) {
			// calculate the coordinates of the circle point of the driving/driven cranks in the world coordinate system
			glm::dvec2 X1 = glm::dvec2(poses[i] * glm::dvec4(inv_W1, 0, 1));
			glm::dvec2 X2 = glm::dvec2(poses[i] * glm::dvec4(inv_W2, 0, 1));

			// calculate the direction from the ground pivot (center point) of the driving crank to the circle point
			glm::dvec2 v1 = X1 - kinematics.diagram.joints[0]->pos;

			// calculate the direction from the ground pivot (center point) of the driven crank to the circle point
			glm::dvec2 v2 = X2 - kinematics.diagram.joints[1]->pos;

			// calculate the direction from the circle point of the driven crank to the circle point of the driving crank
			glm::dvec2 v3 = X1 - X2;

			// calculate its sign
			if (i == 0) {
				sign1 = (v1.x * v3.y - v1.y * v3.x >= 0) ? 1 : -1;
				sign2 = (v2.x * v3.y - v2.y * v3.x >= 0) ? 1 : -1;
			}
			else {
				/*
				if ((v1.x * v3.y - v1.y * v3.x >= 0 ? 1 : -1) != sign1) sign1 = 9999;
				if ((v2.x * v3.y - v2.y * v3.x >= 0 ? 1 : -1) != sign2) sign2 = 9999;
				if (sign1 == 9999 && sign2 == 9999) return true;
				*/
				if ((v1.x * v3.y - v1.y * v3.x >= 0 ? 1 : -1) != sign1) return true;
				if ((v2.x * v3.y - v2.y * v3.x >= 0 ? 1 : -1) != sign2) return true;
			}
		}

		return false;
	}
	else if (type == 1) {	// Grashof (Crank-rocker)
		int sign = 1;

		glm::dvec2 inv_W1 = glm::dvec2(glm::inverse(poses[0]) * glm::dvec4(kinematics.diagram.joints[2]->pos, 0, 1));
		glm::dvec2 inv_W2 = glm::dvec2(glm::inverse(poses[0]) * glm::dvec4(kinematics.diagram.joints[3]->pos, 0, 1));

		for (int i = 0; i < poses.size(); i++) {
			// calculate the coordinates of the circle point of the driving/driven cranks in the world coordinate system
			glm::dvec2 X1 = glm::dvec2(poses[i] * glm::dvec4(inv_W1, 0, 1));
			glm::dvec2 X2 = glm::dvec2(poses[i] * glm::dvec4(inv_W2, 0, 1));

			// calculate the direction from the ground pivot (center point) of the driven crank to the circle point
			glm::dvec2 v1 = X2 - kinematics.diagram.joints[1]->pos;

			// calculate the direction from the circle point of the driven crank to the circle point of the driving crank
			glm::dvec2 v2 = X1 - X2;
			
			// calculate its sign
			if (i == 0) {
				sign = v1.x * v2.y - v1.y * v2.x >= 0 ? 1 : -1;
			}
			else {
				if ((v1.x * v2.y - v1.y * v2.x >= 0 ? 1 : -1) != sign) return true;
			}
		}

		return false;
	}
	else if (type == 2 || type == 3) {	// Grashof (Rocker-crank or Double rocker)
		int sign1 = 1;
		int sign2 = 1;

		glm::dvec2 inv_W1 = glm::dvec2(glm::inverse(poses[0]) * glm::dvec4(kinematics.diagram.joints[2]->pos, 0, 1));
		glm::dvec2 inv_W2 = glm::dvec2(glm::inverse(poses[0]) * glm::dvec4(kinematics.diagram.joints[3]->pos, 0, 1));

		for (int i = 0; i < poses.size(); i++) {
			// calculate the coordinates of the circle point of the driving/driven cranks in the world coordinate system
			glm::dvec2 X1 = glm::dvec2(poses[i] * glm::dvec4(inv_W1, 0, 1));
			glm::dvec2 X2 = glm::dvec2(poses[i] * glm::dvec4(inv_W2, 0, 1));

			// calculate the direction from the ground pivot (center point) of the driving crank to the circle point
			glm::dvec2 v1 = X1 - kinematics.diagram.joints[0]->pos;

			// calculate the direction from the ground pivot (center point) of the driven crank to the circle point
			glm::dvec2 v2 = X2 - kinematics.diagram.joints[1]->pos;

			// calculate the direction from the center point of the driven crank to the center point of the driving crank
			glm::dvec2 v3 = kinematics.diagram.joints[0]->pos - kinematics.diagram.joints[1]->pos;

			// calculate its sign
			if (i == 0) {
				sign1 = (v1.x * v3.y - v1.y * v3.x >= 0) ? 1 : -1;
				sign2 = (v2.x * v3.y - v2.y * v3.x >= 0) ? 1 : -1;
			}
			else {
				if ((v1.x * v3.y - v1.y * v3.x >= 0 ? 1 : -1) != sign1) return true;
				if ((v2.x * v3.y - v2.y * v3.x >= 0 ? 1 : -1) != sign2) return true;
			}
		}

		return false;
	}
	else {	// Non-Grashof
		int sign1 = 1;
		int sign2 = 1;

		glm::dvec2 inv_W1 = glm::dvec2(glm::inverse(poses[0]) * glm::dvec4(kinematics.diagram.joints[2]->pos, 0, 1));
		glm::dvec2 inv_W2 = glm::dvec2(glm::inverse(poses[0]) * glm::dvec4(kinematics.diagram.joints[3]->pos, 0, 1));

		for (int i = 0; i < poses.size(); i++) {
			// calculate the coordinates of the circle point of the driving/driven cranks in the world coordinate system
			glm::dvec2 X1 = glm::dvec2(poses[i] * glm::dvec4(inv_W1, 0, 1));
			glm::dvec2 X2 = glm::dvec2(poses[i] * glm::dvec4(inv_W2, 0, 1));

			// calculate the direction from the ground pivot (center point) of the driving crank to the circle point
			glm::dvec2 v1 = X1 - kinematics.diagram.joints[0]->pos;

			// calculate the direction from the ground pivot (center point) of the driven crank to the circle point
			glm::dvec2 v2 = X2 - kinematics.diagram.joints[1]->pos;

			// calculate the direction from the circle point of the driven crank to the circle point of the driving crank
			glm::dvec2 v3 = X1 - X2;

			// calculate its sign
			if (i == 0) {
				sign1 = (v1.x * v3.y - v1.y * v3.x >= 0) ? 1 : -1;
				sign2 = (v2.x * v3.y - v2.y * v3.x >= 0) ? 1 : -1;
			}
			else {
				if ((v1.x * v3.y - v1.y * v3.x >= 0 ? 1 : -1) != sign1) sign1 = 9999;
				if ((v2.x * v3.y - v2.y * v3.x >= 0 ? 1 : -1) != sign2) sign2 = 9999;
				if (sign1 == 9999 && sign2 == 9999) return true;
			}
		}

		return false;
	}
}

void Canvas::animation_update() {
	try {
		kinematics.stepForward(simulation_speed);
	}
	catch (char* ex) {
		simulation_speed = -simulation_speed;
		//stop();
		std::cerr << "Animation is stopped by error:" << std::endl;
		std::cerr << ex << std::endl;
	}

	update();

}

void Canvas::paintEvent(QPaintEvent *e) {
	QPainter painter(this);

	// draw axes
	painter.save();
	painter.setPen(QPen(QColor(128, 128, 128), 1, Qt::DashLine));
	painter.drawLine(-10000 * scale + origin.x(), origin.y(), 10000 * scale + origin.y(), origin.y());
	painter.drawLine(origin.x(), 10000 * scale + origin.y(), origin.x(), -10000 * scale + origin.y());
	painter.restore();

	if (showCenterPointCurve) {
		// draw center point curve
		if (solutions.size() == 2) {
			for (int i = 0; i < extreme_poses.size(); i++) {
				for (int j = 0; j < extreme_poses[i].size(); j++) {
					int index = std::get<0>(extreme_poses[i][j]);
					int pose1 = std::get<1>(extreme_poses[i][j]);
					int pose2 = std::get<2>(extreme_poses[i][j]);
					if (pose1 == -1 || pose2 == -1) {
						painter.setPen(QPen(QColor(0, 0, 255), 1));
						painter.setBrush(QBrush(QColor(0, 0, 255)));
					}
					else {
						painter.setPen(QPen(QColor(0, 0, 255), 3));
						painter.setBrush(QBrush(QColor(0, 0, 255)));
					}

					int end;
					if (j < extreme_poses[i].size() - 1) {
						end = std::get<0>(extreme_poses[i][j + 1]);
					}
					else {
						end = solutions[1][i].size();
					}

					for (int k = index; k < end; k++) {
						int next = (k + 1) % solutions[0][i].size();
						if (glm::length(solutions[0][i][k] - solutions[0][i][next]) > 20) continue;
						painter.drawLine(origin.x() + solutions[0][i][k].x * scale, origin.y() - solutions[0][i][k].y * scale, origin.x() + solutions[0][i][next].x * scale, origin.y() - solutions[0][i][next].y * scale);
					}
				}
			}
		}

		// draw poles
		if (poles.size() == 2) {
			for (int i = 0; i < poles[0].size(); i++) {
				for (int j = i + 1; j < poles[0][i].size(); j++) {
					painter.setPen(QPen(QColor(0, 0, 0), 1));
					painter.setBrush(QBrush(QColor(0, 0, 0)));
					painter.drawEllipse(QPoint(origin.x() + poles[0][i][j].x * scale, origin.y() - poles[0][i][j].y * scale), 3, 3);

					painter.setPen(QPen(QColor(0, 0, 0), 1));
					QString text = QString("P%1%2").arg(i + 1).arg(j + 1);
					painter.drawText(origin.x() + poles[0][i][j].x * scale + 5, origin.y() - poles[0][i][j].y * scale + 12, text);
				}
			}
		}

		// draw pole intersections
		if (pole_intersections.size() == 2) {
			for (int i = 0; i < pole_intersections[0].size(); i++) {
				for (int j = 0; j < pole_intersections[0][i].size(); j++) {
					painter.setPen(QPen(QColor(0, 0, 0), 1));
					painter.setBrush(QBrush(QColor(0, 255, 255)));
					painter.drawEllipse(QPoint(origin.x() + solutions[0][i][pole_intersections[0][i][j].index].x * scale, origin.y() - solutions[0][i][pole_intersections[0][i][j].index].y * scale), 3, 3);

					painter.setPen(QPen(QColor(0, 0, 0), 1));
					QString text = QString("Q%1%2").arg(pole_intersections[0][i][j].subscript.first + 1).arg(pole_intersections[0][i][j].subscript.second + 1);
					painter.drawText(origin.x() + solutions[0][i][pole_intersections[0][i][j].index].x * scale + 5, origin.y() - solutions[0][i][pole_intersections[0][i][j].index].y * scale - 4, text);
				}
			}
		}
	}
		
	if (showCirclePointCurve) {
		// draw circle point curve
		if (solutions.size() == 2) {
			for (int i = 0; i < extreme_poses.size(); i++) {
				for (int j = 0; j < extreme_poses[i].size(); j++) {
					int index = std::get<0>(extreme_poses[i][j]);
					int pose1 = std::get<1>(extreme_poses[i][j]);
					int pose2 = std::get<2>(extreme_poses[i][j]);
					if (pose1 == -1 || pose2 == -1) {
						painter.setPen(QPen(QColor(255, 0, 0), 1));
						painter.setBrush(QBrush(QColor(255, 0, 0)));
					}
					else {
						painter.setPen(QPen(QColor(255, 0, 0), 3));
						painter.setBrush(QBrush(QColor(255, 0, 0)));
					}

					int end;
					if (j < extreme_poses[i].size() - 1) {
						end = std::get<0>(extreme_poses[i][j + 1]);
					}
					else {
						end = solutions[1][i].size();
					}

					for (int k = index; k < end; k++) {
						int next = (k + 1) % solutions[1][i].size();
						if (glm::length(solutions[1][i][k] - solutions[1][i][next]) > 20) continue;
						painter.drawLine(origin.x() + solutions[1][i][k].x * scale, origin.y() - solutions[1][i][k].y * scale, origin.x() + solutions[1][i][next].x * scale, origin.y() - solutions[1][i][next].y * scale);
					}
				}
			}
		}

		// draw poles
		if (poles.size() == 2) {
			for (int i = 0; i < poles[1].size(); i++) {
				for (int j = i + 1; j < poles[1][i].size(); j++) {
					painter.setPen(QPen(QColor(0, 0, 0), 1));
					painter.setBrush(QBrush(QColor(0, 0, 0)));
					painter.drawEllipse(QPoint(origin.x() + poles[1][i][j].x * scale, origin.y() - poles[1][i][j].y * scale), 3, 3);

					painter.setPen(QPen(QColor(0, 0, 0), 1));
					QString text = QString("P%1%2'").arg(i + 1).arg(j + 1);
					painter.drawText(origin.x() + poles[1][i][j].x * scale + 5, origin.y() - poles[1][i][j].y * scale + 12, text);
				}
			}
		}

		// draw pole intersections
		if (pole_intersections.size() == 2) {
			for (int i = 0; i < pole_intersections[1].size(); i++) {
				for (int j = 0; j < pole_intersections[1][i].size(); j++) {
					painter.setPen(QPen(QColor(0, 0, 0), 1));
					painter.setBrush(QBrush(QColor(0, 255, 255)));
					painter.drawEllipse(QPoint(origin.x() + solutions[1][i][pole_intersections[1][i][j].index].x * scale, origin.y() - solutions[1][i][pole_intersections[1][i][j].index].y * scale), 3, 3);

					painter.setPen(QPen(QColor(0, 0, 0), 1));
					QString text = QString("Q%1%2'").arg(pole_intersections[1][i][j].subscript.first + 1).arg(pole_intersections[1][i][j].subscript.second + 1);
					painter.drawText(origin.x() + solutions[1][i][pole_intersections[1][i][j].index].x * scale + 5, origin.y() - solutions[1][i][pole_intersections[1][i][j].index].y * scale - 4, text);
				}
			}
		}

		// DEBUG
		/*
		if (poles.size() >= 8) {
			painter.setPen(QPen(QColor(0, 0, 0), 1));
			painter.setBrush(QBrush(QColor(0, 0, 0)));
			glm::dvec2 p1 = poles[8] + (poles[1] - poles[8]) * 100.0;
			glm::dvec2 p2 = poles[1] + (poles[8] - poles[1]) * 100.0;
			painter.drawLine(origin.x() + p1.x * scale, origin.y() - p1.y * scale, origin.x() + p2.x * scale, origin.y() - p2.y * scale);
		}
		*/

		// draw Us and Ts
		for (int i = 0; i < UTs.size(); i++) {
			for (int j = 0; j < UTs[i].size(); j++) {
				painter.setPen(QPen(QColor(0, 0, 0), 1));
				painter.setBrush(QBrush(QColor(255, 255, 0)));
				painter.drawEllipse(QPoint(origin.x() + solutions[1][i][UTs[i][j].index].x * scale, origin.y() - solutions[1][i][UTs[i][j].index].y * scale), 3, 3);

				painter.setPen(QPen(QColor(0, 0, 0), 1));
				QString text = QString("U%1%2").arg(UTs[i][j].subscript.first + 1).arg(UTs[i][j].subscript.second + 1);
				painter.drawText(origin.x() + solutions[1][i][UTs[i][j].index].x * scale + 5, origin.y() - solutions[1][i][UTs[i][j].index].y * scale - 4, text);
			}
		}

		// draw B
		for (int i = 0; i < Bs.size(); i++) {
			painter.setPen(QPen(QColor(0, 0, 0), 1));
			painter.setBrush(QBrush(QColor(255, 255, 0)));
			painter.drawEllipse(QPoint(origin.x() + Bs[i].x * scale, origin.y() - Bs[i].y * scale), 3, 3);

			painter.drawText(origin.x() + Bs[i].x * scale + 5, origin.y() - Bs[i].y * scale - 4, "B");
		}
	}

#if 0
	if (poles.size() >= 8) {
		// draw Burmester
		painter.setPen(QPen(QColor(0, 0, 0), 1, Qt::DashLine));
		painter.setBrush(QBrush(QColor(0, 0, 0, 0)));

		painter.drawLine(origin.x() + poles[0].x * scale, origin.y() - poles[0].y * scale, origin.x() + poles[1].x * scale, origin.y() - poles[1].y * scale);
		painter.drawLine(origin.x() + poles[4].x * scale, origin.y() - poles[4].y * scale, origin.x() + poles[5].x * scale, origin.y() - poles[5].y * scale);

		glm::dvec2 v1 = poles[1] - poles[2];
		v1 /= glm::length(v1);
		glm::dvec2 h1(-v1.y, v1.x);
		glm::dvec2 u1(cos(alpha) * v1.x - sin(alpha) * v1.y, sin(alpha) * v1.x + cos(alpha) * v1.y);
		glm::dvec2 M1;
		kinematics::lineLineIntersection(poles[2], u1, (poles[1] + poles[2]) * 0.5, h1, M1);
		double r1 = glm::length(poles[2] - M1);
		painter.drawLine(origin.x() + poles[2].x * scale, origin.y() - poles[2].y * scale, origin.x() + M1.x * scale, origin.y() - M1.y * scale);
		painter.drawEllipse(QPoint(origin.x() + M1.x * scale, origin.y() - M1.y * scale), (int)(r1 * scale + 0.5), (int)(r1 * scale + 0.5));

		if (showCenterPointCurve) {
			glm::dvec2 v2 = poles[3] - poles[4];
			v2 /= glm::length(v2);
			glm::dvec2 h2(-v2.y, v2.x);
			glm::dvec2 u2(cos(alpha) * v2.x - sin(alpha) * v2.y, sin(alpha) * v2.x + cos(alpha) * v2.y);
			glm::dvec2 M2;
			kinematics::lineLineIntersection(poles[4], u2, (poles[3] + poles[4]) * 0.5, h2, M2);
			double r2 = glm::length(poles[4] - M2);
			painter.setPen(QPen(QColor(0, 0, 0), 1, Qt::DashLine));
			painter.setBrush(QBrush(QColor(0, 0, 0, 0)));
			painter.drawLine(origin.x() + poles[4].x * scale, origin.y() - poles[4].y * scale, origin.x() + M2.x * scale, origin.y() - M2.y * scale);
			painter.drawEllipse(QPoint(origin.x() + M2.x * scale, origin.y() - M2.y * scale), (int)(r2 * scale + 0.5), (int)(r2 * scale + 0.5));

			try {
				painter.setPen(QPen(QColor(0, 0, 0), 1));
				painter.setBrush(QBrush(QColor(0, 0, 0, 255)));
				glm::dvec2 C1 = kinematics::circleCircleIntersection(M1, r1, M2, r2);
				glm::dvec2 C2 = kinematics::circleCircleIntersection(M2, r2, M1, r1);
				painter.drawEllipse(QPoint(origin.x() + C1.x * scale, origin.y() - C1.y * scale), 3, 3);
				painter.drawEllipse(QPoint(origin.x() + C2.x * scale, origin.y() - C2.y * scale), 3, 3);
			}
			catch (char* ex) {
			}
		}


		if (showCirclePointCurve) {
			glm::dvec2 v3 = poles[6] - poles[7];
			v3 /= glm::length(v3);
			glm::dvec2 h3(-v3.y, v3.x);
			glm::dvec2 u3(cos(alpha) * v3.x - sin(alpha) * v3.y, sin(alpha) * v3.x + cos(alpha) * v3.y);
			glm::dvec2 M3;
			kinematics::lineLineIntersection(poles[7], u3, (poles[6] + poles[7]) * 0.5, h3, M3);
			double r3 = glm::length(poles[7] - M3);
			painter.setPen(QPen(QColor(0, 0, 0), 1, Qt::DashLine));
			painter.setBrush(QBrush(QColor(0, 0, 0, 0)));
			painter.drawLine(origin.x() + poles[7].x * scale, origin.y() - poles[7].y * scale, origin.x() + M3.x * scale, origin.y() - M3.y * scale);
			painter.drawEllipse(QPoint(origin.x() + M3.x * scale, origin.y() - M3.y * scale), (int)(r3 * scale + 0.5), (int)(r3 * scale + 0.5));

			try {
				painter.setPen(QPen(QColor(0, 0, 0), 1));
				painter.setBrush(QBrush(QColor(0, 0, 0, 255)));
				glm::dvec2 C1 = kinematics::circleCircleIntersection(M1, r1, M3, r3);
				glm::dvec2 C2 = kinematics::circleCircleIntersection(M3, r3, M1, r1);
				painter.drawEllipse(QPoint(origin.x() + C1.x * scale, origin.y() - C1.y * scale), 3, 3);
				painter.drawEllipse(QPoint(origin.x() + C2.x * scale, origin.y() - C2.y * scale), 3, 3);
			}
			catch (char* ex) {
			}
		}
	}
#endif

	// draw boxes
	painter.setPen(QPen(QColor(0, 0, 0), 1, Qt::DashLine));
	painter.setBrush(QBrush(QColor(0, 0, 0, 0)));
	for (int i = 0; i < body_pts.size(); i++) {
		QPolygonF pts;
		for (int j = 0; j < body_pts[i].size(); j++) {
			pts.push_back(QPointF(origin.x() + body_pts[i][j].x * scale, origin.y() - body_pts[i][j].y * scale));
		}
		painter.drawPolygon(pts);
	}

	kinematics.draw(painter, origin, scale);

	painter.setPen(QPen(QColor(0, 0, 0)));
	if (linkage_type >= 0) {
		if (linkage_type == 0) {
			painter.drawText(QPoint(6, 20), "Grashof (Drag-link)");
		}
		else if (linkage_type == 1) {
			painter.drawText(QPoint(6, 20), "Grashof (Crank-rocker)");
		}
		else if (linkage_type == 2) {
			painter.drawText(QPoint(6, 20), "Grashof (Rocker-crank)");
		}
		else if (linkage_type == 3) {
			painter.drawText(QPoint(6, 20), "Grashof (Double-rocker)");
		}
		else if (linkage_type == 4) {
			painter.drawText(QPoint(6, 20), "Non-Grashof (0-0 rocker)");
		}
		else if (linkage_type == 5) {
			painter.drawText(QPoint(6, 20), "Non-Grashof (pi-pi rocker)");
		}
		else if (linkage_type == 6) {
			painter.drawText(QPoint(6, 20), "Non-Grashof (pi-0 rocker)");
		}
		else if (linkage_type == 7) {
			painter.drawText(QPoint(6, 20), "Non-Grashof (0-pi rocker)");
		}
	}
	painter.setPen(QPen(QColor(255, 0, 0)));
	if (grashofDefect) {
		painter.drawText(QPoint(6, 36), "Grashof defect");
	}
	if (orderDefect) {
		painter.drawText(QPoint(6, 52), "Order defect");
	}
	if (branchDefect) {
		painter.drawText(QPoint(6, 68), "Branch defect");
	}
}

void Canvas::mousePressEvent(QMouseEvent* e) {
	if (e->buttons() & Qt::LeftButton) {
		// convert the mouse position to the world coordinate system
		glm::dvec2 pt((e->x() - origin.x()) / scale, -(e->y() - origin.y()) / scale);

		// select a joint to move
		selectedJoint.reset();
		double min_dist = 6;
		for (int i = 0; i < kinematics.diagram.joints.size(); i++) {
			double dist = glm::length(kinematics.diagram.joints[i]->pos - pt);
			if (dist < min_dist) {
				min_dist = dist;
				selectedJoint = kinematics.diagram.joints[i];
			}
		}
	}

	prev_mouse_pt = e->pos();
}

void Canvas::mouseMoveEvent(QMouseEvent* e) {
	if (e->buttons() & Qt::LeftButton && selectedJoint) {
		if (ctrlPressed) {
			// move the selected joint
			selectedJoint->pos.x = (e->x() - origin.x()) / scale;
			selectedJoint->pos.y = -(e->y() - origin.y()) / scale;

			// update the geometry
			kinematics.diagram.bodies.clear();
			kinematics.diagram.addBody(kinematics.diagram.joints[2], kinematics.diagram.joints[3], body_pts[0]);

			// setup the kinematic system
			kinematics.diagram.initialize();
			update();

			grashofDefect = checkGrashofDefect();
			orderDefect = checkOrderDefect();
			branchDefect = checkBranchDefect();
		}
		else {
			// select a solution
			glm::dvec2 pt((e->x() - origin.x()) / scale, -(e->y() - origin.y()) / scale);
			selectedSolution = findSolution(selectedJoint->ground, pt);

			// move the selected joint
			int offset = 1;
			if (selectedJoint->ground) {
				offset = 0;
			}
			selectedJoint->pos = solutions[offset][selectedSolution.first][selectedSolution.second];

			// move the other end joint
			if (selectedJoint->ground) {
				kinematics.diagram.joints[selectedJoint->id + 2]->pos = solutions[1][selectedSolution.first][selectedSolution.second];
			}
			else {
				kinematics.diagram.joints[selectedJoint->id - 2]->pos = solutions[0][selectedSolution.first][selectedSolution.second];
			}

			// initialize the other link
			if (selectedJoint->ground) {
				int joint_id = 1 - selectedJoint->id;
				std::pair<int, int> sol_index = findSolution(selectedJoint->ground, kinematics.diagram.joints[joint_id]->pos);
				kinematics.diagram.joints[joint_id]->pos = solutions[0][sol_index.first][sol_index.second];
				kinematics.diagram.joints[joint_id + 2]->pos = solutions[1][sol_index.first][sol_index.second];
			}
			else {
				int joint_id = 5 - selectedJoint->id;
				std::pair<int, int> sol_index = findSolution(selectedJoint->ground, kinematics.diagram.joints[joint_id]->pos);
				kinematics.diagram.joints[joint_id - 2]->pos = solutions[0][sol_index.first][sol_index.second];
				kinematics.diagram.joints[joint_id]->pos = solutions[1][sol_index.first][sol_index.second];
			}

			// update the geometry
			kinematics.diagram.bodies.clear();
			kinematics.diagram.addBody(kinematics.diagram.joints[2], kinematics.diagram.joints[3], body_pts[0]);

			// setup the kinematic system
			kinematics.diagram.initialize();
			update();

			grashofDefect = checkGrashofDefect();
			orderDefect = checkOrderDefect();
			branchDefect = checkBranchDefect();
		}
	}
	else {
		// move the camera
		if (e->buttons() & Qt::RightButton) {
			// translate the Origin
			origin += e->pos() - prev_mouse_pt;
			update();
		}
	}

	prev_mouse_pt = e->pos();
}

void Canvas::mouseReleaseEvent(QMouseEvent* e) {
}

void Canvas::mouseDoubleClickEvent(QMouseEvent* e) {
}

void Canvas::wheelEvent(QWheelEvent* e) {
	scale += e->delta() * 0.01;
	scale = std::min(std::max(0.1, scale), 1000.0);
	update();
}

void Canvas::resizeEvent(QResizeEvent *e) {
}

void Canvas::keyPressEvent(QKeyEvent* e) {
	ctrlPressed = false;
	shiftPressed = false;

	if (e->modifiers() & Qt::ControlModifier) {
		ctrlPressed = true;
	}
	if (e->modifiers() & Qt::ShiftModifier) {
		shiftPressed = true;
	}

	switch (e->key()) {
	case Qt::Key_Escape:
		break;
	case Qt::Key_Space:
		break;
	case Qt::Key_Delete:
		break;
	case Qt::Key_Z:
		alpha -= 0.01;
		std::cout << alpha / kinematics::M_PI * 180 << std::endl;
		update();
		break;
	case Qt::Key_X:
		alpha += 0.01;
		std::cout << alpha / kinematics::M_PI * 180 << std::endl;
		update();
		break;
	}

	update();
}

void Canvas::keyReleaseEvent(QKeyEvent* e) {
	switch (e->key()) {
	case Qt::Key_Control:
		ctrlPressed = false;
		break;
	case Qt::Key_Shift:
		shiftPressed = false;
		break;
	default:
		break;
	}
}

