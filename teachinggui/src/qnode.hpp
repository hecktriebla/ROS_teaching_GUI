
#ifndef teachinggui_QNODE_HPP_
#define teachinggui_QNODE_HPP_

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/network.h>
#endif
#include <std_msgs/String.h>
#include "std_msgs/Int16.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Bool.h"
#include <QThread>
#include <QStringListModel>
#include <fstream>
#include <cmath>
#include <math.h>
#include <string>
#include <vector>

namespace teachinggui {

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	void run();

	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
    QStringListModel logging_model;
	QModelIndex index;

	bool freeServos = 0;

	void thetaSubscriberCallback(const std_msgs::Int16MultiArray& msg);
	void timerDelayCallback(const ros::TimerEvent&);

	void printPosesVector();
	void clearPosesVector();
	void savePoseFunction(int);
	void saveToFileFunction(std::string);
	void startRoutine(std::string);
	void addToLogWindow(QString);
	
Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();

private:
	int init_argc;
	char** init_argv;

	std::ifstream inputFile;

	ros::Timer timerDelay;
	ros::Subscriber thetaSubscriber;
	ros::Publisher thetaPublisher;

	std_msgs::Int16MultiArray thetaMsg;

	std::string filePath;
	std::string scaraFilePath;
	std::string subTopicName;
	std::string pubTopicName;

	int Theta1, Theta2, Theta3;
	int actPos = 0;

    std::vector<int> pose {0, 0, 0, 0};
    std::vector<std::vector<int>> poses;

    const float delay = 0.1;

	bool vecFull = false;
	bool execRoutine = false;
	bool poseReached = true;
	int cnt = 0;
	int i = 0;
	char comma = ',';
	const int deltaDeg = 3;	// depends on how accurate the robot axis is


};

}

#endif /* teachinggui_QNODE_HPP_ */
