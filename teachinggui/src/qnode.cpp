#include "qnode.hpp"

namespace teachinggui {
/**
 * @brief Construct a new QNode::QNode object
 */
QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}
/**
 * @brief Destroy the QNode::QNode object
 */
QNode::~QNode()
{
    if(ros::isStarted()) {
      ros::shutdown();
      ros::waitForShutdown();
    }
	wait();
}
/**
 * @brief Initialising function for Subscribers, Publishers, Timers
 * 
 * @param topicName 
 * @return true 
 * @return false 
 */
bool QNode::init()
{
	addToLogWindow("Connecting to ROS Master ...");

	ros::init(init_argc,init_argv,"teachinggui");
	if ( ! ros::master::check() )
	{
		addToLogWindow("Connection failed.");
	}
	ros::start();
	ros::NodeHandle n;

	addToLogWindow("Connected, Instantiating Subscribers, Publishers, Timers");

	timerDelay = n.createTimer(ros::Duration(delay), &QNode::timerDelayCallback, this);
	thetaSubscriber = n.subscribe(subTopicName, 1000, &QNode::thetaSubscriberCallback, this);
    thetaPublisher = n.advertise<std_msgs::Int16MultiArray>(pubTopicName, 10);
	
	start();
	return true;
}
/**
 * @brief Run function starts ROS processes
 */
void QNode::run()
{
	while ( ros::ok() )
	{
		ros::spin();
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown();
}
/**
 * @brief Subscriber Callback for Theta values coming from Robot
 * 
 * @param msg 
 */
void QNode::thetaSubscriberCallback(const std_msgs::Int16MultiArray& msg)
{
	Theta1 = msg.data[0];
	Theta2 = msg.data[1];
	Theta3 = msg.data[2];
	actPos = msg.data[3];
}
/**
 * @brief function for saving current Theta Values into int-Vectors
 */
void QNode::savePoseFunction(int endeffectorState)
{
	pose[0] = Theta1;
	pose[1] = Theta2;
	pose[2] = Theta3;
	pose[3] = endeffectorState;

	poses.push_back(pose);

	addToLogWindow(	"Pose " + QString::number(cnt+1) + 
					": T1: " + QString::number(poses[cnt][0]) + 
					" deg    T2: " + QString::number(poses[cnt][1]) + 
					" deg    T3: " + QString::number(poses[cnt][2]) +
					" deg" + " Endeffector:    " + QString::number(poses[cnt][3]));

	cnt++;
}
/**
 * @brief function opens textfile, and copys values from int-Vectors into txt-File, closes textfile
 */
void QNode::saveToFileFunction(std::string fileName)
{
	addToLogWindow("saveToFileFunction executed");
	std::ofstream outputFile;
	outputFile.open(filePath + fileName, std::ofstream::out | std::ofstream::trunc);
	if (!outputFile)
	{
		logging_model.insertRows(logging_model.rowCount(),1);
		index = logging_model.index(logging_model.rowCount() - 1, 0);
		logging_model.setData(index, "Error while writing to file: File location not available or no Pose(s) saved!");
	}
	addToLogWindow("outputFile successfully opened");
	for(i = 0; i<cnt; i++)
	{
		outputFile << poses[i][0] << ',' << poses[i][1] << ',' << poses[i][2] << ',' << poses[i][3] << std::endl;
	}
	i=0;

	addToLogWindow(QString::number(cnt) + " Poses saved to: " + QString::fromStdString(filePath + fileName));
	addToLogWindow("data written to outputFile");

	clearPosesVector();
	outputFile.close();

	addToLogWindow("integer vectors with theta Values cleared, outputFile closed");
	cnt = 0;
}
/**
 * @brief function opens textfile, reads integer values and copies them into int-Vectors
 * 
 * @param fileName 
 */
void QNode::startRoutine(std::string fileName)
{
	addToLogWindow("Loading file: " + QString::fromStdString(filePath + fileName));
	addToLogWindow("Routine execution started, proceeding to open inputFile");

	inputFile.open(filePath + fileName);
	if (!inputFile)
	{
		addToLogWindow("Error while loading file");
	}

	addToLogWindow("inputFile opened, proceeding to read integer Values and copy them into integer-Vectors");
	while (inputFile >> pose[0] >> comma >> pose[1] >> comma >> pose[2] >> comma >> pose[3])
	{
		poses.push_back(pose);
		cnt++;
	}

	addToLogWindow(QString::number(cnt-1) + " Poses detected");
	printPosesVector();

	execRoutine = true;
	inputFile.close();
}

/**
 * @brief function to fully clear poses vector
 * 
 */
void QNode::clearPosesVector()
{
	for (int j = 0; j < cnt; j++)
	{
		poses[j].clear();	
	}
	poses.clear();
}
/**
 * @brief prints poses vector to logging screen
 * 
 */
void QNode::printPosesVector()
{
	addToLogWindow("Read following poses:");

	for (int j = 0; j < cnt; j++)
	{
		addToLogWindow("Pose " + QString::number(j+1) + 
						": T1: " + QString::number(poses[j][0]) + 
						" deg    T2: " + QString::number(poses[j][1]) + 
						" deg    T3: " + QString::number(poses[j][2]) +
						" deg" + " Endeffector:    " + QString::number(poses[j][3]));	
	}
}
/**
 * @brief timer callback function which checks if desired pose is reached, if so next pose is published
 * 
 */
void QNode::timerDelayCallback(const ros::TimerEvent&)
{
	if (execRoutine == true)
	{
		if (poseReached == true ) 
		{
			// T1	T2	T3	Sauger-1/0 Nr.Position
			thetaMsg.data.push_back(poses[i][0]);
			thetaMsg.data.push_back(poses[i][1]);
			thetaMsg.data.push_back(poses[i][2]);
			thetaMsg.data.push_back(poses[i][3]);
			thetaMsg.data.push_back(i);

			thetaPublisher.publish(thetaMsg);
			poseReached = false;
			addToLogWindow("Aiming for Pose " + QString::number(i+1) + 
						": T1: " + QString::number(poses[i][0]) + 
						"deg    T2: " + QString::number(poses[i][1]) + 
						"deg    T3: " + QString::number(poses[i][2]) + "deg");
		}

		if (actPos == i + 1)
		{
			addToLogWindow("Pose " + QString::number(i+1) + " reached");
			i++;
			thetaMsg.data.clear();
			poseReached = true;

			if (i == cnt)
			{
				ROS_INFO("All poses reached, end of execution");
				clearPosesVector();

				inputFile.close();
				cnt = 0;
				i = 0;
				execRoutine = false;
			}
		}
	}
}
/**
 * @brief increments index for logging view and adds new line
 * 
 * @param newLine 
 */
void QNode::addToLogWindow(QString newLine)
{
	ROS_INFO("%s", newLine.toStdString().c_str());
	logging_model.insertRows(logging_model.rowCount(),1);
	index = logging_model.index(logging_model.rowCount() - 1, 0);
	logging_model.setData(index, newLine);
}

}  // namespace teachinggui
