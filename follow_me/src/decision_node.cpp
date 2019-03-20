#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include <cmath>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>

class decision
{
  private:
	ros::NodeHandle n;

	// communication with one_moving_person_detector or person_tracker
	ros::Publisher pub_goal_reached;
	ros::Subscriber sub_goal_to_reach;

	// communication with rotation
	ros::Publisher pub_movement_to_do;
	ros::Subscriber sub_movement_done;

	geometry_msgs::Point movement_to_do; // x = translation, y = rotation
	geometry_msgs::Point movement_done;

	bool new_goal_to_reach; // to check if a new /goal_to_reach is available or not
	bool new_movement_done; // to check if a new /rotation_done is available or not

	geometry_msgs::Point goal_to_reach;
	geometry_msgs::Point goal_reached;

	int state;
	bool display_state;

  public:
	decision()
	{

		// communication with moving_persons_detector or person_tracker
		pub_goal_reached = n.advertise<geometry_msgs::Point>("goal_reached", 1);
		sub_goal_to_reach = n.subscribe("goal_to_reach", 1, &decision::goal_to_reachCallback, this);

		// communication with rotation_action
		pub_movement_to_do = n.advertise<geometry_msgs::Point>("movement_to_do", 0);
		sub_movement_done = n.subscribe("movement_done", 1, &decision::movement_doneCallback, this);

		state = 1;
		display_state = false;
		new_goal_to_reach = false;
		new_movement_done = false;

		// INFINTE LOOP TO COLLECT LASER DATA AND PROCESS THEM
		ros::Rate r(10); // this node will work at 10hz
		while (ros::ok()) {
			ros::spinOnce(); // each callback is called once
			update();
			r.sleep(); // we wait if the processing (ie, callback+update) has taken less than 0.1s (ie, 10 hz)
		}
	}

	// UPDATE: main processing
	/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
	void update()
	{

		if (!display_state) {
			display_state = true;
			ROS_INFO("state: %i", state);
		}

		// we receive a new /goal_to_reach and robair is not doing a translation or a rotation
		if ((new_goal_to_reach) && (state == 1)) {

			ROS_INFO("(decision_node) /goal_to_reach received: (%f, %f)", goal_to_reach.x, goal_to_reach.y);
			new_goal_to_reach = false;

			// we have a rotation and a translation to perform
			// we compute the /translation_to_do
			double translation_to_do = sqrt((goal_to_reach.x * goal_to_reach.x) + (goal_to_reach.y * goal_to_reach.y));

			if (translation_to_do) {
				// we compute the /rotation_to_do
				double rotation_to_do = acos(goal_to_reach.x / translation_to_do);

				if (goal_to_reach.y < 0)
					rotation_to_do *= -1;

				display_state = false;

				ROS_INFO("(decision_node) /rotation_to_do: %f", rotation_to_do * 180 / M_PI);

				geometry_msgs::Point msg_movement_to_do;
				msg_movement_to_do.x = translation_to_do;
                msg_movement_to_do.y = rotation_to_do;
				pub_movement_to_do.publish(msg_movement_to_do);

				state = 3;
			} else {
				geometry_msgs::Point msg_goal_reached;
				msg_goal_reached.x = 0;
				msg_goal_reached.y = 0;

				ROS_INFO("(decision_node) /goal_reached (%f, %f)", msg_goal_reached.x, msg_goal_reached.y);
				pub_goal_reached.publish(msg_goal_reached);
			}
		}

		// we receive an ack from translation_action_node. So, we send an ack to the moving_persons_detector_node
		if ((new_movement_done) && (state == 3)) {
			ROS_INFO("(decision_node) /movement_done : %f,%lf\n", movement_done.x, movement_done.y);
			new_movement_done = false;

			display_state = false;
			// the translation_to_do is done so we send the goal_reached to the detector/tracker node
			geometry_msgs::Point msg_goal_reached;
			ROS_INFO("(decision_node) /goal_reached (%f, %f)", msg_goal_reached.x, msg_goal_reached.y);
			// to complete

			msg_goal_reached = goal_reached;

			pub_goal_reached.publish(msg_goal_reached);
			new_goal_to_reach = false;

			ROS_INFO(" ");
			ROS_INFO("(decision_node) waiting for a /goal_to_reach");
			state = 1;
		}

	} // update

	// CALLBACKS
	/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
	void goal_to_reachCallback(const geometry_msgs::Point::ConstPtr& g)
	{
		// process the goal received from moving_persons detector

		new_goal_to_reach = true;
		goal_to_reach.x = g->x;
		goal_to_reach.y = g->y;
	}

	void movement_doneCallback(const geometry_msgs::Point::ConstPtr& r)
	{
		// process the range received from the translation node

		new_movement_done = true;
		movement_done.x = r->x;
        movement_done.y = r->y;
	}

	// Distance between two points
	float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb)
	{

		return sqrt(pow((pa.x - pb.x), 2.0) + pow((pa.y - pb.y), 2.0));
	}
};

int main(int argc, char **argv)
{

	ROS_INFO("(decision_node) waiting for a /goal_to_reach");
	ros::init(argc, argv, "decision");

	decision bsObject;

	ros::spin();

	return 0;
}
