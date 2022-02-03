#include <mbot/mbot_channels.h>
#include <common/timestamp.h>
#include <lcmtypes/mbot_motor_command_t.hpp>
#include <lcmtypes/odometry_t.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <lcmtypes/robot_path_t.hpp>
#include <lcmtypes/timestamp_t.hpp>
#include <lcmtypes/message_received_t.hpp>
#include <common/angle_functions.hpp>
#include <common/pose_trace.hpp>
#include <common/lcm_config.h>
#include <slam/slam_channels.h>
#include <lcm/lcm-cpp.hpp>
#include <algorithm>
#include <iostream>
#include <cassert>
#include <signal.h>
#include <unistd.h>
#include "maneuver_controller.h"
#include <unistd.h>

#include <rc/math.h>
#include <rc/time.h>

/////////////////////// TODO: /////////////////////////////
/**
 * Code below is a little more than a template. You will need
 * to update the maneuver controllers to function more effectively
 * and/or add different controllers. 
 * You will at least want to:
 *  - Add a form of PID to control the speed at which your
 *      robot reaches its target pose.
 *  - Add a rotation element to the StratingManeuverController
 *      to maintian a avoid deviating from the intended path.
 *  - Limit (min max) the speeds that your robot is commanded
 *      to avoid commands to slow for your bots or ones too high
 */
///////////////////////////////////////////////////////////

const float k_omega = 0.7;
const float v_desired = 0.8;
const float w_max = 3.14;
const float kw_p = 1.0;
const float kw_d = 1.0;

float  max(float a, float b){
	if (a >= b) return a;
	else return b;
}

class StraightManeuverController : public ManeuverControllerBase
{
public:
    StraightManeuverController() = default;
 
    int toggle = 0.0;

    virtual mbot_motor_command_t get_command(const pose_xyt_t &pose, const pose_xyt_t &target) override
    {
        
        // calculate error in heading
        float dx = target.x - pose.x;
        float dy = target.y - pose.y;
        float target_heading = atan2(dy, dx);
        float heading_error = angle_diff(pose.theta, target_heading);

        // compute desired speed and heading

	/*
        float r = sqrt(dx*dx + dy*dy);
	float v = 0.0;
	if (toggle < 3)
	{
        	v = 0.5 * v_desired * cos(heading_error)*cos(heading_error);
		toggle++;
	}
	else if (toggle < 8)
	{
        	v = v_desired * cos(heading_error)*cos(heading_error);
		toggle++;
	}
	else
	{
        // saturated pid
        float r = sqrt(dx*dx + dy*dy);
	float R = 1.7;
        float r_sat = r/R;
        if (r_sat >= 1.0) r_sat = 1.0;
	

        v = r_sat * v_desired * cos(heading_error)*cos(heading_error);
	// v = 0.0;
	}*/

	
        // saturated pid
        float r = sqrt(dx*dx + dy*dy);
        float R = 0.61;
        float r_sat = r/R;
        if (r_sat >= 1.0) r_sat = 1.0;

        float v = r_sat * v_desired * cos(heading_error)*cos(heading_error);
	

	printf("toggle: %d\tcurrent pose: %f\tr: %f\tv: %f\n", toggle, pose.x, r, v);

        float w = - k_omega * heading_error;
        if (w >  w_max) w =  w_max;
        if (w < -w_max) w = -w_max;

        // w = 0.0*w;



        return {0, v, w};
    }

/*
    virtual bool target_reached(const pose_xyt_t &pose, const pose_xyt_t &target) override
    {
        return ((fabs(pose.x - target.x) < 0.1) && (fabs(pose.y - target.y) < 0.1));
    }
*/

    virtual bool target_reached(const pose_xyt_t &pose, const pose_xyt_t &target) override
    {
        // this just gets you to a desired location, but doesnt require it to turn to head in the right direction
        if ((fabs(pose.x - target.x) < 0.1) && (fabs(pose.y - target.y) < 0.1))
	{
		toggle = 0;
		return 1;
	}
	else
		return 0;
    }
};

class TurnManeuverController : public ManeuverControllerBase
{
public:
    TurnManeuverController() = default;

	float prev_heading_error;
	int toggle = 0.0;
    virtual mbot_motor_command_t get_command(const pose_xyt_t &pose, const pose_xyt_t &target) override
    {

        // calculate error in heading
        float dx = target.x - pose.x;
        float dy = target.y - pose.y;
        float target_heading = atan2(dy, dx);
        float heading_error = angle_diff(pose.theta, target_heading);

        // float w = -2.0 *  heading_error;
        // if (w > w_max) w = w_max;
        // if (w < -w_max) w = -w_max;

        float w = 0.0;
	
	
	// works well for pi/4
        if (heading_error < -1){
            w = w_max;
        }
        else if(heading_error > 1){
            w = -w_max;
        } 
        else{
            w = -0.5 * heading_error;
        }


	/*
	// works well for pi/4 on my table
        if (heading_error < -1){
            w = w_max;
        }
        else if(heading_error > 1){
            w = -w_max;
        } 
        else{
            w = -1.5 * heading_error;
        }*/

	
	/*
	if (fabs(heading_error) > 1.4)
	{
		w = -(kw_p * heading_error + kw_d * (heading_error - prev_heading_error));
		prev_heading_error = heading_error;
	}
	else
		w = -0.5 * heading_error;
	*/

	/* This works for pi rad/s
	if(heading_error > 1.0)
	{	
		if(toggle == 0)
		{	
			w = -w_max;
			toggle = 1;
			usleep(1000);
		}
		else
		{
			w = 0.0;
			toggle = 0;
			usleep(10000);
		}	
	}
	else if(heading_error < -1.0)
	{	
		if(toggle == 0)
		{	
			w = w_max;
			toggle = 1;
			usleep(1000);
		}
		else
		{
			w = 0.0;
			toggle = 0;
			usleep(10000);
		}	
	}
	else
		w = -0.5 * heading_error;
	*/

	/*
	if (toggle < 2)
	{
		if (heading_error < 0)
			w = w_max / 2.0;
		else 
			w = -w_max / 2.0;
		
		// usleep(1000);

		toggle++;
	}
	if (toggle < 6)
	{
		if (heading_error < 0)
			w = w_max;
		else 
			w = -w_max;
		
		// usleep(1000);

		toggle++;
	}
	else
	{
		w = -0.5 * heading_error;
	}*/


	/*
	if (toggle < 4)
	{
		if (heading_error < 0)
			w = w_max;
		else 
			w = -w_max;
		
		// usleep(1000);

		toggle++;
	}
	else
	{
		w = -0.5 * heading_error;
	}*/	
	

	printf("toggle: %d\tw: %f\n", toggle, w);

	// printf("heading error: %f\n", heading_error);
	// printf("w after processing: %f\n", w); 
	// printf("**************************************");
        
        return {0, 0, w};
    }

/*
    virtual bool target_reached(const pose_xyt_t &pose, const pose_xyt_t &target) override
    {
        // calculate error in heading
        float dx = target.x - pose.x;
        float dy = target.y - pose.y;
        float target_heading = atan2(dy, dx);
        float heading_error = angle_diff(pose.theta, target_heading);
        return (fabs(heading_error) < 0.05);
    }
*/

    virtual bool target_reached(const pose_xyt_t &pose, const pose_xyt_t &target) override
    {
        // calculate error in heading
        float dx = target.x - pose.x;
        float dy = target.y - pose.y;
        float target_heading = atan2(dy, dx);
        float heading_error = angle_diff(pose.theta, target_heading);
	
	if (fabs(heading_error) < 0.05)
	{
		toggle = 0;
		return 1;
	}        
	else
		return 0;

    }
};


class MotionController
{
public:
    /**
    * Constructor for MotionController.
    */
    MotionController(lcm::LCM *instance)
        : lcmInstance(instance),
          odomToGlobalFrame_{0, 0, 0, 0}
    {
        subscribeToLcm();

        time_offset = 0;
        timesync_initialized_ = false;
    }

    /**
    * \brief updateCommand calculates the new motor command to send to the Mbot. This method is called after each call to
    * lcm.handle. You need to check if you have sufficient data to calculate a new command, or if the previous command
    * should just be used again until for feedback becomes available.
    * 
    * \return   The motor command to send to the mbot_driver.
    */
    mbot_motor_command_t updateCommand(void)
    {
        mbot_motor_command_t cmd{now(), 0.0, 0.0};

        if (odomTrace_.empty())
        {
            printf("ODOM TRACE IS EMPTY!\n");
        }
	else{
	//pose_xyt_t pose = currentPose();		
        //printf("CURRENT POSE: %f, %f, %f\n", pose.x, pose.y, pose.theta);

	}
	if (!targets_.empty() && !odomTrace_.empty())
        {
            pose_xyt_t target = targets_.back();
            pose_xyt_t pose = currentPose();

            if (state_ == TURN)
            {
                // printf("IN STATE TURN \n");
                if (turn_controller.target_reached(pose, target))
                {
                    state_ = DRIVE;
		    sleep(1);
                }
                else
                {
                    cmd = turn_controller.get_command(pose, target);
                }
            }
            else if (state_ == DRIVE)
            {
                // printf("IN STATE DRIVE \n");
                if (straight_controller.target_reached(pose, target))
                {
                    if (!assignNextTarget())
                    {
                        std::cout << "\rTarget Reached!";
                    }
                    sleep(1);
		}
                else
                {
                    cmd = straight_controller.get_command(pose, target);
                }
            }
            else
            {
                std::cerr << "ERROR: MotionController: Entered unknown state: " << state_ << '\n';
            }
        }
	//printf(" -  CMD: %f, %f\n", cmd.trans_v, cmd.angular_v);
        return cmd;
    }

    bool timesync_initialized() { return timesync_initialized_; }

    void handleTimesync(const lcm::ReceiveBuffer *buf, const std::string &channel, const timestamp_t *timesync)
    {
        timesync_initialized_ = true;
        time_offset = timesync->utime - utime_now();
    }

    void handlePath(const lcm::ReceiveBuffer *buf, const std::string &channel, const robot_path_t *path)
    {
        targets_ = path->path;
        std::reverse(targets_.begin(), targets_.end()); // store first at back to allow for easy pop_back()

        std::cout << "received new path at time: " << path->utime << "\n";
        for (auto pose : targets_)
        {
            std::cout << "(" << pose.x << "," << pose.y << "," << pose.theta << "); ";
        }
        std::cout << std::endl;

        assignNextTarget();

        //confirm that the path was received
        message_received_t confirm{now(), path->utime, channel};
        lcmInstance->publish(MESSAGE_CONFIRMATION_CHANNEL, &confirm);
    }

    void handleOdometry(const lcm::ReceiveBuffer *buf, const std::string &channel, const odometry_t *odometry)
    {
        pose_xyt_t pose{odometry->utime, odometry->x, odometry->y, odometry->theta};
        odomTrace_.addPose(pose);
    }

    void handlePose(const lcm::ReceiveBuffer *buf, const std::string &channel, const pose_xyt_t *pose)
    {
        computeOdometryOffset(*pose);
    }

private:
    enum State
    {
        TURN,
        DRIVE
    };

    pose_xyt_t odomToGlobalFrame_; // transform to convert odometry into the global/map coordinates for navigating in a map
    PoseTrace odomTrace_;          // trace of odometry for maintaining the offset estimate
    std::vector<pose_xyt_t> targets_;

    State state_;

    int64_t time_offset;
    bool timesync_initialized_;

    lcm::LCM *lcmInstance;

    TurnManeuverController turn_controller;
    StraightManeuverController straight_controller;

    int64_t now()
    {
        return utime_now() + time_offset;
    }

    bool assignNextTarget(void)
    {
        if (!targets_.empty())
        {
            targets_.pop_back();
        }
        state_ = TURN;
        return !targets_.empty();
    }

    void computeOdometryOffset(const pose_xyt_t &globalPose)
    {
        pose_xyt_t odomAtTime = odomTrace_.poseAt(globalPose.utime);
        double deltaTheta = globalPose.theta - odomAtTime.theta;
        double xOdomRotated = (odomAtTime.x * std::cos(deltaTheta)) - (odomAtTime.y * std::sin(deltaTheta));
        double yOdomRotated = (odomAtTime.x * std::sin(deltaTheta)) + (odomAtTime.y * std::cos(deltaTheta));

        odomToGlobalFrame_.x = globalPose.x - xOdomRotated;
        odomToGlobalFrame_.y = globalPose.y - yOdomRotated;
        odomToGlobalFrame_.theta = deltaTheta;
    }

    pose_xyt_t currentPose(void)
    {
        assert(!odomTrace_.empty());

        pose_xyt_t odomPose = odomTrace_.back();
        pose_xyt_t pose;
        pose.x = (odomPose.x * std::cos(odomToGlobalFrame_.theta)) - (odomPose.y * std::sin(odomToGlobalFrame_.theta)) + odomToGlobalFrame_.x;
        pose.y = (odomPose.x * std::sin(odomToGlobalFrame_.theta)) + (odomPose.y * std::cos(odomToGlobalFrame_.theta)) + odomToGlobalFrame_.y;
        pose.theta = angle_sum(odomPose.theta, odomToGlobalFrame_.theta);

        return pose;
    }

    void subscribeToLcm()
    {
        lcmInstance->subscribe(ODOMETRY_CHANNEL, &MotionController::handleOdometry, this);
        lcmInstance->subscribe(SLAM_POSE_CHANNEL, &MotionController::handlePose, this);
        lcmInstance->subscribe(CONTROLLER_PATH_CHANNEL, &MotionController::handlePath, this);
        lcmInstance->subscribe(MBOT_TIMESYNC_CHANNEL, &MotionController::handleTimesync, this);
        printf("SUBSCRIBED TO ALL CHANNELS\n");
    }
};

int main(int argc, char **argv)
{
    printf("STARTING MOTION CONTROLLER\n");
    lcm::LCM lcmInstance(MULTICAST_URL);

    MotionController controller(&lcmInstance);

    signal(SIGINT, exit);

    while (true)
    {
        lcmInstance.handleTimeout(50); // update at 20Hz minimum

        if (controller.timesync_initialized())
        {
            mbot_motor_command_t cmd = controller.updateCommand();
            lcmInstance.publish(MBOT_MOTOR_COMMAND_CHANNEL, &cmd);
        }
    }

    return 0;
}
