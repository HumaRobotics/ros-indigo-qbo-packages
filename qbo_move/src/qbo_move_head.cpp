#include "ros/ros.h"

#include "std_srvs/Empty.h"
#include "qbo_arduqbo/motor_state.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"

//pointer to nodeHandle so the callback can access to it too
ros::NodeHandle* n;

ros::Publisher* move_head;
//variables for head PIDs
ros::Time timePreviousTilt;

int zeroTilt = 0;
double tiltRadPerTicks = 0;

double currentTilt = 0;
double previousTilt = 0;
double iTilt = 0;

int zeroPan = 0;
double panRadPerTicks = 0;

double currentPan = 0;
double previousPan = 0;
double iPan = 0;



void initValues()
{
    //get the zero values (in ticks)
    n->getParam("/qbo_arduqbo/dynamixelservo/head_tilt_joint/neutral", zeroTilt);
    ros::param::get("/qbo_arduqbo/dynamixelservo/head_pan_joint/neutral", zeroPan);

//get the range (in degrees) and the range (in ticks)
//calculate the convertion
    double range;
    n->getParam("/qbo_arduqbo/dynamixelservo/head_tilt_joint/range", range);
    int ticks;
    n->getParam("/qbo_arduqbo/dynamixelservo/head_tilt_joint/ticks", ticks);

    tiltRadPerTicks = range*M_PI/(ticks*180);

    n->getParam("/qbo_arduqbo/dynamixelservo/head_pan_joint/range", range);
    n->getParam("/qbo_arduqbo/dynamixelservo/head_pan_joint/ticks", ticks);

    panRadPerTicks = range*M_PI/(ticks*180);

//PID parameters as ROS params
    ros::param::set("/qbo_head_tracker/pan/kp", 0.35);
    ros::param::set("/qbo_head_tracker/pan/kd", 0.08);
    ros::param::set("/qbo_head_tracker/pan/ki", 0.01);
    ros::param::set("/qbo_head_tracker/tilt/kp", 0.6);
    ros::param::set("/qbo_head_tracker/tilt/kd", 0.07);
    ros::param::set("/qbo_head_tracker/tilt/ki", 0.005);


    timePreviousTilt = ros::Time::now();
}

bool isReadyService(std_srvs::Empty::Request  &req,
                    std_srvs::Empty::Response &res)
{
    return true;
}

//get the tilt position of the head and translate it into radians
void qboTiltCallback(const qbo_arduqbo::motor_state::ConstPtr& msg)
{
    currentTilt = (msg->position- zeroTilt)*tiltRadPerTicks;
}

//get the pan position of the head and translate it into radians
void qboPanCallback(const qbo_arduqbo::motor_state::ConstPtr& msg)
{
    currentPan = (msg->position- zeroPan)*panRadPerTicks;
}


float controlPID(float x, float ix, float dx, float Kp, float Ki, float Kd)
{
    //ROS_INFO("PID %f, %f, %f, %f, %f, %f",x,  ix, dx,  Kp,  Ki,  Kd );
    return Kp*x+Ki*ix+Kd*dx;
}


void moveHeadAbs(double pan, double tilt)
{
    sensor_msgs::JointState mvmt_msg;
    std::vector<std::string> names;
    names.push_back("head_pan_joint");
    names.push_back("head_tilt_joint");

    mvmt_msg.name = names;
    std::vector<double> position;
    position.push_back(pan);
    position.push_back(tilt);

//ROS_INFO("Moving head at: [x: %lf, y: %lf]", pan, tilt);

    mvmt_msg.position = position;

//   ROS_INFO("move head to %lf, %lf", pan, tilt);
    move_head->publish(mvmt_msg);
}

void moveHeadRel(double pan, double tilt)
{
    /****** PID calculation for tilt ***********/

//Integral term
    iTilt = 0.9*iTilt + (tilt);
//Derivated term
    ros::Time now = ros::Time::now();
    double dTilt = (tilt - previousTilt)/((now - timePreviousTilt).sec+0.001*(now - timePreviousTilt).nsec);

//Parameters
    double kp;
    ros::param::get("/qbo_head_tracker/tilt/kp", kp);
    double kd;
    ros::param::get("/qbo_head_tracker/tilt/kd", kd);
    double ki;
    ros::param::get("/qbo_head_tracker/tilt/ki", ki);

    double  tiltPID = controlPID(tilt, iTilt, dTilt, kp, kd, ki);

    /****** PID calculation for pan ***********/

//Integral term
    iPan = 0.9*iPan + (pan);
//Derivated term
    double dPan = (pan - previousPan)/((now - timePreviousTilt).sec+0.001*(now - timePreviousTilt).nsec);
//Parameters
    ros::param::get("/qbo_head_tracker/pan/kp", kp);
    ros::param::get("/qbo_head_tracker/pan/kd", kd);
    ros::param::get("/qbo_head_tracker/pan/ki", ki);

    double  panPID = controlPID(pan, iPan, dPan, kp, kd, ki);

//move head
    moveHeadAbs(currentPan + panPID, currentTilt + tiltPID);
ROS_INFO("Moving head from: [x: %lf, y: %lf]", panPID, tiltPID);

//update
    timePreviousTilt = now;
    previousTilt = currentTilt;
    previousPan = currentPan;
}


void moveEyelidsAbs(double left, double right)
{
    sensor_msgs::JointState mvmt_msg;
    std::vector<std::string> names;
    names.push_back("left_eyelid_joint");
    names.push_back("right_eyelid_joint");

    mvmt_msg.name = names;
    std::vector<double> position;
    position.push_back(left);
    position.push_back(right);

    mvmt_msg.position = position;

//   same topic as the head
    move_head->publish(mvmt_msg);
}

/**********
HR_move_head message :
pan
tilt
isAbsolute
***********/
void qboMoveHeadCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    // ROS_INFO("movement order: [pan: %lf, tilt: %lf]", msg->pan, msg->tilt);

    if(msg->z > 0) //absolute move
    {
        moveHeadAbs(msg->x, msg->y);
    }
    else //relative move
    {
        moveHeadRel( msg->x,  msg->y);
    }

}


/**********
HR_move_eyelids message :
left
right
***********/
void qboMoveEyelidsCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    //  ROS_INFO("eyelid order: [left: %lf, right: %lf]", msg->left, msg->right);
    moveEyelidsAbs(msg->x, msg->y);
}


/**********
HR_tracker message :
x = position in width in the image. In range [-1 (left), +1 (right)]
y = position in height in the image. In range [-1 (up), +1 (down)]
distance : distance to object (unused if 0 or less)
***********/
void trackerCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    // ROS_INFO("tracker callback for %s: [x: %lf, y: %lf]", msg->object_name.c_str(), msg->x, msg->y);
if(msg->point.z > 0){
ROS_INFO("Got image position: [x: %lf, y: %lf]", msg->point.x, msg->point.y);
	double currentObjectPosX = msg->point.x;
double currentObjectPosY = msg->point.y;
double objectTargetX = 0.0;
double objectTargetY = 0.0;

    moveHeadRel(-currentObjectPosX+objectTargetX , -objectTargetY + currentObjectPosY );
}

}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "qbo_move_head");
    n = new ros::NodeHandle();

ros::service::waitForService("qbo_arduqbo/test_service", -1);

    initValues();
/****
* Subscribers
****/
    ros::Subscriber tilt_sub = n->subscribe("qbo_arduqbo/head_tilt_joint/state", 10, qboTiltCallback);
    ros::Subscriber pan_sub  =  n->subscribe("qbo_arduqbo/head_pan_joint/state", 10, qboPanCallback);

    ros::Subscriber move_head_sub  = n->subscribe("qbo_move_head", 10, qboMoveHeadCallback);

    ros::Subscriber move_eyelids_sub  = n->subscribe("qbo_move_eyelids", 10, qboMoveEyelidsCallback);

    ros::Subscriber tracker_sub  = n->subscribe("qbo_head_tracker", 10, trackerCallback);

/****
* Publisher
****/
    ros::Publisher temp = n->advertise<sensor_msgs::JointState>("/cmd_joints", 10);
    move_head = &(temp);

   /*ros::Rate loop_rate(1);
    loop_rate.sleep();
    loop_rate.sleep();
    moveHeadAbs(0., 0.);*/
    ROS_INFO("qbo_move_head node ready");
    ros::ServiceServer serviceReady = n->advertiseService("qbo_move_head/isReady", isReadyService);

    ros::spin();
    return 0;
}
