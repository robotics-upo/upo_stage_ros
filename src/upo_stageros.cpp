/*
 *  stageros
 *  Copyright (c) 2008, Willow Garage, Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/**

@mainpage

@htmlinclude manifest.html
**/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <signal.h>


// libstage
#include <stage.hh>

// roscpp
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/MapMetaData.h>
#include <geometry_msgs/Twist.h>
#include <rosgraph_msgs/Clock.h>

#include "tf/transform_broadcaster.h"

//--- Added by Noé -------
#include <upo_msgs/PersonPoseArrayUPO.h>
#include <upo_msgs/PersonPoseUPO.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>

#define PI	3.14159265358979323846
//------------------------

#define USAGE "upo_stageros <worldfile>"
#define IMAGE "image"
#define DEPTH "depth"
#define CAMERA_INFO "camera_info"
#define ODOM "odom"
#define BASE_SCAN "base_scan"
#define BASE_POSE_GROUND_TRUTH "base_pose_ground_truth"
#define CMD_VEL "cmd_vel"

// Our node
class StageNode
{
private:

    // roscpp-related bookkeeping
    ros::NodeHandle n_;

    // A mutex to lock access to fields that are used in message callbacks
    boost::mutex msg_lock;

    // The models that we're interested in
    std::vector<Stg::ModelCamera *> cameramodels;
    std::vector<Stg::ModelRanger *> lasermodels;
    std::vector<Stg::ModelPosition *> positionmodels;
    //-- Added by Noé --------------------------------
    std::vector<Stg::ModelFiducial *> fiducialmodels;
    std::vector<Stg::Model *> blockmodels;
    std::string fiducials_frame;
    tf::TransformListener tf_listener;
    double map_frame_origin_x;
    double map_frame_origin_y;
	bool robot_position_changed;
    //------------------------------------------------

    //a structure representing a robot in the simulator
    struct StageRobot
    {
        //stage related models
        Stg::ModelPosition* positionmodel; //one position
        std::vector<Stg::ModelCamera *> cameramodels; //multiple cameras per position
        std::vector<Stg::ModelRanger *> lasermodels; //multiple rangers per position
	//---Added by Noé--------------------------------
	std::vector<Stg::ModelFiducial *> fiducialmodels;
	//-----------------------------------------------

        //ros publishers
        ros::Publisher odom_pub; //one odom
        ros::Publisher ground_truth_pub; //one ground truth

        std::vector<ros::Publisher> image_pubs; //multiple images
        std::vector<ros::Publisher> depth_pubs; //multiple depths
        std::vector<ros::Publisher> camera_pubs; //multiple cameras
        std::vector<ros::Publisher> laser_pubs; //multiple lasers
	//----Added by Noé---------------------------------
	unsigned int pose_seq;
	std::vector<upo_msgs::PersonPoseArrayUPO> people_auxs;
	std::vector<ros::Publisher> people_pubs;
	std::vector<ros::Publisher> markers_pubs;
	//-------------------------------------------------
        ros::Subscriber cmdvel_sub; //one cmd_vel subscriber
    };

    std::vector<StageRobot*> robotmodels_; //<StageRobot const *>


    ros::Publisher clock_pub_;
    
    bool isDepthCanonical;
    bool use_model_names;

    // A helper function that is executed for each stage model.  We use it
    // to search for models of interest.
    static void ghfunc(Stg::Model* mod, StageNode* node);

    static bool s_update(Stg::World* world, StageNode* node)
    {
        node->WorldCallback();
        // We return false to indicate that we want to be called again (an
        // odd convention, but that's the way that Stage works).
        return false;
    }

    // Appends the given robot ID to the given message name.  If omitRobotID
    // is true, an unaltered copy of the name is returned.
    const char *mapName(const char *name, size_t robotID, Stg::Model* mod) const;
    const char *mapName(const char *name, size_t robotID, size_t deviceID, Stg::Model* mod) const;

    tf::TransformBroadcaster tf;

    // Last time that we received a velocity command
    ros::Time base_last_cmd;
    ros::Duration base_watchdog_timeout;

    // Current simulation time
    ros::Time sim_time;
    
    // Last time we saved global position (for velocity calculation).
    ros::Time base_last_globalpos_time;
    // Last published global pose of each robot
    std::vector<Stg::Pose> base_last_globalpos;

public:
    // Constructor; stage itself needs argc/argv.  fname is the .world file
    // that stage should load.
    StageNode(int argc, char** argv, bool gui, const char* fname, bool use_model_names);
    ~StageNode();

    // Subscribe to models of interest.  Currently, we find and subscribe
    // to the first 'laser' model and the first 'position' model.  Returns
    // 0 on success (both models subscribed), -1 otherwise.
    int SubscribeModels();

    // Our callback
    void WorldCallback();
    
    // Do one update of the world.  May pause if the next update time
    // has not yet arrived.
    bool UpdateWorld();

    // Message callback for a MsgBaseVel message, which set velocities.
    void cmdvelReceived(int idx, const boost::shared_ptr<geometry_msgs::Twist const>& msg);

    //----Added by Noé---------------------------------
    void peoplePoseReceived(const boost::shared_ptr<upo_msgs::PersonPoseArrayUPO const>& msg);
    void mapMetadataReceived(const boost::shared_ptr<nav_msgs::MapMetaData const>& msg);
	void robotPoseReceived(const boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const> &msg);
    std::vector<upo_msgs::PersonPoseArrayUPO> people_auxs;
    std::vector<ros::Publisher> people_pubs;
    std::vector<ros::Publisher> markers_pubs;
    ros::Subscriber people_pose_sub_;
    ros::Subscriber map_metadata_sub_;
	//Subscription to "initial pose" topic to change the position of the robot
	//as AMCL does it.	
	ros::Subscriber change_robot_pose_sub_;
    //-------------------------------------------------

    // The main simulator object
    Stg::World* world;
};

// since stageros is single-threaded, this is OK. revisit if that changes!
const char *
StageNode::mapName(const char *name, size_t robotID, Stg::Model* mod) const
{
    //ROS_INFO("Robot %lu: Device %s", robotID, name);
    bool umn = this->use_model_names;

    if ((positionmodels.size() > 1 ) || umn)
    {
        static char buf[100];
        std::size_t found = std::string(((Stg::Ancestor *) mod)->Token()).find(":");

        if ((found==std::string::npos) && umn)
        {
            snprintf(buf, sizeof(buf), "/%s/%s", ((Stg::Ancestor *) mod)->Token(), name);
        }
        else
        {
            snprintf(buf, sizeof(buf), "/robot_%u/%s", (unsigned int)robotID, name);
        }

        return buf;
    }
    else
        return name;
}

const char *
StageNode::mapName(const char *name, size_t robotID, size_t deviceID, Stg::Model* mod) const
{
    //ROS_INFO("Robot %lu: Device %s:%lu", robotID, name, deviceID);
    bool umn = this->use_model_names;

    if ((positionmodels.size() > 1 ) || umn)
    {
        static char buf[100];
        std::size_t found = std::string(((Stg::Ancestor *) mod)->Token()).find(":");

        if ((found==std::string::npos) && umn)
        {
            snprintf(buf, sizeof(buf), "/%s/%s_%u", ((Stg::Ancestor *) mod)->Token(), name, (unsigned int)deviceID);
        }
        else
        {
            snprintf(buf, sizeof(buf), "/robot_%u/%s_%u", (unsigned int)robotID, name, (unsigned int)deviceID);
        }

        return buf;
    }
    else
    {
        static char buf[100];
        snprintf(buf, sizeof(buf), "/%s_%u", name, (unsigned int)deviceID);
        return buf;
    }
}

void
StageNode::ghfunc(Stg::Model* mod, StageNode* node)
{
    if (dynamic_cast<Stg::ModelRanger *>(mod))
	node->lasermodels.push_back(dynamic_cast<Stg::ModelRanger *>(mod));
    else if (dynamic_cast<Stg::ModelPosition *>(mod))
        node->positionmodels.push_back(dynamic_cast<Stg::ModelPosition *>(mod));
    else if (dynamic_cast<Stg::ModelCamera *>(mod))
        node->cameramodels.push_back(dynamic_cast<Stg::ModelCamera *>(mod));

    //-- Added by Noé ---------------------------------------
    else if (dynamic_cast<Stg::ModelFiducial *>(mod))
    	node->fiducialmodels.push_back(dynamic_cast<Stg::ModelFiducial *>(mod));

    else if (mod->GetFiducialReturn())
    {
    	node->blockmodels.push_back(mod);
	ROS_INFO("Block found. Assuming it is person %lu", node->blockmodels.size()-1);
    }
    //--------------------------------------------------------
}

void
StageNode::cmdvelReceived(int idx, const boost::shared_ptr<geometry_msgs::Twist const>& msg)
{
    boost::mutex::scoped_lock lock(msg_lock);
    this->positionmodels[idx]->SetSpeed(msg->linear.x,
                                        msg->linear.y,
                                        msg->angular.z);
    this->base_last_cmd = this->sim_time;
}

//---Added by Jesús and Noé------------------------------------------
void
StageNode::peoplePoseReceived(const boost::shared_ptr<upo_msgs::PersonPoseArrayUPO const>& msg)
{
    boost::mutex::scoped_lock lock(msg_lock);

	std::vector<geometry_msgs::PoseStamped> poses;

	//Tranform to map coordinates if necessary
	geometry_msgs::PoseStamped pose;
	
	for(int k=0; k < msg->size; k++)
	{
		upo_msgs::PersonPoseUPO p = msg->personPoses[k];
		pose.header = p.header;
		pose.header.stamp = ros::Time();
		pose.pose.position = p.position;
		pose.pose.orientation = p.orientation;

		if(msg->header.frame_id.compare("map") != 0 && msg->header.frame_id.compare("/map") != 0)
		{
			geometry_msgs::PoseStamped pose_out;
			try {
				tf_listener.transformPose("map", pose, pose_out);
			}catch (tf::TransformException ex){
				ROS_WARN("upo_stage_ros. PeoplePoseReceived. TransformException: %s",ex.what());
			}
			pose = pose_out;
		}
		poses.push_back(pose);
	}
    
    for(int i = 0; i < this->blockmodels.size(); i++)
    {
		if(msg->size > i)
		{
			this->blockmodels[i]->SetColor(Stg::Color(std::string("purple")));
			this->blockmodels[i]->SetRangerReturn(0.5);
			this->blockmodels[i]->SetObstacleReturn(1.0);
			this->blockmodels[i]->SetFiducialKey(0);
			this->blockmodels[i]->SetFiducialReturn(i+1);
			Stg::Pose p_pose;
			p_pose.x = poses.at(i).pose.position.x - map_frame_origin_x;
			p_pose.y = poses.at(i).pose.position.y - map_frame_origin_y;
			p_pose.a = tf::getYaw(poses.at(i).pose.orientation);
			this->blockmodels[i]->SetGlobalPose(p_pose);

		} else {
			//Color(double r, double g, double b, double a=1.0)
			this->blockmodels[i]->SetColor(Stg::Color(0.0, 0.5, 0.5, 0.2));
			this->blockmodels[i]->SetRangerReturn(-1.0);
			this->blockmodels[i]->SetObstacleReturn(-1.0);
			//this->blockmodels[i]->SetFiducialReturn(-1);
			this->blockmodels[i]->SetFiducialKey(2);
			Stg::Pose p_pose;
			p_pose.x = 0.0;
			p_pose.y = 0.0;
			p_pose.a = 0.0;
			this->blockmodels[i]->SetGlobalPose(p_pose);
		}
    }
}

void
StageNode::robotPoseReceived(const boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const>& msg)
{
    boost::mutex::scoped_lock lock(msg_lock);
   
	Stg::Pose p_pose;// p_pose.x .y .z .a
	p_pose.x = msg->pose.pose.position.x - map_frame_origin_x;
	p_pose.y = msg->pose.pose.position.y - map_frame_origin_y;
	p_pose.z = 0.0;
	geometry_msgs::Quaternion q = msg->pose.pose.orientation;
	p_pose.a = tf::getYaw(q); //*180.0/PI; //transform to degrees
	ROS_INFO("Stage. Changing robot position to x:%.2f, y:%.2f, a:%.2f\n", p_pose.x, p_pose.y, p_pose.a);
	//this will move only the first robot in case we have more than one
	this->positionmodels[0]->SetGlobalPose(p_pose);

	//Parameters used to reset the odometry values.
	robot_position_changed = true;
	
	
}

void
StageNode::mapMetadataReceived(const boost::shared_ptr<nav_msgs::MapMetaData const>& msg)
{
    boost::mutex::scoped_lock lock(msg_lock);
    
    map_frame_origin_x = msg->origin.position.x;
    map_frame_origin_y = msg->origin.position.y;
}
//------------------------------------------------------------

StageNode::StageNode(int argc, char** argv, bool gui, const char* fname, bool use_model_names)
{
    this->use_model_names = use_model_names;
    this->sim_time.fromSec(0.0);
    this->base_last_cmd.fromSec(0.0);
    double t;
    ros::NodeHandle localn("~");
    if(!localn.getParam("base_watchdog_timeout", t))
        t = 0.2;
    this->base_watchdog_timeout.fromSec(t);

    if(!localn.getParam("is_depth_canonical", isDepthCanonical))
        isDepthCanonical = true;

    //---Added by Noé -----------------------------------------
    if(!localn.getParam("fiducials_frame", fiducials_frame))
	fiducials_frame = "map";

    map_frame_origin_x = 0.0;
    map_frame_origin_y = 0.0;
	robot_position_changed = false;
    //----------------------------------------------------------


    // We'll check the existence of the world file, because libstage doesn't
    // expose its failure to open it.  Could go further with checks (e.g., is
    // it readable by this user).
    struct stat s;
    if(stat(fname, &s) != 0)
    {
        ROS_FATAL("The world file %s does not exist.", fname);
        ROS_BREAK();
    }

    // initialize libstage
    Stg::Init( &argc, &argv );

    if(gui)
        this->world = new Stg::WorldGui(600, 400, "UPO Stage (ROS)");
    else
        this->world = new Stg::World();

    // Apparently an Update is needed before the Load to avoid crashes on
    // startup on some systems.
    // As of Stage 4.1.1, this update call causes a hang on start.
    //this->UpdateWorld();
    this->world->Load(fname);

    // We add our callback here, after the Update, so avoid our callback
    // being invoked before we're ready.
    this->world->AddUpdateCallback((Stg::world_callback_t)s_update, this);

    this->world->ForEachDescendant((Stg::model_callback_t)ghfunc, this);
}


// Subscribe to models of interest.  Currently, we find and subscribe
// to the first 'laser' model and the first 'position' model.  Returns
// 0 on success (both models subscribed), -1 otherwise.
//
// Eventually, we should provide a general way to map stage models onto ROS
// topics, similar to Player .cfg files.
int
StageNode::SubscribeModels()
{
    n_.setParam("/use_sim_time", true);

    for (size_t r = 0; r < this->positionmodels.size(); r++)
    {
        StageRobot* new_robot = new StageRobot;
        new_robot->positionmodel = this->positionmodels[r];
        new_robot->positionmodel->Subscribe();


        for (size_t s = 0; s < this->lasermodels.size(); s++)
        {
            if (this->lasermodels[s] and this->lasermodels[s]->Parent() == new_robot->positionmodel)
            {
                new_robot->lasermodels.push_back(this->lasermodels[s]);
                this->lasermodels[s]->Subscribe();
            }
        }

        for (size_t s = 0; s < this->cameramodels.size(); s++)
        {
            if (this->cameramodels[s] and this->cameramodels[s]->Parent() == new_robot->positionmodel)
            {
                new_robot->cameramodels.push_back(this->cameramodels[s]);
                this->cameramodels[s]->Subscribe();
            }
        }

	//---Added by Noé ------------------------------------
 	for (size_t s = 0; s < this->fiducialmodels.size(); s++)
        {
            if (this->fiducialmodels[s] and this->fiducialmodels[s]->Parent() == new_robot->positionmodel)
            {
                new_robot->fiducialmodels.push_back(this->fiducialmodels[s]);
                this->fiducialmodels[s]->Subscribe();
            }
        }
	//------------------------------------------------------

        ROS_INFO("Found %lu laser devices and %lu cameras in robot %lu", new_robot->lasermodels.size(), new_robot->cameramodels.size(), r);

        //----Added by Noé----------------------------------
	if(new_robot->fiducialmodels.size() > 0)
		ROS_INFO("Found %lu fiducial devices in robot %lu", new_robot->fiducialmodels.size(), r);

	new_robot->pose_seq = 0;
	//--------------------------------------------------

        new_robot->odom_pub = n_.advertise<nav_msgs::Odometry>(mapName(ODOM, r, static_cast<Stg::Model*>(new_robot->positionmodel)), 10);
        new_robot->ground_truth_pub = n_.advertise<nav_msgs::Odometry>(mapName(BASE_POSE_GROUND_TRUTH, r, static_cast<Stg::Model*>(new_robot->positionmodel)), 10);
        new_robot->cmdvel_sub = n_.subscribe<geometry_msgs::Twist>(mapName(CMD_VEL, r, static_cast<Stg::Model*>(new_robot->positionmodel)), 10, boost::bind(&StageNode::cmdvelReceived, this, r, _1));

        for (size_t s = 0;  s < new_robot->lasermodels.size(); ++s)
        {
            if (new_robot->lasermodels.size() == 1)
                new_robot->laser_pubs.push_back(n_.advertise<sensor_msgs::LaserScan>(mapName(BASE_SCAN, r, static_cast<Stg::Model*>(new_robot->positionmodel)), 10));
            else
                new_robot->laser_pubs.push_back(n_.advertise<sensor_msgs::LaserScan>(mapName(BASE_SCAN, r, s, static_cast<Stg::Model*>(new_robot->positionmodel)), 10));

        }

 	//---Added by Noé--------------------------------------
	for (size_t s = 0;  s < new_robot->fiducialmodels.size(); ++s)
        {
            if (new_robot->fiducialmodels.size() == 1) {										///pdt_node/people_pose
				new_robot->people_pubs.push_back(n_.advertise<upo_msgs::PersonPoseArrayUPO>(mapName("/people/navigation", r, static_cast<Stg::Model*>(new_robot->positionmodel)), 1));                                                                      ///pdt_node/people_markers
				new_robot->markers_pubs.push_back(n_.advertise<visualization_msgs::MarkerArray>(mapName("/people/people_markers", r, static_cast<Stg::Model*>(new_robot->positionmodel)), 1));
            } else {
                new_robot->people_pubs.push_back(n_.advertise<upo_msgs::PersonPoseArrayUPO>(mapName("/people/navigation", r, s, static_cast<Stg::Model*>(new_robot->positionmodel)), 1));
				new_robot->markers_pubs.push_back(n_.advertise<visualization_msgs::MarkerArray>(mapName("/people/people_markers", r, s, static_cast<Stg::Model*>(new_robot->positionmodel)), 1));
			}
			upo_msgs::PersonPoseArrayUPO ppa;
			new_robot->people_auxs.push_back(ppa);
        }
	//-------------------------------------------------------

        for (size_t s = 0;  s < new_robot->cameramodels.size(); ++s)
        {
            if (new_robot->cameramodels.size() == 1)
            {
                new_robot->image_pubs.push_back(n_.advertise<sensor_msgs::Image>(mapName(IMAGE, r, static_cast<Stg::Model*>(new_robot->positionmodel)), 10));
                new_robot->depth_pubs.push_back(n_.advertise<sensor_msgs::Image>(mapName(DEPTH, r, static_cast<Stg::Model*>(new_robot->positionmodel)), 10));
                new_robot->camera_pubs.push_back(n_.advertise<sensor_msgs::CameraInfo>(mapName(CAMERA_INFO, r, static_cast<Stg::Model*>(new_robot->positionmodel)), 10));
            }
            else
            {
                new_robot->image_pubs.push_back(n_.advertise<sensor_msgs::Image>(mapName(IMAGE, r, s, static_cast<Stg::Model*>(new_robot->positionmodel)), 10));
                new_robot->depth_pubs.push_back(n_.advertise<sensor_msgs::Image>(mapName(DEPTH, r, s, static_cast<Stg::Model*>(new_robot->positionmodel)), 10));
                new_robot->camera_pubs.push_back(n_.advertise<sensor_msgs::CameraInfo>(mapName(CAMERA_INFO, r, s, static_cast<Stg::Model*>(new_robot->positionmodel)), 10));
            }
        }

        this->robotmodels_.push_back(new_robot);
    }
    clock_pub_ = n_.advertise<rosgraph_msgs::Clock>("/clock", 10);

    //---Added by Noé--------------------------------------
    people_pose_sub_ = n_.subscribe<upo_msgs::PersonPoseArrayUPO>("people_pose", 1, &StageNode::peoplePoseReceived, this);
    map_metadata_sub_ = n_.subscribe<nav_msgs::MapMetaData>("map_metadata", 1, &StageNode::mapMetadataReceived, this);
	change_robot_pose_sub_ = n_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1, &StageNode::robotPoseReceived, this);
    //-------------------------------------------------------
    return(0);
}

StageNode::~StageNode()
{                 //<StageRobot const*>   
    for (std::vector<StageRobot *>::iterator r = this->robotmodels_.begin(); r != this->robotmodels_.end(); ++r)
        delete *r;
}

bool
StageNode::UpdateWorld()
{
    return this->world->UpdateAll();
}

void
StageNode::WorldCallback()
{
    boost::mutex::scoped_lock lock(msg_lock);

    this->sim_time.fromSec(world->SimTimeNow() / 1e6);
    // We're not allowed to publish clock==0, because it used as a special
    // value in parts of ROS, #4027.
    if(this->sim_time.sec == 0 && this->sim_time.nsec == 0)
    {
        ROS_DEBUG("Skipping initial simulation step, to avoid publishing clock==0");
        return;
    }

    // TODO make this only affect one robot if necessary
    if((this->base_watchdog_timeout.toSec() > 0.0) &&
            ((this->sim_time - this->base_last_cmd) >= this->base_watchdog_timeout))
    {
        for (size_t r = 0; r < this->positionmodels.size(); r++)
            this->positionmodels[r]->SetSpeed(0.0, 0.0, 0.0);
    }

    //-- Added by Noé ----------------------------------------------
    tf::TransformBroadcaster tf_broadcaster;
    //----------------------------------------------------------------

    //loop on the robot models
    for (size_t r = 0; r < this->robotmodels_.size(); ++r)
    {
        StageRobot* robotmodel = this->robotmodels_[r]; //StageRobot const *

		//--- Added by Noé ------------------------
		if(robot_position_changed) {
			Stg::Pose zeropose;
			zeropose.x = 0.0;
			zeropose.y = 0.0;
			zeropose.z = 0.0;
			zeropose.a = 0.0;
			robotmodel->positionmodel->SetOdom(zeropose);
			robot_position_changed = false;
		}
		//-----------------------------------------

        //loop on the laser devices for the current robot
        for (size_t s = 0; s < robotmodel->lasermodels.size(); ++s)
        {

			

            Stg::ModelRanger const* lasermodel = robotmodel->lasermodels[s];
            const std::vector<Stg::ModelRanger::Sensor>& sensors = lasermodel->GetSensors();

            if( sensors.size() > 1 )
                ROS_WARN( "ROS Stage currently supports rangers with 1 sensor only." );

            // for now we access only the zeroth sensor of the ranger - good
            // enough for most laser models that have a single beam origin
            const Stg::ModelRanger::Sensor& sensor = sensors[0];

            if( sensor.ranges.size() )
            {
                // Translate into ROS message format and publish
                sensor_msgs::LaserScan msg;
                msg.angle_min = -sensor.fov/2.0;
                msg.angle_max = +sensor.fov/2.0;
                msg.angle_increment = sensor.fov/(double)(sensor.sample_count-1);
                msg.range_min = sensor.range.min;
                msg.range_max = sensor.range.max;
                msg.ranges.resize(sensor.ranges.size());
                msg.intensities.resize(sensor.intensities.size());

                for(unsigned int i = 0; i < sensor.ranges.size(); i++)
                {
                    msg.ranges[i] = sensor.ranges[i];
                    msg.intensities[i] = (uint8_t)sensor.intensities[i];
                }

                if (robotmodel->lasermodels.size() > 1)
                    msg.header.frame_id = mapName("base_laser_link", r, s, static_cast<Stg::Model*>(robotmodel->positionmodel));
                else
                    msg.header.frame_id = mapName("base_laser_link", r, static_cast<Stg::Model*>(robotmodel->positionmodel));

                msg.header.stamp = sim_time;
                robotmodel->laser_pubs[s].publish(msg);
            }

            // Also publish the base->base_laser_link Tx.  This could eventually move
            // into being retrieved from the param server as a static Tx.
            Stg::Pose lp = lasermodel->GetPose();
            tf::Quaternion laserQ;
            laserQ.setRPY(0.0, 0.0, lp.a);
            tf::Transform txLaser =  tf::Transform(laserQ, tf::Point(lp.x, lp.y, robotmodel->positionmodel->GetGeom().size.z + lp.z));

            if (robotmodel->lasermodels.size() > 1)
                tf.sendTransform(tf::StampedTransform(txLaser, sim_time,
                                                      mapName("base_link", r, static_cast<Stg::Model*>(robotmodel->positionmodel)),
                                                      mapName("base_laser_link", r, s, static_cast<Stg::Model*>(robotmodel->positionmodel))));
            else
                tf.sendTransform(tf::StampedTransform(txLaser, sim_time,
                                                      mapName("base_link", r, static_cast<Stg::Model*>(robotmodel->positionmodel)),
                                                      mapName("base_laser_link", r, static_cast<Stg::Model*>(robotmodel->positionmodel))));
        }



        //the position of the robot
        tf.sendTransform(tf::StampedTransform(tf::Transform::getIdentity(),
                                              sim_time,
                                              mapName("base_footprint", r, static_cast<Stg::Model*>(robotmodel->positionmodel)),
                                              mapName("base_link", r, static_cast<Stg::Model*>(robotmodel->positionmodel))));

        // Get latest odometry data
        // Translate into ROS message format and publish
        nav_msgs::Odometry odom_msg;
        odom_msg.pose.pose.position.x = robotmodel->positionmodel->est_pose.x;
        odom_msg.pose.pose.position.y = robotmodel->positionmodel->est_pose.y;
        odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(robotmodel->positionmodel->est_pose.a);
		        
		Stg::Velocity v = robotmodel->positionmodel->GetVelocity();
        odom_msg.twist.twist.linear.x = v.x;
        odom_msg.twist.twist.linear.y = v.y;
        odom_msg.twist.twist.angular.z = v.a;

        //@todo Publish stall on a separate topic when one becomes available
        //this->odomMsgs[r].stall = this->positionmodels[r]->Stall();
        //
        odom_msg.header.frame_id = mapName("odom", r, static_cast<Stg::Model*>(robotmodel->positionmodel));
        odom_msg.header.stamp = sim_time;

        robotmodel->odom_pub.publish(odom_msg);

        // broadcast odometry transform
        tf::Quaternion odomQ;
        tf::quaternionMsgToTF(odom_msg.pose.pose.orientation, odomQ);
        tf::Transform txOdom(odomQ, tf::Point(odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, 0.0));
        tf.sendTransform(tf::StampedTransform(txOdom, sim_time,
                                              mapName("odom", r, static_cast<Stg::Model*>(robotmodel->positionmodel)),
                                              mapName("base_footprint", r, static_cast<Stg::Model*>(robotmodel->positionmodel))));

        // Also publish the ground truth pose and velocity
        Stg::Pose gpose = robotmodel->positionmodel->GetGlobalPose();
        tf::Quaternion q_gpose;
        q_gpose.setRPY(0.0, 0.0, gpose.a);
        tf::Transform gt(q_gpose, tf::Point(gpose.x, gpose.y, 0.0));
        // Velocity is 0 by default and will be set only if there is previous pose and time delta>0
        Stg::Velocity gvel(0,0,0,0);
        if (this->base_last_globalpos.size()>r){
            Stg::Pose prevpose = this->base_last_globalpos.at(r);
            double dT = (this->sim_time-this->base_last_globalpos_time).toSec();
            if (dT>0)
                gvel = Stg::Velocity(
                            (gpose.x - prevpose.x)/dT,
                            (gpose.y - prevpose.y)/dT,
                            (gpose.z - prevpose.z)/dT,
                            Stg::normalize(gpose.a - prevpose.a)/dT
                            );
            this->base_last_globalpos.at(r) = gpose;
        }else //There are no previous readings, adding current pose...
            this->base_last_globalpos.push_back(gpose);

        nav_msgs::Odometry ground_truth_msg;
        ground_truth_msg.pose.pose.position.x     = gt.getOrigin().x();
        ground_truth_msg.pose.pose.position.y     = gt.getOrigin().y();
        ground_truth_msg.pose.pose.position.z     = gt.getOrigin().z();
        ground_truth_msg.pose.pose.orientation.x  = gt.getRotation().x();
        ground_truth_msg.pose.pose.orientation.y  = gt.getRotation().y();
        ground_truth_msg.pose.pose.orientation.z  = gt.getRotation().z();
        ground_truth_msg.pose.pose.orientation.w  = gt.getRotation().w();
        ground_truth_msg.twist.twist.linear.x = gvel.x;
        ground_truth_msg.twist.twist.linear.y = gvel.y;
        ground_truth_msg.twist.twist.linear.z = gvel.z;
        ground_truth_msg.twist.twist.angular.z = gvel.a;

        ground_truth_msg.header.frame_id = mapName("odom", r, static_cast<Stg::Model*>(robotmodel->positionmodel));
        ground_truth_msg.header.stamp = sim_time;

        robotmodel->ground_truth_pub.publish(ground_truth_msg);

		
	//---Added by Noé -------------------------------------
	//fiducials. Loop on the fiducials models of each robot. 
	for (size_t s = 0; s < robotmodel->fiducialmodels.size(); ++s)
        {

            Stg::ModelFiducial * fiducialmodel = robotmodel->fiducialmodels[s];
	    const std::vector<Stg::ModelFiducial::Fiducial>& fids = fiducialmodel->GetFiducials();

	    visualization_msgs::MarkerArray mArray;	
	    upo_msgs::PersonPoseArrayUPO poseArr;
	    robotmodel->pose_seq = robotmodel->pose_seq + 1;
	    poseArr.header.seq = robotmodel->pose_seq;
	    poseArr.header.stamp = sim_time;
	    poseArr.size = fids.size();
	    visualization_msgs::Marker marker;

	    //publish people in other frame different from map frame
	    if(fiducials_frame.compare(std::string("map")) != 0 && fiducials_frame.compare(std::string("/map")) != 0)  
	    {
		for(unsigned int i=0; i<fids.size(); i++)
		{
			// publish TF first (local coordinates)
			geometry_msgs::TransformStamped msg;

			int id = fids[i].id;
			char child_frame_no[128];
    			snprintf(child_frame_no, sizeof(child_frame_no), "fiducial_%d", id);
			msg.header.stamp = sim_time;

			if (robotmodel->fiducialmodels.size() > 1) {
				msg.child_frame_id = mapName(child_frame_no, r, s, static_cast<Stg::Model*>(robotmodel->positionmodel));
				msg.header.frame_id = mapName("base_fiducial_link", r, s, static_cast<Stg::Model*>(robotmodel->positionmodel));
                	} else {
				msg.child_frame_id = mapName(child_frame_no, r, static_cast<Stg::Model*>(robotmodel->positionmodel));
				msg.header.frame_id = mapName("base_fiducial_link", r, static_cast<Stg::Model*>(robotmodel->positionmodel));
			}

			Stg::Pose location = fiducialmodel->GlobalToLocal(fids[i].pose);
			msg.transform.translation.x = location.x; 
			msg.transform.translation.y = location.y;
			msg.transform.translation.z = location.z;
			msg.transform.rotation = tf::createQuaternionMsgFromYaw(location.a);
				
			tf_broadcaster.sendTransform(msg);			
	
			// then, publish the people array in the topic /pdt_node/people_pose
			// and in the frame indicated by fiducials_frame
			poseArr.header.frame_id = fiducials_frame;

			double x= 0.0, y=0.0, h=0.0;
			bool correct = false;
			tf::StampedTransform transform;
    			try{
      				tf_listener.lookupTransform(fiducials_frame, msg.child_frame_id,  
                        		ros::Time(0), transform);
				correct = true;
    			}
    			catch (tf::TransformException ex){
      				ROS_WARN("%s",ex.what());
    			}
			if(correct) {
				x = (float)transform.getOrigin().getX();
				y = (float)transform.getOrigin().getY();
				h = (float)tf::getYaw(transform.getRotation());
			}

			upo_msgs::PersonPoseUPO pose;
			pose.header.stamp = poseArr.header.stamp;
			pose.header.frame_id = poseArr.header.frame_id;
			pose.id = fids[i].id;
			pose.position.x = x;
			pose.position.y = y;
			pose.position.z = 1.0; //location.z;
			pose.orientation = tf::createQuaternionMsgFromYaw(h);

			//calculate the module of the velocity
			bool found = false;
			if(robotmodel->people_auxs[s].personPoses.size() > 0) 
			{
				for(int j=0; j < robotmodel->people_auxs[s].personPoses.size() && found==false; j++)
				{ 
					if(robotmodel->people_auxs[s].personPoses.at(j).id == pose.id) {
						double dist = sqrt(pow(pose.position.x-robotmodel->people_auxs[s].personPoses.at(j).position.x,2) + pow(pose.position.y-robotmodel->people_auxs[s].personPoses.at(j).position.y,2));
						double time = (pose.header.stamp - robotmodel->people_auxs[s].header.stamp).toSec();
						if(time > 0.0) {
							pose.vel = dist/time;
							found = true;
							//ROS_INFO("DIST: %.2f, TIME: %.2f, vel: %.2f",dist, time, pose.vel);
						}
					}
				}
			}
			if(!found) {
				pose.vel = 0.0;
			}

			poseArr.personPoses.push_back(pose);

			//Markers
			// Fill up arrow marker information
			marker.header.frame_id = poseArr.header.frame_id;
			marker.header.stamp = sim_time; 
			marker.ns = "people_pose";
			marker.id = fids[i].id;
			marker.type = visualization_msgs::Marker::ARROW;
			marker.action = visualization_msgs::Marker::ADD;
			marker.pose.position.x = x;
			marker.pose.position.y = y;
			marker.pose.position.z = 1.0; //location.z;
			marker.pose.orientation.x = 0.0;
			marker.pose.orientation.y = 0.0;
			marker.pose.orientation.z = sin(h/2.0);
			marker.pose.orientation.w = cos(h/2.0);
			marker.scale.x = 0.7; //0.5;
			marker.scale.y = 0.2; //1.0;
			marker.scale.z = 0.5; //0.8;
			marker.color.a = 1.0;
			marker.color.r = 1.0*(fids[i].id%5)/5;
			marker.color.g = 1.0*(fids[i].id%50)/50;
			marker.color.b = 1.0*(fids[i].id%10)/10;
			marker.lifetime = ros::Duration(0.1);
			mArray.markers.push_back(marker);
				
			// Fill up text marker information
			char text[256];
			sprintf(text, "%d", fids[i].id);
			marker.header.frame_id = poseArr.header.frame_id;
			marker.header.stamp = sim_time;
			marker.ns = "people_text";
			marker.id = fids[i].id;
			marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
			marker.action = visualization_msgs::Marker::ADD;
			marker.pose.position.x = x;
			marker.pose.position.y = y;
			marker.pose.position.z = 1.0+0.3;
			marker.pose.orientation.x = 0.0;
			marker.pose.orientation.y = 0.0;
			marker.pose.orientation.z = sin(h/2.0);
			marker.pose.orientation.w = cos(h/2.0);
			marker.scale.z = 0.5;
			marker.color.a = 1.0;
			marker.color.r = 1.0; //*(people[i].id%5)/5;
			marker.color.g = 0.0; //1.0*(people[i].id%50)/50;
			marker.color.b = 0.0; //1.0*(people[i].id%10)/10;
			marker.text = text;
			marker.lifetime = ros::Duration(0.1);
			mArray.markers.push_back(marker);
					
		} //end of for loop fids.size()

				 
	    } else {  //if the frame is map

		for(unsigned int i=0; i<fids.size(); i++)
		{
			// publish TF first (map coordinates)
			geometry_msgs::TransformStamped msg;

			int id = fids[i].id;
			char child_frame_no[128];
    			snprintf(child_frame_no, sizeof(child_frame_no), "fiducial_%d", id);
			msg.header.frame_id = "map";
			msg.header.stamp = sim_time;

			if (robotmodel->fiducialmodels.size() > 1) {
				msg.child_frame_id = mapName(child_frame_no, r, s, static_cast<Stg::Model*>(robotmodel->positionmodel));
                	} else {
				msg.child_frame_id = mapName(child_frame_no, r, static_cast<Stg::Model*>(robotmodel->positionmodel));
			}
				
			msg.transform.translation.x = fids[i].pose.x; 
			msg.transform.translation.y = fids[i].pose.y;
			msg.transform.translation.z = fids[i].pose.z;
			msg.transform.rotation = tf::createQuaternionMsgFromYaw(fids[i].pose.a);
				
			tf_broadcaster.sendTransform(msg);

			// then, publish the people array in the topic /pdt_node/people_pose
			// and in the map frame
			poseArr.header.frame_id = "map";

			upo_msgs::PersonPoseUPO pose;
			pose.header.stamp = poseArr.header.stamp;
			pose.header.frame_id = poseArr.header.frame_id;
			pose.id = fids[i].id;
			pose.position.x = fids[i].pose.x;
			pose.position.y = fids[i].pose.y;
			pose.position.z = 1.0; //location.z;
			pose.orientation = tf::createQuaternionMsgFromYaw(fids[i].pose.a);

			//calculate the module of the velocity
			bool found = false;
			if(robotmodel->people_auxs[s].personPoses.size() > 0) 
			{
				for(int j=0; j < robotmodel->people_auxs[s].personPoses.size() && found==false; j++)
				{ 
					if(robotmodel->people_auxs[s].personPoses.at(j).id == pose.id) {
						double dist = sqrt(pow(pose.position.x-robotmodel->people_auxs[s].personPoses.at(j).position.x,2) + pow(pose.position.y-robotmodel->people_auxs[s].personPoses.at(j).position.y,2));
						double time = (pose.header.stamp - robotmodel->people_auxs[s].header.stamp).toSec();
						if(time > 0.0) {
							pose.vel = dist/time;
							found = true;
							//ROS_INFO("DIST: %.2f, TIME: %.2f, vel: %.2f",dist, time, pose.vel);
						}
					}
				}
			}
			if(!found) {
				pose.vel = 0.0;
			}

			poseArr.personPoses.push_back(pose);

			//Markers
			// Fill up arrow marker information
			marker.header.frame_id = poseArr.header.frame_id;
			marker.header.stamp = sim_time; 
			marker.ns = "people_pose";
			marker.id = fids[i].id;
			marker.type = visualization_msgs::Marker::ARROW;
			marker.action = visualization_msgs::Marker::ADD;
			marker.pose.position.x = fids[i].pose.x;
			marker.pose.position.y = fids[i].pose.y;
			marker.pose.position.z = 1.0; //location.z;
			marker.pose.orientation.x = 0.0;
			marker.pose.orientation.y = 0.0;
			marker.pose.orientation.z = sin(fids[i].pose.a/2.0);
			marker.pose.orientation.w = cos(fids[i].pose.a/2.0);
			marker.scale.x = 0.7; //0.5;
			marker.scale.y = 0.2; //1.0;
			marker.scale.z = 0.5; //0.8;
			marker.color.a = 1.0;
			marker.color.r = 1.0*(fids[i].id%5)/5;
			marker.color.g = 1.0*(fids[i].id%50)/50;
			marker.color.b = 1.0*(fids[i].id%10)/10;
			marker.lifetime = ros::Duration(0.1);
			mArray.markers.push_back(marker);
				
			// Fill up text marker information
			char text[256];
			sprintf(text, "%d", fids[i].id);
			marker.header.frame_id = poseArr.header.frame_id;
			marker.header.stamp = sim_time;
			marker.ns = "people_text";
			marker.id = fids[i].id;
			marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
			marker.action = visualization_msgs::Marker::ADD;
			marker.pose.position.x = fids[i].pose.x;
			marker.pose.position.y = fids[i].pose.y;
			marker.pose.position.z = 1.0+0.3;
			marker.pose.orientation.x = 0.0;
			marker.pose.orientation.y = 0.0;
			marker.pose.orientation.z = sin(fids[i].pose.a/2.0);
			marker.pose.orientation.w = cos(fids[i].pose.a/2.0);
			marker.scale.z = 0.5;
			marker.color.a = 1.0;
			marker.color.r = 1.0; //*(people[i].id%5)/5;
			marker.color.g = 0.0; //1.0*(people[i].id%50)/50;
			marker.color.b = 0.0; //1.0*(people[i].id%10)/10;
			marker.text = text;
			marker.lifetime = ros::Duration(0.1);
			mArray.markers.push_back(marker);
								
		} //end of for loop fids.size()

			
	    } //end of else clause (if the frame is map)


	    //we keep the last people array in order to calculate the velocity
	    // in the next iteration
	    std::vector<upo_msgs::PersonPoseArrayUPO> aux;
	    for(unsigned int a=0; a<robotmodel->people_auxs.size(); a++)
	    {
		if(a == s)
			aux.push_back(poseArr);
		else	
			aux.push_back(robotmodel->people_auxs.at(a));
	    }
	    robotmodel->people_auxs = aux;
 
	    //publish the people and the markers
	    //if(poseArr.size) //publish only when size>0
		robotmodel->people_pubs[s].publish(poseArr);

	    if(mArray.markers.size())
		robotmodel->markers_pubs[s].publish(mArray);


	    // publish the tf base_link->base_fiducial_link
	    Stg::Pose fidu = fiducialmodel->GetPose();
	    tf::Quaternion fiduQ;
	    fiduQ.setRPY(0.0, 0.0, fidu.a);
	    tf::Transform txFiducial =  tf::Transform(fiduQ, tf::Point(fidu.x, fidu.y, 0.15));
	    if (robotmodel->fiducialmodels.size() > 1) {
		tf.sendTransform(tf::StampedTransform(txFiducial, sim_time,
			mapName("base_link", r, s, static_cast<Stg::Model*>(robotmodel->positionmodel)),
			mapName("base_fiducial_link", r, s, static_cast<Stg::Model*>(robotmodel->positionmodel))));
	    } else {
		tf.sendTransform(tf::StampedTransform(txFiducial, sim_time,
			mapName("base_link", r, static_cast<Stg::Model*>(robotmodel->positionmodel)),
			mapName("base_fiducial_link", r, static_cast<Stg::Model*>(robotmodel->positionmodel))));
	    }

	} //end for each fiducial model
	//-----------------------------------------------------


        //cameras
        for (size_t s = 0; s < robotmodel->cameramodels.size(); ++s)
        {
            Stg::ModelCamera* cameramodel = robotmodel->cameramodels[s];
            // Get latest image data
            // Translate into ROS message format and publish
            if (robotmodel->image_pubs[s].getNumSubscribers() > 0 && cameramodel->FrameColor())
            {
                sensor_msgs::Image image_msg;

                image_msg.height = cameramodel->getHeight();
                image_msg.width = cameramodel->getWidth();
                image_msg.encoding = "rgba8";
                //this->imageMsgs[r].is_bigendian="";
                image_msg.step = image_msg.width*4;
                image_msg.data.resize(image_msg.width * image_msg.height * 4);

                memcpy(&(image_msg.data[0]), cameramodel->FrameColor(), image_msg.width * image_msg.height * 4);

                //invert the opengl weirdness
                int height = image_msg.height - 1;
                int linewidth = image_msg.width*4;

                char* temp = new char[linewidth];
                for (int y = 0; y < (height+1)/2; y++)
                {
                    memcpy(temp,&image_msg.data[y*linewidth],linewidth);
                    memcpy(&(image_msg.data[y*linewidth]),&(image_msg.data[(height-y)*linewidth]),linewidth);
                    memcpy(&(image_msg.data[(height-y)*linewidth]),temp,linewidth);
                }

                if (robotmodel->cameramodels.size() > 1)
                    image_msg.header.frame_id = mapName("camera", r, s, static_cast<Stg::Model*>(robotmodel->positionmodel));
                else
                    image_msg.header.frame_id = mapName("camera", r,static_cast<Stg::Model*>(robotmodel->positionmodel));
                image_msg.header.stamp = sim_time;

                robotmodel->image_pubs[s].publish(image_msg);
            }

            //Get latest depth data
            //Translate into ROS message format and publish
            //Skip if there are no subscribers
            if (robotmodel->depth_pubs[s].getNumSubscribers()>0 && cameramodel->FrameDepth())
            {
                sensor_msgs::Image depth_msg;
                depth_msg.height = cameramodel->getHeight();
                depth_msg.width = cameramodel->getWidth();
                depth_msg.encoding = this->isDepthCanonical?sensor_msgs::image_encodings::TYPE_32FC1:sensor_msgs::image_encodings::TYPE_16UC1;
                //this->depthMsgs[r].is_bigendian="";
                int sz = this->isDepthCanonical?sizeof(float):sizeof(uint16_t);
                size_t len = depth_msg.width * depth_msg.height;
                depth_msg.step = depth_msg.width * sz;
                depth_msg.data.resize(len*sz);

                //processing data according to REP118
                if (this->isDepthCanonical){
                    double nearClip = cameramodel->getCamera().nearClip();
                    double farClip = cameramodel->getCamera().farClip();
                    memcpy(&(depth_msg.data[0]),cameramodel->FrameDepth(),len*sz);
                    float * data = (float*)&(depth_msg.data[0]);
                    for (size_t i=0;i<len;++i)
                        if(data[i]<=nearClip)
                            data[i] = -INFINITY;
                        else if(data[i]>=farClip)
                            data[i] = INFINITY;
                }
                else{
                    int nearClip = (int)(cameramodel->getCamera().nearClip() * 1000);
                    int farClip = (int)(cameramodel->getCamera().farClip() * 1000);
                    for (size_t i=0;i<len;++i){
                        int v = (int)(cameramodel->FrameDepth()[i]*1000);
                        if (v<=nearClip || v>=farClip) v = 0;
                        ((uint16_t*)&(depth_msg.data[0]))[i] = (uint16_t) ((v<=nearClip || v>=farClip) ? 0 : v );
                    }
                }

                //invert the opengl weirdness
                int height = depth_msg.height - 1;
                int linewidth = depth_msg.width*sz;

                char* temp = new char[linewidth];
                for (int y = 0; y < (height+1)/2; y++)
                {
                    memcpy(temp,&depth_msg.data[y*linewidth],linewidth);
                    memcpy(&(depth_msg.data[y*linewidth]),&(depth_msg.data[(height-y)*linewidth]),linewidth);
                    memcpy(&(depth_msg.data[(height-y)*linewidth]),temp,linewidth);
                }

                if (robotmodel->cameramodels.size() > 1)
                    depth_msg.header.frame_id = mapName("camera", r, s, static_cast<Stg::Model*>(robotmodel->positionmodel));
                else
                    depth_msg.header.frame_id = mapName("camera", r, static_cast<Stg::Model*>(robotmodel->positionmodel));
                depth_msg.header.stamp = sim_time;
                robotmodel->depth_pubs[s].publish(depth_msg);
            }

            //sending camera's tf and info only if image or depth topics are subscribed to
            if ((robotmodel->image_pubs[s].getNumSubscribers()>0 && cameramodel->FrameColor())
                    || (robotmodel->depth_pubs[s].getNumSubscribers()>0 && cameramodel->FrameDepth()))
            {

                Stg::Pose lp = cameramodel->GetPose();
                tf::Quaternion Q; Q.setRPY(
                            (cameramodel->getCamera().pitch()*M_PI/180.0)-M_PI,
                            0.0,
                            lp.a+(cameramodel->getCamera().yaw()*M_PI/180.0) - robotmodel->positionmodel->GetPose().a
                            );

                tf::Transform tr =  tf::Transform(Q, tf::Point(lp.x, lp.y, robotmodel->positionmodel->GetGeom().size.z+lp.z));

                if (robotmodel->cameramodels.size() > 1)
                    tf.sendTransform(tf::StampedTransform(tr, sim_time,
                                                          mapName("base_link", r, static_cast<Stg::Model*>(robotmodel->positionmodel)),
                                                          mapName("camera", r, s, static_cast<Stg::Model*>(robotmodel->positionmodel))));
                else
                    tf.sendTransform(tf::StampedTransform(tr, sim_time,
                                                          mapName("base_link", r, static_cast<Stg::Model*>(robotmodel->positionmodel)),
                                                          mapName("camera", r, static_cast<Stg::Model*>(robotmodel->positionmodel))));

                sensor_msgs::CameraInfo camera_msg;
                if (robotmodel->cameramodels.size() > 1)
                    camera_msg.header.frame_id = mapName("camera", r, s, static_cast<Stg::Model*>(robotmodel->positionmodel));
                else
                    camera_msg.header.frame_id = mapName("camera", r, static_cast<Stg::Model*>(robotmodel->positionmodel));
                camera_msg.header.stamp = sim_time;
                camera_msg.height = cameramodel->getHeight();
                camera_msg.width = cameramodel->getWidth();

                double fx,fy,cx,cy;
                cx = camera_msg.width / 2.0;
                cy = camera_msg.height / 2.0;
                double fovh = cameramodel->getCamera().horizFov()*M_PI/180.0;
                double fovv = cameramodel->getCamera().vertFov()*M_PI/180.0;
                //double fx_ = 1.43266615300557*this->cameramodels[r]->getWidth()/tan(fovh);
                //double fy_ = 1.43266615300557*this->cameramodels[r]->getHeight()/tan(fovv);
                fx = cameramodel->getWidth()/(2*tan(fovh/2));
                fy = cameramodel->getHeight()/(2*tan(fovv/2));

                //ROS_INFO("fx=%.4f,%.4f; fy=%.4f,%.4f", fx, fx_, fy, fy_);


                camera_msg.D.resize(4, 0.0);

                camera_msg.K[0] = fx;
                camera_msg.K[2] = cx;
                camera_msg.K[4] = fy;
                camera_msg.K[5] = cy;
                camera_msg.K[8] = 1.0;

                camera_msg.R[0] = 1.0;
                camera_msg.R[4] = 1.0;
                camera_msg.R[8] = 1.0;

                camera_msg.P[0] = fx;
                camera_msg.P[2] = cx;
                camera_msg.P[5] = fy;
                camera_msg.P[6] = cy;
                camera_msg.P[10] = 1.0;

                robotmodel->camera_pubs[s].publish(camera_msg);

            }

        }
    }

    this->base_last_globalpos_time = this->sim_time;
    rosgraph_msgs::Clock clock_msg;
    clock_msg.clock = sim_time;
    this->clock_pub_.publish(clock_msg);
}

int 
main(int argc, char** argv)
{ 
    if( argc < 2 )
    {
        puts(USAGE);
        exit(-1);
    }

    ros::init(argc, argv, "upo_stageros");

    bool gui = true;
    bool use_model_names = false;
    for(int i=0;i<(argc-1);i++)
    {
        if(!strcmp(argv[i], "-g"))
            gui = false;
        if(!strcmp(argv[i], "-u"))
            use_model_names = true;
    }

    StageNode sn(argc-1,argv,gui,argv[argc-1], use_model_names);

    if(sn.SubscribeModels() != 0)
        exit(-1);

    boost::thread t = boost::thread(boost::bind(&ros::spin));

    // New in Stage 4.1.1: must Start() the world.
    sn.world->Start();

    // TODO: get rid of this fixed-duration sleep, using some Stage builtin
    // PauseUntilNextUpdate() functionality.
    ros::WallRate r(10.0);
    while(ros::ok() && !sn.world->TestQuit())
    {
        if(gui)
            Fl::wait(r.expectedCycleTime().toSec());
        else
        {
            sn.UpdateWorld();
            r.sleep();
        }
    }
    t.join();

    exit(0);
}

