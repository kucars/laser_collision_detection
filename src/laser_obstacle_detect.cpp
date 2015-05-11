/**********************************************************************************
* Copyright (C) 2013 - 2014 by                                                    *
* Tarek Taha, Reem Ashour Khalifa University Robotics Institute KURI              *
* <tarek.taha@kustar.ac.ae>, <reem.ashour@kustar.ac.ae>                           *
*                                                                                 *
*                                                                                 *
* This program is free software; you can redistribute it and/or modify     	  *
* it under the terms of the GNU General Public License as published by            *
* the Free Software Foundation; either version 2 of the License, or               *
* (at your option) any later version.                                             *
*                                                                                 *
* This program is distributed in the hope that it will be useful,                 *
* but WITHOUT ANY WARRANTY; without even the implied warranty of                  *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the                    *
* GNU General Public License for more details.                              	  *
*                                                                                 *
* You should have received a copy of the GNU General Public License               *
* along with this program; if not, write to the                                   *
* Free Software Foundation, Inc.,                                                 *
* 51 Franklin Steet, Fifth Floor, Boston, MA 02111-1307, USA.                     *
***********************************************************************************/
#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include "geometry_msgs/PoseStamped.h"

#include <octomap_msgs/Octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include "nav_msgs/Odometry.h"

#include <iostream>
#include <octomap_msgs/conversions.h>
#include <deque>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Bool.h>

#include "fcl/shape/geometric_shapes.h"
#include "fcl/narrowphase/narrowphase.h"
#include "fcl/collision.h"
#include "fcl/ccd/motion.h"
#include <stdlib.h>
#include <boost/foreach.hpp>
#include <Eigen/Eigen>
#include "fcl/octree.h"
#include "fcl/traversal/traversal_node_octree.h"
#include "fcl/broadphase/broadphase.h"
#include "fcl/shape/geometric_shape_to_BVH_model.h"
#include "fcl/math/transform.h"

#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/array.hpp>
#include <octomap/octomap.h>
#include "fcl/BV/AABB.h"
#include "fcl/collision_object.h"

using namespace std;
using namespace fcl;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

inline double dist(geometry_msgs::Point32 &p1, geometry_msgs::Pose &p2)
{
    return sqrt((p2.position.x - p1.x)*(p2.position.x - p1.x) + (p2.position.y - p1.y)*(p2.position.y - p1.y) + (p2.position.z - p1.z)*(p2.position.z - p1.z));
}

class LaserObstacleDetector{
public:

    ros::NodeHandle node;

    laser_geometry::LaserProjection projector_;
    tf::TransformListener tfListener;

    ros::Subscriber laserSub;
    ros::Subscriber robotPoseSub ;
    ros::Subscriber robotVelSub;

    ros::Publisher visPub ;
    ros::Publisher visCubePub ;
    ros::Publisher laserOctmapPub;
    ros::Publisher collisionFlagPub;
    double roll, pitch, yaw ;
    geometry_msgs::PoseStamped robotPose;
    geometry_msgs::Point32     robotVel;
    std::string cmdVelTopic,poseTopic,scanTopic;
    double minDist,maxDist,octMapRes,robotRadius;
    double velDeltaT; // predict collision in seconds interval
    LaserObstacleDetector(ros::NodeHandle n):node(n)
    {
        ros::NodeHandle privateNh("~");
        privateNh.param<std::string>("cmd_vel_topic", cmdVelTopic,   std::string("/uav/cmd_vel"));
        privateNh.param<std::string>("pose_topic",    poseTopic, std::string("/mavros/vision_pose/pose"));
        privateNh.param<std::string>("scan_topic",    scanTopic, std::string("/scan"));
        privateNh.param<double>("min_dist",    minDist, 0.5);
        privateNh.param<double>("max_dist",    maxDist, 4.0);
        privateNh.param<double>("vel_delta_t", velDeltaT, 1.0);
        privateNh.param<double>("octmap_res",  octMapRes, 1.0);
        privateNh.param<double>("robot_radius", robotRadius, 1.0);
        ROS_INFO("Scan topic:%s, Pose topic:%s, Cmd Vel topic:%s, Min dist:%f, Max Dist:%f, Vel DeltaT:%f, OctMap res:%f, Robot r:%f",scanTopic.c_str(),poseTopic.c_str(),cmdVelTopic.c_str(),minDist,maxDist,velDeltaT,octMapRes,robotRadius);
        // subscribers
        laserSub      = node.subscribe(scanTopic,10, &LaserObstacleDetector::laserCallback, this);
        robotPoseSub  = node.subscribe(poseTopic , 100 ,&LaserObstacleDetector::poseCallback, this);
        robotVelSub   = node.subscribe(cmdVelTopic , 100 ,&LaserObstacleDetector::velCallback, this);

        // Publishers
        visPub           = node.advertise<visualization_msgs::Marker>("Sphere", 1);
        visCubePub       = node.advertise<visualization_msgs::MarkerArray>("Cube", 1);
        laserOctmapPub   = node.advertise<octomap_msgs::Octomap>("LaserOctmap", 1);
        collisionFlagPub = node.advertise<std_msgs::Bool>("collision_flag",1) ;
    }

    void laserCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
    {
        if(!tfListener.waitForTransform(scan_in->header.frame_id,
                                       "world",
                                       scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
                                       ros::Duration(1.0)))
        {
            ROS_WARN("Can't get the correct transformation in time");
            return;
        }

        sensor_msgs::PointCloud msg;
        projector_.transformLaserScanToPointCloud("world",*scan_in, msg,tfListener);

        // create empty tree with resolution 0.1
        octomap::OcTree* octTree = new octomap::OcTree(octMapRes);
        octomap::Pointcloud octPointCloud;
        double distance = 0;
        //Filter laser data if needed
        for(int i = 0;i<msg.points.size();i++)
        {
            distance = dist(msg.points[i],robotPose.pose);
            if(distance>=minDist && distance<=maxDist)
            {
                octomap::point3d endpoint((float) msg.points[i].x,(float) msg.points[i].y,(float) msg.points[i].z);
                octPointCloud.push_back(endpoint);
            }
        }
        octomap::point3d origin(0.0,0.0,0.0);
        octTree->insertPointCloud(octPointCloud,origin);
        octTree->updateInnerOccupancy();
        //octTree->writeBinary("static_occ.bt");

        // convert the octomap::octree to fcl::octree fcl_octree object
        OcTree* tree2 = new OcTree(boost::shared_ptr<const octomap::OcTree>(octTree));

        octomap_msgs::Octomap octomap ;
        octomap.binary = 1 ;
        octomap.id = 1 ;
        octomap.resolution =0.1;
        octomap.header.frame_id = "/map";
        octomap.header.stamp = ros::Time::now();
        bool res = octomap_msgs::fullMapToMsg(*octTree, octomap);
        if(res)
        {
            laserOctmapPub.publish(octomap);
        }
        else
        {
            ROS_WARN("OCT Map serialization failed!");
        }

        std_msgs::Bool collisionFlag;
        boost::shared_ptr<Sphere> Shpere0(new Sphere(robotRadius));
        visualization_msgs::MarkerArray marker_array ;
        visualization_msgs::Marker marker ;

        /*
         * Predict Collision based on currnet speed: only in 2D
         */
        Transform3f tf0;
        tf0.setIdentity();
        double speedInx  = robotVel.z;
        double speedIny  = robotVel.y;
        double xPrime    = speedInx*velDeltaT ;
        double yPrime    = speedIny*velDeltaT ;
        tf0.setTranslation(Vec3f(robotPose.pose.position.x +xPrime, robotPose.pose.position.y +yPrime,robotPose.pose.position.z));
        marker  = drawCUBE(Vec3f(robotPose.pose.position.x +xPrime, robotPose.pose.position.y +yPrime,robotPose.pose.position.z),1000,3) ;

        marker_array.markers.push_back(marker);

        CollisionObject co0(Shpere0, tf0);
        // for visualization
        AABB a    = co0.getAABB() ;
        Vec3f vec =  a.center() ;
        drawSphere(vec) ;

        std::vector<CollisionObject*> boxes;
        generateBoxesFromOctomap(boxes, *tree2);
        bool collisionDetected = false;
        for(size_t i = 0; i < boxes.size(); ++i)
        {
            CollisionObject* box =  boxes[i];
            static const int num_max_contacts = std::numeric_limits<int>::max();
            static const bool enable_contact = true ;
            // for visualization
            AABB b = box->getAABB() ;
            Vec3f vec2 =  b.center();

            fcl::CollisionResult result;
            fcl::CollisionRequest request(num_max_contacts, enable_contact);
            fcl::collide(&co0, box, request, result);

            if (result.isCollision() == true )
            {
                marker = drawCUBE(vec2,i,2) ;
                marker_array.markers.push_back(marker);
                collisionFlag.data = true ;
                collisionFlagPub.publish(collisionFlag) ;
                collisionDetected = true;
            }
            else
            {
                marker = drawCUBE(vec2, i, 1) ;
                marker_array.markers.push_back(marker);
            }

        }
        visCubePub.publish(marker_array);
        for(size_t i = 0; i < boxes.size(); ++i)
            delete boxes[i];
        if(!collisionDetected)
        {
            collisionFlag.data = false ;
            collisionFlagPub.publish(collisionFlag) ;
        }
    }
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr & robot_pose)
    {
        robotPose = *robot_pose;
        tf::Quaternion q(robotPose.pose.orientation.x, robotPose.pose.orientation.y ,robotPose.pose.orientation.z,robotPose.pose.orientation.w);
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    }

    void velCallback(const geometry_msgs::TwistStamped::ConstPtr&  cmd_vel)
    {
        robotVel.x =  cmd_vel->twist.linear.x ;
        robotVel.y =  cmd_vel->twist.linear.y  ;
        robotVel.z =  cmd_vel->twist.linear.z ;
    }

    void generateBoxesFromOctomap(std::vector<CollisionObject*>& boxes, OcTree& tree)
    {

        std::vector<boost::array<FCL_REAL, 6> > boxes_ = tree.toBoxes();
        for(std::size_t i = 0; i < boxes_.size(); ++i)
        {
            FCL_REAL x = boxes_[i][0];
            FCL_REAL y = boxes_[i][1];
            FCL_REAL z = boxes_[i][2];
            FCL_REAL size = boxes_[i][3];
            FCL_REAL cost = boxes_[i][4];
            FCL_REAL threshold = boxes_[i][5];
            Box* box = new Box(size, size, size);
            box->cost_density = cost;
            box->threshold_occupied = threshold;
            CollisionObject* obj = new CollisionObject(boost::shared_ptr<CollisionGeometry>(box), Transform3f(Vec3f(x, y, z)));
            boxes.push_back(obj);
        }
    }

    void drawSphere(Vec3f vec )
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/world";
        marker.header.stamp = ros::Time();
        marker.ns = "my_namespace";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = vec[0];
        marker.pose.position.y = vec[1];
        marker.pose.position.z = vec[2];
        marker.pose.orientation = robotPose.pose.orientation;
        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 1;
        marker.color.a = 0.3;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.lifetime = ros::Duration(0.3);
        visPub.publish( marker );
    }

    visualization_msgs::Marker drawCUBE(Vec3f vec , int id , int c_color)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/world";
        marker.header.stamp = ros::Time();
        marker.ns = "my_namespace";
        marker.id = id;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = vec[0];
        marker.pose.position.y = vec[1];
        marker.pose.position.z = vec[2];
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;//poseQ[1];
        marker.pose.orientation.z = 0;//poseQ[2];
        marker.pose.orientation.w = 1;//poseQ[3];

        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        if(c_color == 1)
        {
            marker.color.r = 0.0;
            marker.color.b = 1.0;
            marker.color.g = 0.0;
        }
        else if(c_color == 2)
        {
            marker.color.r = 1.0;
            marker.color.b = 0.0;
            marker.color.g = 0.0;
        }
        else
        {
            marker.color.r = 0.0;
            marker.color.b = 0.0;
            marker.color.g = 1.0;
        }
        marker.lifetime = ros::Duration(0.3);
        return marker ;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_obstacle_detection_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);
    LaserObstacleDetector lstopc(n);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
