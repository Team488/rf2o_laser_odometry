/** ****************************************************************************************
*  This node presents a fast and precise method to estimate the planar motion of a lidar
*  from consecutive range scans. It is very useful for the estimation of the robot odometry from
*  2D laser range measurements.
*  This module is developed for mobile robots with innacurate or inexistent built-in odometry.
*  It allows the estimation of a precise odometry with low computational cost.
*  For more information, please refer to:
*
*  Planar Odometry from a Radial Laser Scanner. A Range Flow-based Approach. ICRA'16.
*  Available at: http://mapir.isa.uma.es/mapirwebsite/index.php/mapir-downloads/papers/217
*
* Maintainer: Javier G. Monroy
* MAPIR group: http://mapir.isa.uma.es/
******************************************************************************************** */

#ifndef CLaserOdometry2D_H
#define CLaserOdometry2D_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <angles/angles.h>
//#include <maidbot_obstacle_identification/OccupancyData.h>

// MRPT related headers
#include <mrpt/version.h>
#if MRPT_VERSION>=0x130
    #include <mrpt/obs/CObservation2DRangeScan.h>
    #include <mrpt/obs/CObservationOdometry.h>
    #include <mrpt/utils/CTicTac.h>
    using namespace mrpt::obs;
#else
#   include <mrpt/slam/CObservation2DRangeScan.h>
#   include <mrpt/slam/CObservationOdometry.h>
    using namespace mrpt::slam;
    #include <mrpt/utils.h>
#endif

#if MRPT_VERSION<0x150
#   include <mrpt/system/threads.h>
#endif

#include <mrpt/system/os.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/opengl.h>
#include <mrpt/math/CHistogram.h>

#include <boost/bind.hpp>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <iostream>
#include <fstream>
#include <numeric>
#include <cmath>

class CLaserOdometry2D
{
public:
	CLaserOdometry2D();
    ~CLaserOdometry2D();
    bool is_initialized();
    bool scan_available();
    void Init();
    void odometryCalculation();
    void handleMissingData();
    bool sensorHasTimedOut();

    std::string         laser_scan_topic;
    std::string         odom_topic;
    bool                publish_tf;
    std::string         base_frame_id;
    std::string         odom_frame_id;
    std::string         init_pose_from_topic;
    double              freq;

protected:
    ros::NodeHandle             n;
    sensor_msgs::LaserScan      last_scan;
    sensor_msgs::LaserScan      benchmark_scan_;
    bool                        module_initialized,first_laser_scan,new_scan_available, GT_pose_initialized, verbose;
    tf::TransformListener       tf_listener;          //Do not put inside the callback
    tf::TransformBroadcaster    odom_broadcaster;
    nav_msgs::Odometry          initial_robot_pose;

    //Subscriptions & Publishers
    ros::Subscriber laser_sub, initPose_sub, occ_hist_sub_;
    ros::Publisher odom_pub, interp_scan_pub_;

    double max_angular_speed_;
    double max_linear_speed_;
    double filter_lag_duration_;
    float  sensor_timeout_;

    //CallBacks
    void LaserCallBack(const sensor_msgs::LaserScan::ConstPtr& new_scan);
    void initPoseCallBack(const nav_msgs::Odometry::ConstPtr& new_initPose);

    //void occHistCb(const maidbot_obstacle_identification::OccupancyData::ConstPtr& msg);
    void setOdomCovariances(nav_msgs::Odometry& odom);
    //maidbot_obstacle_identification::OccupancyData occ_hist_;
    std::vector<float> x_info_densities_;
    std::vector<float> y_info_densities_;
    int density_avg_window_;
    int odom_avg_window_;

    float min_info_density_;
    float max_linear_cov_;
    float min_linear_cov_;
    double angular_cov_mult_;
    bool use_constant_cov_;

    // Internal Data
	std::vector<Eigen::MatrixXf> range;
	std::vector<Eigen::MatrixXf> range_old;
	std::vector<Eigen::MatrixXf> range_inter;
	std::vector<Eigen::MatrixXf> range_warped;
	std::vector<Eigen::MatrixXf> xx;
	std::vector<Eigen::MatrixXf> xx_inter;
	std::vector<Eigen::MatrixXf> xx_old;
	std::vector<Eigen::MatrixXf> xx_warped;
	std::vector<Eigen::MatrixXf> yy;
	std::vector<Eigen::MatrixXf> yy_inter;
	std::vector<Eigen::MatrixXf> yy_old;
	std::vector<Eigen::MatrixXf> yy_warped;
	std::vector<Eigen::MatrixXf> transformations;

	Eigen::MatrixXf range_wf;
	Eigen::MatrixXf dtita;
	Eigen::MatrixXf dt;
	Eigen::MatrixXf rtita;
	Eigen::MatrixXf normx, normy, norm_ang;
	Eigen::MatrixXf weights;
	Eigen::MatrixXi null;

    Eigen::MatrixXf A,Aw;
    Eigen::MatrixXf B,Bw;
	Eigen::Matrix<float, 3, 1> Var;	//3 unknowns: vx, vy, w
	Eigen::Matrix<float, 3, 3> cov_odo;

    //std::string LaserVarName;				//Name of the topic containing the scan lasers \laser_scan
	float fps;								//In Hz
	float fovh;								//Horizontal FOV
  std::vector<float> range_angles_;

  unsigned int cols;
	unsigned int cols_i;
	unsigned int width;
	unsigned int ctf_levels;
	unsigned int image_level, level;
	unsigned int num_valid_range;
    unsigned int iter_irls;
	float g_mask[5];

    //mrpt::gui::CDisplayWindowPlots window;
	mrpt::utils::CTicTac		m_clock;
	float		m_runtime;
    ros::Time last_odom_time;

	mrpt::math::CMatrixFloat31 kai_abs;
	mrpt::math::CMatrixFloat31 kai_loc;
	mrpt::math::CMatrixFloat31 kai_loc_old;
	mrpt::math::CMatrixFloat31 kai_loc_level;

	mrpt::poses::CPose3D laser_pose;
	mrpt::poses::CPose3D laser_oldpose;
    mrpt::poses::CPose3D robot_pose;
    mrpt::poses::CPose3D robot_oldpose;
	bool test;
    std::vector<double> last_m_x_speeds;
    std::vector<double> last_m_y_speeds;
    std::vector<double> last_m_th_speeds;


	// Methods
  void interpolateScanToFixedAngles(
    const sensor_msgs::LaserScan::ConstPtr& new_scan,
    sensor_msgs::LaserScan& interpolated_scan);
	void createImagePyramid();
	void calculateCoord();
	void performWarping();
	void calculaterangeDerivativesSurface();
	void computeNormals();
	void computeWeights();
	void findNullPoints();
	void solveSystemOneLevel();
    void solveSystemNonLinear();
	void filterLevelSolution();
	void PoseUpdate();
    void Reset(mrpt::poses::CPose3D ini_pose, CObservation2DRangeScan scan);
};

#endif
