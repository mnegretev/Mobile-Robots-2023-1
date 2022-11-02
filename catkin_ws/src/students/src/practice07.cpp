/*
 * MOBILE ROBOTS - UNAM, FI, 2023-1
 * PRACTICE 7 - LOCALIZATION BY PARTICLE FILTERS
 *
 * Instructions:
 * Write the code necessary to implement localization by particle filters.
 * Modify only the sections marked with the TODO comment. 
 */

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/GetMap.h"
#include "random_numbers/random_numbers.h"
#include "occupancy_grid_utils/ray_tracer.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose2D.h"
#include "tf/transform_broadcaster.h"

#define NOMBRE "SANCHEZ TORRES SERGIO DANIEL"

#define LASER_DOWNSAMPLING  10
#define SENSOR_NOISE        0.1
#define RESAMPLING_NOISE    0.1
#define MOVEMENT_NOISE      0.1
#define DISTANCE_THRESHOLD  0.2
#define ANGLE_THRESHOLD     0.2

sensor_msgs::LaserScan real_sensor_info;
sensor_msgs::LaserScan real_scan;

geometry_msgs::PoseArray get_initial_distribution(int N, float min_x, float max_x, float min_y, float max_y, float min_a, float max_a)
{
    random_numbers::RandomNumberGenerator rnd;
    geometry_msgs::PoseArray particles;
    particles.poses.resize(N);
    particles.header.frame_id = "map";

    /*
     * TODO:
     *
     * Generate a set of N particles (each particle represented by a Pose message)
     * with positions uniformly distributed within bounding box given by min_x, ..., max_a.
     * The set of particles must be a PoseArray message.
     * To generate uniformly distributed random numbers, you can use the funcion rnd.uniformReal(min, max)
     * Remember that orientation in a Pose message is represented by a quaternion (x,y,z,w)
     * For the Euler angles (roll, pitch, yaw) = (0,0,theta) the corresponding quaternion is
     * given by (0,0,sin(theta/2), cos(theta/2)). 
     */

    for(int i = 0; i < particles.poses.size(); i++){
        particles.poses[i].position.x = rnd.uniformReal(min_x, max_x);
        particles.poses[i].position.y = rnd.uniformReal(min_y, max_y);

        float theta = rnd.uniformReal(min_a, max_a);

        particles.poses[i].orientation.w = cos(theta / 2);
        particles.poses[i].orientation.z = sin(theta / 2);
    }
    
    return particles;
}

std::vector<sensor_msgs::LaserScan> simulate_particle_scans(geometry_msgs::PoseArray& particles, nav_msgs::OccupancyGrid& map)
{
    std::vector<sensor_msgs::LaserScan> simulated_scans;
    simulated_scans.resize(particles.poses.size());
    /*
     * TODO:
     *
     * Simulate a laser scan for each particle given the set of particles and a static map. 
     * Store the simulated scans in 'simulated_scans'.
     * You can use the function occupancy_grid_utils::simulateRangeScan(map, pose, info).
     * Check online documentation
     * http://docs.ros.org/groovy/api/occupancy_grid_utils/html/namespaceoccupancy__grid__utils.html
     * Use the variable 'real_sensor_info' (already declared as global variable) for the real sensor information
     */
    for(int i = 0; i < particle.poses.size(); i++){
        simulated_scans[i] = *occupancy_grid_utils::simulateRangeScan(map, particles.poses[i], real_sensor_info);
    }

    return simulated_scans;
}

std::vector<float> calculate_particle_weights(std::vector<sensor_msgs::LaserScan>& simulated_scans, sensor_msgs::LaserScan& real_scan)
{
    std::vector<float> weights;
    weights.resize(simulated_scans.size());
    /*
     * TODO:
     *
     * For each particle, calculate a weight indicating the similarity between its simulated scan and the real scan.
     * Normalize all weights (the sum of all values must always be 1.0)
     * Store results in 'weights'.
     * IMPORTANT NOTE 1. The real sensor scans are DOWNSAMPLED. That is, only 1 out of LASER_DOWNSAMPLING scans is considered, i.e.,
     * For example, if LASER_DOWNSAMPLING=10, then, if real sensor has 500 ranges, simulated scans will only have 50 ranges
     * When comparing readings, for each reading in the simulated scan, you should skip LASER_DOWNSAMPLING readings
     * in the real sensor.
     * IMPORTANT NOTE 2. Both, simulated an real scans, can have infinite ranges. Thus, when comparing readings,
     * ensure both simulated and real ranges are finite values. 
     */
    double = sum_weights = 0;

    for(int i = 0; i < simulated_scans.size(); i++){
        weights[i] = 0;

        for(int j = 0; j < simulated_scans[i].ranges.size(); j++){
            if(real_scan.ranges[j * LASER_DOWNSAMPLING] < real_scan.range_max && simulated_scans[i].ranges[j] < real_scan.range_max){
                weights[i] += fabs(simulated_scans[i].ranges[j] - real_scan.ranges[j * LASER_DOWNSAMPLING]);
            } else {
                weight[i] += real_scan.range_max;
            }
        }
        weights[i] /= simulated_scans[i].ranges.size();
        weights[i] = exp(-weights[i] * weights[i] / SENSOR_NOISE);
        weights_sum += weights[i];
    }

    for(int i = 0; i < weights.size(); i++){
        weights[i] /= weights_sums;
    }
    
    return weights;
}

int random_choice(std::vector<float>& weights)
{
    random_numbers::RandomNumberGenerator rnd;
    
    /*
     * TODO:
     *
     * Write an algorithm to choice an integer in the range [0, N-1], with N, the size of 'weights'.
     * Probability of picking an integer 'i' is given by the corresponding weights[i] value.
     * Return the chosen integer. 
     */

    float rndInt = rnd.uniformReal(0, 1);

    for(int i = 0; i < weights.size(); i++){
        if(rndInt < weights[i]){
            return i;
        } else {
            rndInt -= weights[i];
        }
    }
    
    return -1;
}

geometry_msgs::PoseArray resample_particles(geometry_msgs::PoseArray& particles, std::vector<float>& weights)
{
    random_numbers::RandomNumberGenerator rnd;
    geometry_msgs::PoseArray resampled_particles;
    resampled_particles.header.frame_id = "map";
    resampled_particles.poses.resize(particles.poses.size());
    /*
     * TODO:
     *
     * Sample, with replacement, N particles from the set 'particles'.
     * The probability of the i-th particle of being resampled is given by weights[i].
     * Use the random_choice function to pick a particle with the correct probability.
     * Add gaussian noise to each sampled particle (add noise to x,y and theta).
     * Use RESAMPLING_NOISE as noise variance.
     * Return the set of new particles.
     * IMPORTANT NOTE. Remember the orientation (roll, pitch, yaw) = (0,0,theta) is
     * given by the quaternion (0,0,sin(theta/2), cos(theta/2)), thus, you should first
     * get the corresponding angle, then add noise, and the get again the corresponding quaternion.
     */

    for(int i = 0; i < particles.poses.size(); i++){
        int rndX = random_choice(weights);
        
        resampled_particles.poses[i] = particles.poses[rndX];
        resampled_particles.poses[i].position.x += rnd.gaussian(0, RESAMPLING_NOISE);
        resampled_particles.poses[i].position.y += rnd.gaussian(0, RESAMPLING_NOISE);

        float ang = atan2(particles.poses[rndX].orientation.z, particles.poses[rndX].orientation.w) * 2;

        ang += rnd.gaussian(0, RESAMPLING_NOISE);

        resampled_particles.poses[i].orientation.w = cos(ang / 2);
        resampled_particles.poses[i].orientation.z = sin(ang / 2); 
    }
    return resampled_particles;
}

void move_particles(geometry_msgs::PoseArray& particles, float delta_x, float delta_y, float delta_t)
{
    random_numbers::RandomNumberGenerator rnd;
    /*
     * TODO:
     *
     * Move each particle a displacement given by delta_x, delta_y and delta_t.
     * Displacement is given w.r.t. particles's frame, i.e., to calculate the new position for
     * each particle you need to rotate delta_x and delta_y, on Z axis, an angle theta_i, where theta_i
     * is the orientation of the i-th particle.
     * Add gaussian noise to each new position. Use MOVEMENT_NOISE as covariances. 
     */

    for(int i = 0; i < particles.poses.size(); i++){
        float f = atan2(particles.poses[i].orientation.z, particles.poses[i].orientation.w) * 2;

        particles.poses[i].position.x += delta_x * cos(f) - delta_y * sin(f) + rnd.gaussian(0, MOVEMENT_NOISE);
        particles.poses[i].position.y += delta_x * sin(f) + delta_y * cos(f) + rnd.gaussian(0, MOVEMENT_NOISE);

        f += delta_t + rnd.gaussian(0, MOVEMENT_NOISE);

        particles.poses[i].orientation.w = cos(f / 2);
        particles.poses[i].orientation.z = sin(f / 2);    
    }
}

bool check_displacement(geometry_msgs::Pose2D& robot_pose, geometry_msgs::Pose2D& delta_pose)
{
    static geometry_msgs::Pose2D last_pose;
    float delta_x = robot_pose.x - last_pose.x;
    float delta_y = robot_pose.y - last_pose.y;
    float delta_a = robot_pose.theta - last_pose.theta;
    if(delta_a >  M_PI) delta_a -= 2*M_PI;
    if(delta_a < -M_PI) delta_a += 2*M_PI;
    if(sqrt(delta_x*delta_x + delta_y*delta_y) > DISTANCE_THRESHOLD || fabs(delta_a) > ANGLE_THRESHOLD)
    {
        last_pose = robot_pose;
        delta_pose.x =  delta_x*cos(robot_pose.theta) + delta_y*sin(robot_pose.theta);
        delta_pose.y = -delta_x*sin(robot_pose.theta) + delta_y*cos(robot_pose.theta);
        delta_pose.theta = delta_a;
        return true;
    }
    return false;
}

geometry_msgs::Pose2D get_robot_odometry(tf::TransformListener& listener)
{
    tf::StampedTransform t;
    geometry_msgs::Pose2D pose;
    try{
        listener.lookupTransform("odom", "base_link", ros::Time(0), t);
        pose.x = t.getOrigin().x();
        pose.y = t.getOrigin().y();
        pose.theta = atan2(t.getRotation().z(), t.getRotation().w())*2;
    }
    catch(std::exception &e){
        pose.x = 0;
        pose.y = 0;
        pose.theta = 0;
    }
    return pose;
}

geometry_msgs::Pose2D get_robot_pose_estimation(geometry_msgs::PoseArray& particles)
{
    geometry_msgs::Pose2D p;
    float z = 0;
    float w = 0;
    for(size_t i=0; i < particles.poses.size(); i++)
    {
        p.x += particles.poses[i].position.x;
        p.y += particles.poses[i].position.y;
        z   += particles.poses[i].orientation.z;
        w   += particles.poses[i].orientation.w;
        
    }
    p.x /= particles.poses.size();
    p.y /= particles.poses.size();
    z   /= particles.poses.size();
    w   /= particles.poses.size();
    p.theta = atan2(z, w)*2;
    return p;
}

void callback_laser_scan(const sensor_msgs::LaserScan::ConstPtr& msg){real_scan = *msg;}

tf::Transform get_map_to_odom_transform(geometry_msgs::Pose2D odom, geometry_msgs::Pose2D loc)
{
    tf::Transform odom_to_base(tf::Quaternion(0,0,sin(odom.theta/2),cos(odom.theta/2)), tf::Vector3(odom.x,odom.y,0));
    tf::Transform map_to_base(tf::Quaternion(0,0,sin(loc.theta/2),cos(loc.theta/2)), tf::Vector3(loc.x, loc.y, 0));
    return map_to_base*odom_to_base.inverse();
}

int main(int argc, char** argv)
{
    std::cout << "PRACTICE 07 - " << NOMBRE << std::endl;
    ros::init(argc, argv, "practice07");
    ros::NodeHandle n("~");
    ros::Rate loop(20);
    ros::Subscriber sub_scan      = n.subscribe("/hardware/scan", 1, callback_laser_scan);
    ros::Publisher  pub_particles = n.advertise<geometry_msgs::PoseArray>("/particle_cloud", 1); 
    tf::TransformListener listener;
    tf::TransformBroadcaster broadcaster;
    nav_msgs::GetMap srv_get_map;

    float init_min_x = -1;
    float init_min_y = -1;
    float init_min_a = -1;
    float init_max_x = 2;
    float init_max_y = 2;
    float init_max_a = 1;
    float number_of_particles = 200;
    if(ros::param::has("~n"))
        ros::param::get("~n", number_of_particles);
    if(ros::param::has("~max_x"))
        ros::param::get("~max_x", init_max_x);
    if(ros::param::has("~min_x"))
        ros::param::get("~min_x", init_min_x);
    if(ros::param::has("~min_y"))
        ros::param::get("~min_y", init_min_y);
    if(ros::param::has("~min_a"))
        ros::param::get("~min_a", init_min_a);
    if(ros::param::has("~max_x"))
        ros::param::get("~max_x", init_max_x);
    if(ros::param::has("~max_y"))
        ros::param::get("~max_y", init_max_y);
    if(ros::param::has("~max_a"))
        ros::param::get("~max_a", init_max_a);

    /*
     * IMPORTANT VARIABLES FOR THE LOCALIZATION PROCESS
     */
    geometry_msgs::PoseArray particles;                   //A set of N particles
    nav_msgs::OccupancyGrid static_map;                   //A static map
    std::vector<sensor_msgs::LaserScan> simulated_scans;  //A set of simulated laser readings, one scan per particle
    std::vector<float> particle_weights;                  //A set of weights for each particle
    geometry_msgs::Pose2D robot_odom;                     //Position estimated by the odometry
    geometry_msgs::Pose2D delta_pose;                     //Displacement since last pose estimation
    geometry_msgs::Pose2D robot_pose;                     //Estimated robot position with respect to map
    tf::Transform map_to_odom_transform;                  //Transformation from map to odom frame (which corrects odometry estimation)

    /*
     * Sentences for getting the static map, info about real lidar sensor,
     * and initialization of corresponding arrays.
     */
    ros::service::waitForService("/static_map", ros::Duration(20));
    ros::service::call("/static_map", srv_get_map);
    static_map = srv_get_map.response.map;
    real_scan = *ros::topic::waitForMessage<sensor_msgs::LaserScan>("/hardware/scan");
    real_sensor_info = real_scan;
    real_sensor_info.angle_increment *= LASER_DOWNSAMPLING;
    std::cout << "Real Scan Info: Number of readings: " << real_scan.ranges.size() << std::endl;
    std::cout << "Min angle: " << real_scan.angle_min << std::endl;
    std::cout << "Angle increment: " << real_scan.angle_increment << std::endl;
    std::cout << "Scan downsampling: " << LASER_DOWNSAMPLING << std::endl;

    particles = get_initial_distribution(number_of_particles, init_min_x, init_max_x, init_min_y, init_max_y, init_min_a, init_max_a);
    robot_pose = get_robot_pose_estimation(particles);
    robot_odom = get_robot_odometry(listener);
    map_to_odom_transform = get_map_to_odom_transform(robot_odom, robot_pose);
    check_displacement(robot_odom, delta_pose);
    pub_particles.publish(particles);
    while(ros::ok())
    {
        robot_odom = get_robot_odometry(listener);
        if(check_displacement(robot_odom, delta_pose))
        {
            std::cout << "Displacement detected. Updating pose estimation..." << std::endl;
            /*
             * TODO:
             *
             * Move all particles a displacement given by delta_pose (Pose2D message) by calling the move_particles function.
             * Get the set of simulated scans for each particles. Use the simulate_particle_scans function.
             * Get the set of weights by calling the calculate_particle_weights function
             * Resample particles by calling the resample_particles function
             */

            /*
             * END OF TODO
             */
            pub_particles.publish(particles);
            map_to_odom_transform = get_map_to_odom_transform(robot_odom, get_robot_pose_estimation(particles));
        }
        broadcaster.sendTransform(tf::StampedTransform(map_to_odom_transform, ros::Time::now(), "map", "odom"));
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}
