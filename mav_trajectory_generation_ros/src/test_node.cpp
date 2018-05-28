#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_node");
    ros::NodeHandle n;

    ros::Publisher vis_pub = n.advertise<visualization_msgs::MarkerArray>("trajectory_traject", 10);
    // rate
    ros::Rate sleep_rate(1);
    // int marker_size = markers.markers.size();
    // int trajectory_size = markers.markers[marker_size-1].points.size();
    // for (int i=0 ; i < trajectory_size ; i++)
    // std::cout<<markers.markers[marker_size-1].points[i]<<std::endl;

    while (ros::ok())
    {   
        float x_,y_,z_;
        n.getParam("/test_node/x_", x_);
        n.getParam("/test_node/y_", y_);
        n.getParam("/test_node/z_", z_);

        mav_trajectory_generation::Vertex::Vector vertices;
        const int dimension = 3;
        const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
        mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);

        // Time count
        ros::Time t0 = ros::Time::now();

        // Add constraints to the vertices
        // curve 1 in paper
        start.makeStartOrEnd(Eigen::Vector3d(0,0,1),derivative_to_optimize);
        vertices.push_back(start);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(1,0,1));
        vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,Eigen::Vector3d(1,2,1));
        vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(x_, y_, z_));
        vertices.push_back(middle);

        end.makeStartOrEnd(Eigen::Vector3d(0,0,1),derivative_to_optimize);
        vertices.push_back(end);

        // curve 2 in paper
        // start.makeStartOrEnd(Eigen::Vector3d(0, 0, 0), derivative_to_optimize);
        // vertices.push_back(start);

        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-5, 5, 2.5));
        // vertices.push_back(middle);

        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-7.07, 7.07, 2));
        // vertices.push_back(middle);

        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(0, 10, 2.5));
        // vertices.push_back(middle);

        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(7.07, 7.07, 1.5));
        // vertices.push_back(middle);

        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(10, 0, 2.5));
        // vertices.push_back(middle);

        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(7.07, -7.07, 2.5));
        // vertices.push_back(middle);

        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(0, -10, 2));
        // vertices.push_back(middle);

        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-7.07, -7.07, 2.5));
        // vertices.push_back(middle);

        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(x_, y_, z_));
        // vertices.push_back(middle);

        // end.makeStartOrEnd(Eigen::Vector3d(0, 0, 0), derivative_to_optimize);
        // vertices.push_back(end);

        //compute the segment times
        std::vector<double> segment_times;
        const double v_max = 1.0;
        const double a_max = 3.0;
        const double magic_fabian_constant = 6.5; // A tuning parameter
        segment_times = estimateSegmentTimes(vertices, v_max, a_max, magic_fabian_constant);

        for (int i = 0; i < segment_times.size(); i++)
        {
            //std::cout << segment_times[i] << std::endl;
        }

        //N denotes the number of coefficients of the underlying polynomial
        //N has to be even.
        //If we want the trajectories to be snap-continuous, N needs to be at least 10.

        const int N = 10;
        mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
        opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
        opt.solveLinear();

        ROS_INFO("Take %f sec to get optimal traject", (ros::Time::now() - t0).toSec());

        //Obtain the polynomial segments
        mav_trajectory_generation::Segment::Vector segments;
        opt.getSegments(&segments);

        //creating Trajectories
        mav_trajectory_generation::Trajectory trajectory;
        opt.getTrajectory(&trajectory);

        //evaluating the trajectory at particular instances of time
        // Single sample:
        double sampling_time = 2.0;
        int derivative_order = mav_trajectory_generation::derivative_order::POSITION;
        Eigen::VectorXd sample = trajectory.evaluate(sampling_time, derivative_order);

        // Sample range:
        double t_start = 2.0;
        double t_end = 10.0;
        double dt = 0.01;
        std::vector<Eigen::VectorXd> result;
        std::vector<double> sampling_times; // Optional.
        trajectory.evaluateRange(t_start, t_end, dt, derivative_order, &result, &sampling_times);
        //std::cout << result.size() << std::endl;
        //std::cout << sampling_times.size() << std::endl;

        //visualizing Trajectories
        visualization_msgs::MarkerArray markers;
        double distance = 1.6; // Distance by which to seperate additional markers. Set 0.0 to disable.
        std::string frame_id = "map";
        // From Trajectory class:
        mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &markers);

        ros::spinOnce();

        vis_pub.publish(markers);
        sleep_rate.sleep();
    }
    return 0;
}