#include <pluginlib/class_list_macros.h>

#include <chrono>
#include <ctime>
#include <math.h>
#include <queue>
#include <unordered_map>

#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <astar_ackermann_planner/astar_ackermann_planner.h>
#include <astar_ackermann_planner/utils.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/transform_listener.h>


namespace planner {
    class Planner
    {

    private:
        ros::NodeHandle *nh;
        astar_ackermann_planner::AStarAckermannPlanner planner;
        tf2_ros::Buffer tfBuffer;
        nav_msgs::OccupancyGrid *grid;
        ros::Subscriber occupancy_grid_sub;
        ros::Publisher costmap_pub;

    public:
        Planner(ros::NodeHandle *nh)
        {
            this->nh = nh;
            this->occupancy_grid_sub = nh->subscribe<nav_msgs::OccupancyGrid>("/map", 1, &Planner::occupancyGridCallback, this);
            this->costmap_pub = nh->advertise<nav_msgs::OccupancyGrid>("/costmap", 1000);
            // this->tfBuffer.setUsingDedicatedThread(true);
        }

        void occupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
        {
            // Create a Costmap2D object and initialize it with the occupancy grid data
          costmap_2d::Costmap2D *costmap = new costmap_2d::Costmap2D(msg->info.width, msg->info.height, msg->info.resolution,
                                            msg->info.origin.position.x, msg->info.origin.position.y);

            // tf2_ros::TransformListener tf_listener(this->tfBuffer);

            // costmap_2d::Costmap2DROS ros_costmap("my_costmap", this->tfBuffer);
            // costmap_2d::Costmap2D *costmap = ros_costmap.getCostmap();
            // costmap->resizeMap(msg->info.width, msg->info.height, msg->info.resolution,
            //                                 msg->info.origin.position.x, msg->info.origin.position.y);
            
             // Copy the occupancy grid data to the costmap
            for (unsigned int x = 0; x < msg->info.width; ++x)
            {
            for (unsigned int y = 0; y < msg->info.height; ++y)
                {
                unsigned int index = x + y * msg->info.width;
                unsigned char cost = msg->data[index];

                // Convert occupancy grid data to costmap value
                unsigned char costmap_cost;
                if (cost >= 0 && cost < 65)
                    costmap_cost = costmap_2d::FREE_SPACE;
                else if (cost >= 65)
                    costmap_cost = costmap_2d::LETHAL_OBSTACLE;
                else
                    costmap_cost = costmap_2d::NO_INFORMATION;

                // Set the cost value in the costmap
                costmap->setCost(x, y, costmap_cost);
                }
            }

            // this->grid = this->costmapToGrid(*costmap);
            // this->publishCostmap();

            planner.initialize("Planner", new astar_ackermann_planner::CostmapAdapter(costmap));
            // planner.initialize("Planner", &ros_costmap);
            std::vector<geometry_msgs::PoseStamped> plan;
            geometry_msgs::PoseStamped start;
            geometry_msgs::PoseStamped goal;
        
            ros::Rate rate(5);
            while (ros::ok()) {
                start.header.stamp = ros::Time::now();
                start.pose.position.x = 0.0;
                start.pose.position.y = 0.0;
                start.pose.position.z = 0.0;
                start.pose.orientation.x = 0.0;
                start.pose.orientation.y = 0.0;
                start.pose.orientation.z = 0.0;
                start.pose.orientation.w = 0.0;
                goal = start;
                planner.makePlan(start, goal, plan);
                plan.clear();
                rate.sleep();
            }
        }

        void publishCostmap() 
        {
            ros::Rate rate(2);
            while (ros::ok()) {
                this->costmap_pub.publish(*(this->grid));
                rate.sleep();
            }
        }

        nav_msgs::OccupancyGrid* costmapToGrid(costmap_2d::Costmap2D costmap)
        {
            nav_msgs::OccupancyGrid *occupancy_grid_msg = new nav_msgs::OccupancyGrid();

            // Set the header of the message (e.g., frame ID and timestamp)
            occupancy_grid_msg->header.frame_id = "map";
            occupancy_grid_msg->header.stamp = ros::Time::now();

            // Set the dimensions and resolution of the occupancy grid
            occupancy_grid_msg->info.width = costmap.getSizeInCellsX();
            occupancy_grid_msg->info.height = costmap.getSizeInCellsY();
            occupancy_grid_msg->info.resolution = costmap.getResolution();

            // Set the origin of the occupancy grid
            occupancy_grid_msg->info.origin.position.x = costmap.getOriginX();
            occupancy_grid_msg->info.origin.position.y = costmap.getOriginY();
            occupancy_grid_msg->info.origin.position.z = 0.0;
            occupancy_grid_msg->info.origin.orientation.x = 0.0;
            occupancy_grid_msg->info.origin.orientation.y = 0.0;
            occupancy_grid_msg->info.origin.orientation.z = 0.0;
            occupancy_grid_msg->info.origin.orientation.w = 1.0;

            // Copy the costmap data to the occupancy grid
            occupancy_grid_msg->data.resize(costmap.getSizeInCellsX() * costmap.getSizeInCellsY());

            for (unsigned int x = 0; x < costmap.getSizeInCellsX(); ++x)
            {
            for (unsigned int y = 0; y < costmap.getSizeInCellsY(); ++y)

            {
                unsigned char cost = costmap.getCost(x, y);

                // Convert costmap value to occupancy grid data
                if (cost == costmap_2d::FREE_SPACE)
                occupancy_grid_msg->data[x + y * occupancy_grid_msg->info.width] = 0;
                else if (cost == costmap_2d::LETHAL_OBSTACLE)
                occupancy_grid_msg->data[x + y * occupancy_grid_msg->info.width] = 100;
                else
                occupancy_grid_msg->data[x + y * occupancy_grid_msg->info.width] = -1;
            }
            }
            return occupancy_grid_msg;
        }
    };
}

 int main(int argc, char **argv) {
        ros::init(argc, argv, "planner");
        ros::NodeHandle nh;
        // tf2_ros::Buffer tfBuffer;
        // tfBuffer.setUsingDedicatedThread(true);
        planner::Planner p(&nh);
        // astar_ackermann_planner::AStarAckermannPlanner planner;
        // costmap_2d::Costmap2DROS costmap("global_costmap", tfBuffer);
        // while (!costmap.isCurrent())
        // {
        //     ros::Duration(0.1).sleep();
        //     ros::spinOnce();
        // }
        ros::spin();
        // planner.initialize("Planner", new astar_ackermann_planner::CostmapAdapter(costmap.getCostmap()));
    
        return 0;
    }