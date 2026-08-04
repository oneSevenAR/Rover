#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <cmath>

class VoxelDownsampler : public rclcpp::Node
{
public:
    VoxelDownsampler() : Node("voxel_downsampler")
    {
        // ROS 2 parameters for dynamic tuning
        this->declare_parameter("obstacle_angle_threshold", 90.0); // Maximum climbing angle
        this->declare_parameter("max_vision_distance", 1.0);       // Max depth (meters) to ignore distant stereo noise

        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/stereo/points2", 10, 
            std::bind(&VoxelDownsampler::pointcloud_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/stereo_camera/points_downsampled", 10);

        RCLCPP_INFO(this->get_logger(), "Traversability Analyzer (MLS + Normal Filtering) Started.");
    }

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // 0. Convert ROS message to a PCL XYZ point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud_xyz);

        // 1. VoxelGrid Downsampling (Compress the cloud)
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxeled(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(cloud_xyz);
        voxel_filter.setLeafSize(0.1f, 0.1f, 0.1f);
        voxel_filter.filter(*cloud_voxeled);

        // 2. PassThrough Filter (Distance Crop to kill distant phantom walls)
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cropped(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud_voxeled);
        pass.setFilterFieldName("z"); // In optical frames, Z is depth
        
        double max_depth = this->get_parameter("max_vision_distance").as_double();
        pass.setFilterLimits(0.0, max_depth); 
        pass.filter(*cloud_cropped);

        // 3. Statistical Outlier Removal (Destroy localized stereo speckle noise)
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sor(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud_cropped);
        sor.setMeanK(50); 
        sor.setStddevMulThresh(1.0); 
        sor.filter(*cloud_sor);

        // 4. Moving Least Squares (The Digital Steamroller)
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_smoothed(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
        mls.setInputCloud(cloud_sor);
        mls.setSearchRadius(0.3); // 30cm radius - flattens everything within this circle
        mls.setPolynomialOrder(2);
        
        pcl::search::KdTree<pcl::PointXYZ>::Ptr mls_tree(new pcl::search::KdTree<pcl::PointXYZ>());
        mls.setSearchMethod(mls_tree);
        mls.process(*cloud_smoothed);

        // 5. Surface Normal Estimation
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud(cloud_smoothed);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr ne_tree(new pcl::search::KdTree<pcl::PointXYZ>());
        ne.setSearchMethod(ne_tree);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>());
        // Search radius needs to be larger than MLS radius to get good slope context
        ne.setRadiusSearch(0.4); 
        ne.compute(*cloud_normals);

        // 6. Kinematic Angle Filtering (Isolate Steep Obstacles)
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_obstacles(new pcl::PointCloud<pcl::PointXYZ>());
        
        // Fetch threshold and convert to radians
        double threshold_deg = this->get_parameter("obstacle_angle_threshold").as_double();
        double threshold_rad = threshold_deg * M_PI / 180.0;

        for (size_t i = 0; i < cloud_normals->points.size(); ++i) {
            double nz = cloud_normals->points[i].normal_z;
            
            // Skip invalid normals (NaN)
            if (std::isnan(nz)) {
                continue;
            }
            
            // Calculate slope angle relative to gravity vector
            double angle = std::acos(std::abs(nz)); 
            
            // If the slope is steeper than the rover's climbing limit, it's an obstacle
            if (angle > threshold_rad) {
                cloud_obstacles->points.push_back(cloud_smoothed->points[i]);
            }
        }
        
        // Finalize the filtered point cloud format
        cloud_obstacles->width = cloud_obstacles->points.size();
        cloud_obstacles->height = 1;
        cloud_obstacles->is_dense = true;

        // 7. Convert back to ROS and publish for Nav2
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*cloud_obstacles, output_msg);
        output_msg.header = msg->header;

        publisher_->publish(output_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VoxelDownsampler>());
    rclcpp::shutdown();
    return 0;
}