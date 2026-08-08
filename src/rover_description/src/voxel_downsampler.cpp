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
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud_xyz);

        // SAFETY CHECK
        if (cloud_xyz->points.empty()) return; 

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxeled(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(cloud_xyz);
        voxel_filter.setLeafSize(0.1f, 0.1f, 0.1f);
        voxel_filter.filter(*cloud_voxeled);

        if (cloud_voxeled->points.empty()) return; // SAFETY CHECK

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cropped(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud_voxeled);
        pass.setFilterFieldName("z"); 
        double max_depth = this->get_parameter("max_vision_distance").as_double();
        pass.setFilterLimits(0.0, max_depth); 
        pass.filter(*cloud_cropped);

        if (cloud_cropped->points.empty()) return; // SAFETY CHECK

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sor(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud_cropped);
        sor.setMeanK(50); 
        sor.setStddevMulThresh(1.0); 
        sor.filter(*cloud_sor);

        if (cloud_sor->points.empty()) return; // SAFETY CHECK

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_smoothed(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
        mls.setInputCloud(cloud_sor);
        mls.setSearchRadius(0.3); 
        mls.setPolynomialOrder(2);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr mls_tree(new pcl::search::KdTree<pcl::PointXYZ>());
        mls.setSearchMethod(mls_tree);
        mls.process(*cloud_smoothed);

        if (cloud_smoothed->points.empty()) return; // SAFETY CHECK

        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud(cloud_smoothed);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr ne_tree(new pcl::search::KdTree<pcl::PointXYZ>());
        ne.setSearchMethod(ne_tree);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>());
        ne.setRadiusSearch(0.4); 
        ne.compute(*cloud_normals);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_obstacles(new pcl::PointCloud<pcl::PointXYZ>());
        double threshold_deg = this->get_parameter("obstacle_angle_threshold").as_double();
        double threshold_rad = threshold_deg * M_PI / 180.0;

        for (size_t i = 0; i < cloud_normals->points.size(); ++i) {
            double nz = cloud_normals->points[i].normal_z;
            if (std::isnan(nz)) continue;
            
            double angle = std::acos(std::abs(nz)); 
            if (angle > threshold_rad) {
                cloud_obstacles->points.push_back(cloud_smoothed->points[i]);
            }
        }
        
        // If there are no obstacles, publish an empty cloud so Nav2 knows the path is clear
        cloud_obstacles->width = cloud_obstacles->points.size();
        cloud_obstacles->height = 1;
        cloud_obstacles->is_dense = true;

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