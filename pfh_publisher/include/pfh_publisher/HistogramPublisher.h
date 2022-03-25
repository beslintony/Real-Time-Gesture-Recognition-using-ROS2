#ifndef PFH_PUBLISHER_HISTOGRAMPUBLISHER
#define PFH_PUBLISHER_HISTOGRAMPUBLISHER

#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

// PCL includes
#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>
#include <pcl/point_types.h>

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/common/geometry.h>
#include <pcl/common/io.h>
#include <pcl/common/time.h>

#include <pcl/common/transforms.h>

#include <pcl/features/normal_3d.h>

#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>
#include <pcl/io/point_cloud_image_extractors.h>
#include <pcl/io/vtk_lib_io.h>

#include <pcl/kdtree/kdtree.h>

#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_planar.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl_conversions/pcl_conversions.h>

#include "pfh_shape_reco.h"

typedef pcl::PointXYZ point_t;


using std::placeholders::_1;

class HistogramPublisher : public rclcpp::Node
{

public:
    HistogramPublisher();

    ~HistogramPublisher();

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr depth_cloud_points_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr histogram_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

    rclcpp::Logger logger_ = rclcpp::get_logger("HistogramPublisher");

    const float min_X_ = -2.5f;
    const float max_X_ = 2.5f;
    const float min_Y_ = -1.75f;
    const float max_Y_ = 2.0f;
    const float min_Z_ = 2.0f;
    const float max_Z_ = 2.3f;


    const int minimum_points_ = 300;

    int counter = 0;
    void init();

    void pointsCallback(const sensor_msgs::msg::PointCloud2::ConstPtr &msg);

    void performPreProcess(pcl::PointCloud<point_t>::Ptr &cloud);

    void downSample(pcl::PointCloud<point_t>::Ptr &cloud);

    void newConditionalRemoval(pcl::PointCloud<point_t>::Ptr &cloud_volume);

    pcl::ConditionalRemoval<point_t> conditionalRemovalVOI();

    void removeNaN(pcl::PointCloud<point_t>::Ptr &cloud);

    void getHands(const pcl::PointCloud<point_t>::ConstPtr &in_cloud, pcl::PointCloud<point_t>::Ptr &out_cloud);

    void getHistogram(const pcl::PointCloud<point_t>::Ptr &cloud);

    void transformCloud(pcl::PointCloud<point_t>::Ptr &cloud);

    pcl::PointCloud<pcl::Normal>::Ptr getCloudNormal(const pcl::PointCloud<point_t>::Ptr &cloud);
};

#endif //PFH_PUBLISHER_HISTOGRAMPUBLISHER