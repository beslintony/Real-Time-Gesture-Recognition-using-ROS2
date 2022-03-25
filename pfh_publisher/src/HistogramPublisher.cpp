#include "../include/pfh_publisher/HistogramPublisher.h"

HistogramPublisher::HistogramPublisher()
    : Node("HistogramPublisher")
{
    init();
}

HistogramPublisher::~HistogramPublisher()
{
    // stop ros instance
    rclcpp::shutdown();
}

void HistogramPublisher::init()
{
    auto sub_callback_ = [this](sensor_msgs::msg::PointCloud2::ConstPtr msg)
    {
        this->pointsCallback(msg);
    };
    depth_cloud_points_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("pcd", 1, sub_callback_);

    histogram_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/pf_histogram", 1);

    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pf_cloud", 1);
}

void HistogramPublisher::pointsCallback(const sensor_msgs::msg::PointCloud2::ConstPtr &msg)
{
    // convert point cloud2 to point cloud<>
    pcl::PointCloud<point_t> cloud;
    pcl::fromROSMsg(*msg, cloud);
    pcl::PointCloud<point_t>::Ptr ptrCloud(new pcl::PointCloud<point_t>(cloud));
    performPreProcess(ptrCloud);
}

void HistogramPublisher::performPreProcess(pcl::PointCloud<point_t>::Ptr &ptrCloud)
{
    sensor_msgs::msg::PointCloud2 out_cloud;

    // step 1: down sample cloud because clustering is slow on big point clouds
    downSample(ptrCloud);
    // Step 2: Use predefined VoI (conditional removal)
    // Set up VoI PCL Pointer
    pcl::PointCloud<point_t>::Ptr voi_cloud(new pcl::PointCloud<point_t>());
    pcl::copyPointCloud(*ptrCloud, *voi_cloud);

    newConditionalRemoval(voi_cloud);
    // Step 3: Remove NaN from point cloud

    removeNaN(voi_cloud);

    // Step 4: Cluster the filtered VoI-cloud to extract hand voxels
    if (static_cast<int>(voi_cloud->points.size()) > 0)
    {

        // Set up output cloud after clustering
        pcl::PointCloud<point_t>::Ptr hand_cluster(new pcl::PointCloud<point_t>());
        getHands(voi_cloud, hand_cluster);

        pcl::toROSMsg(*hand_cluster, out_cloud);
        cloud_pub_->publish(out_cloud);

        if (hand_cluster->size() > minimum_points_)
        {
            ++counter;
            getHistogram(voi_cloud);
        }
    }
}

void HistogramPublisher::transformCloud(pcl::PointCloud<point_t>::Ptr &cloud)
{
    Eigen::Affine3f transform_x = Eigen::Affine3f::Identity();
    transform_x.rotate(Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitX()));

    Eigen::Affine3f transform_y = Eigen::Affine3f::Identity();
    transform_y.rotate(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitY()));

    pcl::transformPointCloud(*cloud, *cloud, transform_x);
    pcl::transformPointCloud(*cloud, *cloud, transform_y);
}

void HistogramPublisher::downSample(pcl::PointCloud<point_t>::Ptr &cloud)
{
    // Down sampling of the point cloud
    // Create a voxel grid and temporary Point Cloud
    pcl::VoxelGrid<point_t> voxel_grid;
    pcl::PointCloud<point_t>::Ptr cloud_filtered(new pcl::PointCloud<point_t>());

    // Set up down sampling parameters
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(0.007f, 0.007f, 0.007f);
    voxel_grid.filter(*cloud_filtered);
    *cloud = *cloud_filtered;
}

void HistogramPublisher::newConditionalRemoval(pcl::PointCloud<point_t>::Ptr &cloud_volume)
{
    // Specify conditional removal object for cropping the input cloud to a desired size
    pcl::ConditionAnd<point_t>::Ptr range_condition(new pcl::ConditionAnd<point_t>());

    // Add range conditions
    range_condition->addComparison(pcl::FieldComparison<point_t>::ConstPtr(
        new pcl::FieldComparison<point_t>("x", pcl::ComparisonOps::GE, min_X_)));
    range_condition->addComparison(pcl::FieldComparison<point_t>::ConstPtr(
        new pcl::FieldComparison<point_t>("x", pcl::ComparisonOps::LE, max_X_)));

    range_condition->addComparison(pcl::FieldComparison<point_t>::ConstPtr(
        new pcl::FieldComparison<point_t>("y", pcl::ComparisonOps::GE, min_Y_)));
    range_condition->addComparison(pcl::FieldComparison<point_t>::ConstPtr(
        new pcl::FieldComparison<point_t>("y", pcl::ComparisonOps::LE, max_Y_)));

    range_condition->addComparison(pcl::FieldComparison<point_t>::ConstPtr(
        new pcl::FieldComparison<point_t>("z", pcl::ComparisonOps::GE, min_Z_)));
    range_condition->addComparison(pcl::FieldComparison<point_t>::ConstPtr(
        new pcl::FieldComparison<point_t>("z", pcl::ComparisonOps::LE, max_Z_)));

    // Create Conditional Removal Object
    pcl::ConditionalRemoval<point_t> cond_removed_obj;
    cond_removed_obj.setCondition(range_condition);

    cond_removed_obj.setInputCloud(cloud_volume);
    cond_removed_obj.filter(*cloud_volume);
}

pcl::ConditionalRemoval<point_t> HistogramPublisher::conditionalRemovalVOI()
{
    // Specify conditional removal object for cropping the input cloud to a desired size
    pcl::ConditionAnd<point_t>::Ptr range_condition(new pcl::ConditionAnd<point_t>());

    // Add range conditions
    range_condition->addComparison(pcl::FieldComparison<point_t>::ConstPtr(
        new pcl::FieldComparison<point_t>("x", pcl::ComparisonOps::GE, min_X_)));
    range_condition->addComparison(pcl::FieldComparison<point_t>::ConstPtr(
        new pcl::FieldComparison<point_t>("x", pcl::ComparisonOps::LE, max_X_)));

    range_condition->addComparison(pcl::FieldComparison<point_t>::ConstPtr(
        new pcl::FieldComparison<point_t>("y", pcl::ComparisonOps::GE, min_Y_)));
    range_condition->addComparison(pcl::FieldComparison<point_t>::ConstPtr(
        new pcl::FieldComparison<point_t>("y", pcl::ComparisonOps::LE, max_Y_)));

    range_condition->addComparison(pcl::FieldComparison<point_t>::ConstPtr(
        new pcl::FieldComparison<point_t>("z", pcl::ComparisonOps::GE, min_Z_)));
    range_condition->addComparison(pcl::FieldComparison<point_t>::ConstPtr(
        new pcl::FieldComparison<point_t>("z", pcl::ComparisonOps::LE, max_Z_)));

    // Create Conditional Removal Object
    pcl::ConditionalRemoval<point_t> cond_removed_obj;
    cond_removed_obj.setCondition(range_condition);

    return cond_removed_obj;
}

void HistogramPublisher::removeNaN(pcl::PointCloud<point_t>::Ptr &cloud)
{
    // Remove NaN-Indices while maintaining the structure
    std::vector<int> index;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, index);
}

void HistogramPublisher::getHands(const pcl::PointCloud<point_t>::ConstPtr &in_cloud,
                                  pcl::PointCloud<point_t>::Ptr &out_cloud)
{
    // Set up KD-Tree
    pcl::search::KdTree<point_t>::Ptr tree(new pcl::search::KdTree<point_t>);

    // Variable to store all clusters
    std::vector<pcl::PointCloud<point_t>::Ptr> cluster;

    point_t camera_origin;
    camera_origin.x = 0;
    camera_origin.y = 0;
    camera_origin.z = 0;

    // Centroid vector of all cluster
    std::vector<point_t> centroids;

    // Use Kd-Tree for structuring the cloud - for faster searching
    tree->setInputCloud(in_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<point_t> ec;

    ec.setClusterTolerance(0.02);
    ec.setMinClusterSize(20);
    ec.setMaxClusterSize(10000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(in_cloud);
    ec.extract(cluster_indices);
    // cluster_indices contains indices for all clusters

    // Iterate over extracted cluster indices
    int cctr = 0; // cluster counter
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
         it != cluster_indices.end(); ++it)
    {
        // Temporary point cloud for calculating the center of the cloud
        pcl::PointCloud<point_t>::Ptr cloud_cluster(new pcl::PointCloud<point_t>());

        // fill temp cloud with values of indices of current cluster
        for (std::vector<int>::const_iterator point_it = it->indices.begin();
             point_it != it->indices.end(); ++point_it)
        {
            cloud_cluster->push_back(in_cloud->points[*point_it]);
        }

        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        // Get centroid for current cluster
        point_t center;
        pcl::computeCentroid(*cloud_cluster, center);

        // Append cluster and corresponding centroid to result vectors
        centroids.push_back(center);
        cluster.push_back(cloud_cluster);

        cctr++;
    }

    // Only do something if there is at least 1 cluster
    if (centroids.size() > 0)
    {
        // Calculate distance to camera_origin for every cluster_centroid
        std::vector<float> dist(centroids.size());
        float l2;
        for (size_t i = 0; i < centroids.size(); i++)
        {
            l2 = sqrt(((centroids.at(i).x - camera_origin.x) * (centroids.at(i).x - camera_origin.x)) +
                      ((centroids.at(i).y - camera_origin.y) * (centroids.at(i).y - camera_origin.y)) +
                      ((centroids.at(i).z - camera_origin.z) * (centroids.at(i).z - camera_origin.z)));
            dist.at(i) = l2;
        }

        int min_index;
        int second_min_index;
        float minIndexDist1 = std::numeric_limits<float>::max();
        float minIndexDist2 = std::numeric_limits<float>::max();

        for (size_t i = 0; i < dist.size(); i++)
        {
            if (dist.at(i) < minIndexDist1)
            {
                min_index = i;
                minIndexDist1 = dist.at(i);
            }
        }

        if (centroids.size() > 1)
        {
            // temporarily save the distance at minimum position
            float tmp_min = dist.at(min_index);
            // assign a very big number at min_index so it won't be considered looking for the next nearest
            dist.at(min_index) = std::numeric_limits<float>::max();

            for (size_t i = 0; i < dist.size(); i++)
            {
                if (dist.at(i) < minIndexDist2)
                {
                    second_min_index = i;
                    minIndexDist2 = dist.at(i);
                }
            }

            // restore overwritten value for min_index
            dist.at(min_index) = tmp_min;
        }

        // Take nearest cluster and filter input cloud with indices extracted from these cluster indices
        std::vector<int> cluster_index = cluster_indices.at(min_index).indices;

        pcl::copyPointCloud<point_t>(*in_cloud, cluster_index, *out_cloud);
    }
}

void HistogramPublisher::getHistogram(const pcl::PointCloud<point_t>::Ptr &cloud)
{
    pcl::PointCloud<point_t>::Ptr cloud_ptr(new pcl::PointCloud<point_t>(*cloud));

    // calculate point cloud normal
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;
    cloud_normals = getCloudNormal(cloud);

    // instantiate PFHShapeReco
    PFHShapeReco<point_t> pfhsr;
    pfhsr.ComputePFH(0.5, cloud_ptr, cloud_normals);
    std::vector<float> hist_plc = pfhsr.GetObjectModel(); // get histogram

    // pf_histogram publication
    std_msgs::msg::Float32MultiArray histogram_pcl;
    histogram_pcl.data.clear();
    for (int i = 0; i < hist_plc.size(); ++i)
    {
        histogram_pcl.data.push_back(hist_plc[i]);
    }
    histogram_pub_->publish(histogram_pcl);
}

pcl::PointCloud<pcl::Normal>::Ptr HistogramPublisher::getCloudNormal(const pcl::PointCloud<point_t>::Ptr &points)
{
    // create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<point_t, pcl::Normal> norm_est;

    // output datasets
    pcl::PointCloud<pcl::Normal>::Ptr normals_out(new pcl::PointCloud<pcl::Normal>);

    // set the input points
    norm_est.setInputCloud(points);

    // use a FLANN-based KdTree to perform neighborhood searches
    pcl::search::KdTree<point_t>::Ptr tree(new pcl::search::KdTree<point_t>());
    norm_est.setSearchMethod(tree);

    // use all neighbors in a sphere of radius 3cm
    norm_est.setRadiusSearch(0.03);

    // estimate the surface normals and store the result in "normals_out"
    norm_est.compute(*normals_out);

    return normals_out;
}
