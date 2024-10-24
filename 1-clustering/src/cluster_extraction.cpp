#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/crop_box.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <boost/filesystem.hpp>

#include <Renderer.hpp>
#include <params.hpp>
#include <utils.hpp>
#include <tree_utilities.hpp>

#include <chrono>
#include <unordered_set>
#include <sstream>

#define USE_PCL_LIBRARY
using namespace lidar_obstacle_detection;

typedef std::unordered_set<int> my_visited_set_t;

//This function sets up the custom kdtree using the point cloud
void setupKdtree(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, my_pcl::KdTree &tree, int dimension)
{
    //insert point cloud points into tree
    for (int i = 0; i < cloud->size(); ++i)
    {
        tree.insert({cloud->at(i).x, cloud->at(i).y, cloud->at(i).z}, i);
    }
}

/*
OPTIONAL
This function computes the nearest neighbors and builds the clusters
    - Input:
        + cloud: Point cloud to be explored
        + target_ndx: i-th point to visit
        + tree: kd tree for searching neighbors
        + distanceTol: Distance tolerance to build the clusters 
        + visited: Visited points --> typedef std::unordered_set<int> my_visited_set_t;
        + cluster: Here we add points that will represent the cluster
        + max: Max cluster size
    - Output:
        + visited: already visited points
        + cluster: at the end of this function we will have one cluster
*/
void proximity(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, int target_ndx, my_pcl::KdTree &tree, float distanceTol, my_visited_set_t& visited, pcl::PointIndices& cluster, int max)
{
	if (cluster.indices.size() < max) {
        cluster.indices.push_back(target_ndx);
        visited.insert(target_ndx);

        std::vector<float> point{cloud->at(target_ndx).x, cloud->at(target_ndx).y, cloud->at(target_ndx).z};
    
        // get all neighboring indices of point
        std::vector<int> neighborNdxs = tree.search(point, distanceTol);

        for (int neighborNdx : neighborNdxs) {
            // if point was not visited
            if (visited.find(neighborNdx) == visited.end())
            {
                proximity(cloud, neighborNdx, tree, distanceTol, visited, cluster, max);
            }

            if (cluster.indices.size() >= max)
            {
                return;
            }
        }
    }
}

/*
OPTIONAL
This function builds the clusters following a euclidean clustering approach
    - Input:
        + cloud: Point cloud to be explored
        + tree: kd tree for searching neighbors
        + distanceTol: Distance tolerance to build the clusters 
        + setMinClusterSize: Minimum cluster size
        + setMaxClusterSize: Max cluster size
    - Output:
        + cluster: at the end of this function we will have a set of clusters
*/
std::vector<pcl::PointIndices> euclideanCluster(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, my_pcl::KdTree &tree, float distanceTol, int setMinClusterSize, int setMaxClusterSize) {
	my_visited_set_t visited{};                 //already visited points
	std::vector<pcl::PointIndices> clusters;    //vector of PointIndices that will contain all the clusters
    pcl::PointIndices cluster_idxs;             //vector of int that is used to store the points that the function proximity will give me back
	
    for (unsigned idx = 0; idx < cloud->size(); ++idx) {
        if (visited.find(idx) == visited.end()) {
            // Whenever a new point is encountered, it is marked as visited
            visited.insert(idx);

            // Then a cluster is built from that point
            proximity(cloud, idx, tree, distanceTol, visited, cluster_idxs, setMaxClusterSize);
            
            // If the cluster size is above the minimum threshold (max not checked since already done in proximity function)
            if (cluster_idxs.indices.size() >= setMinClusterSize) {
                clusters.emplace_back(cluster_idxs);
            }
            // Clusters that are too small are discarded.
            
            // The cluster indexes are cleared at the end of the iteration anyway
            cluster_idxs.indices.clear();
        }
    }

	return clusters;	
}

void ProcessAndRenderPointCloud (Renderer& renderer, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    using Pxyz = pcl::PointXYZ;
    const auto &parameters = params::Params::getInstance();
    
    // 1) Downsample the dataset
    std::cerr
        << "[DOWNSAMPLING] Before downsampling - PC size: "
        << cloud->size()
        << " (" << pcl::getFieldsList(*cloud) << ") "
        << std::endl;

    pcl::PointCloud<Pxyz>::Ptr cloud_filtered(new pcl::PointCloud<Pxyz>());
    pcl::VoxelGrid<Pxyz> voxel_filterer;
    voxel_filterer.setInputCloud(cloud);
    voxel_filterer.setLeafSize(parameters.voxel_leaf_size);
    voxel_filterer.filter(*cloud_filtered);

    std::cerr
        << "[DOWNSAMPLING] After downsampling - PC size: "
        << cloud_filtered->size()
        << " (" << pcl::getFieldsList(*cloud) << ") "
        << std::endl;
    
    // 2) Crop the points that are far away from us, in which we are not interested
    pcl::CropBox<Pxyz> cb(true);
    cb.setInputCloud(cloud_filtered);
    cb.setMin(parameters.crop_box_min);
    cb.setMax(parameters.crop_box_max);
    cb.filter(*cloud_filtered);

    // 3) Apply RANSAC to segment ground plane    
    pcl::SACSegmentation<Pxyz> segmentation;
    segmentation.setOptimizeCoefficients(true);
    segmentation.setModelType(pcl::SACMODEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setMaxIterations(parameters.ransac_max_iterations);
    segmentation.setDistanceThreshold(parameters.inlier_admission_threshold);
    segmentation.setInputCloud(cloud_filtered);
    
    pcl::PointIndices::Ptr inliers_idx(new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());

    auto startTime = std::chrono::steady_clock::now();
    segmentation.segment(*inliers_idx, *coefficients);
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
    std::cout
        << "[SEGMENTATION] Ransac execution time [us]: "
        << elapsedTime.count()
        << std:: endl;

    // 4) Remove the planar inliers
    pcl::PointCloud<Pxyz>::Ptr ground_plane (new pcl::PointCloud<Pxyz>());
    pcl::ExtractIndices<Pxyz> extract_filterer;
    extract_filterer.setInputCloud(cloud_filtered);
    extract_filterer.setIndices(inliers_idx);

    // It could be useful to display which is the removed set of points,
    // however, in the vast majority of cases, it's an unwanted additional overhead.
    // So its rendering has been parametrized; in that way the choice is up to the user.
    if (parameters.render_ground) {
        extract_filterer.setNegative(false);
        extract_filterer.filter(*ground_plane);
        renderer.addGroundCloud(ground_plane);
    }
        
    extract_filterer.setNegative(true);  // At this stage we want to remove the points' indexes of the groud plane
    extract_filterer.filter(*cloud_filtered);

    // 5) Remove observer's artifacts from the scene
    // It has been noticed that some points are still at the center of the scene.
    // These points were part of the ego vehicle captured by the LiDAR.
    // A purely geometric strategy has been chosen to clean them out.
    pcl::PointCloud<Pxyz>::Ptr ego_vehicle (new pcl::PointCloud<Pxyz>());
    pcl::CropBox<Pxyz> ego_box(true);
    ego_box.setInputCloud(cloud_filtered);
    ego_box.setMin(parameters.ego_box_min);
    ego_box.setMax(parameters.ego_box_max);
    
    if (parameters.render_ego) {
        ego_box.setNegative(false);
        ego_box.filter(*ego_vehicle);
        renderer.addEgoCloud(ego_vehicle);
    }

    ego_box.setNegative(true);
    ego_box.filter(*cloud_filtered);
    

    // 6) Create the KDTree and run euclidean clustering
    std::vector<pcl::PointIndices> clusters_idxs;

    if (not parameters.use_custom_clustering) {
        pcl::search::KdTree<Pxyz>::Ptr kdtree(new pcl::search::KdTree<Pxyz>());
        kdtree->setInputCloud(cloud_filtered);

        pcl::EuclideanClusterExtraction<Pxyz> clustering_extractor;
        clustering_extractor.setClusterTolerance(parameters.cluster_tolerance);
        clustering_extractor.setMinClusterSize(parameters.cluster_min_threshold);
        clustering_extractor.setMaxClusterSize(parameters.cluster_max_threshold);
        clustering_extractor.setSearchMethod(kdtree);
        clustering_extractor.setInputCloud(cloud_filtered);

        clustering_extractor.extract(clusters_idxs);
    }
    else {
        my_pcl::KdTree treeM;
        treeM.set_dimension(3);
        setupKdtree(cloud_filtered, treeM, 3);
        clusters_idxs = euclideanCluster(cloud_filtered, treeM, parameters.cluster_tolerance, parameters.cluster_min_threshold, parameters.cluster_max_threshold);
    }

    // 7) Render the scene. See above for ground plane and ego vehicle.
    if (parameters.render_raw_pc) renderer.RenderPointCloud(cloud, "originalCloud");
    if (parameters.render_filtered_pc) renderer.RenderPointCloud(cloud_filtered, "filteredCloud");

    if (clusters_idxs.empty()) {
        std::cerr << "No cluster found in the scene, maybe some params tuning is necessary" << std::endl;
    }

    /**
    To separate each cluster out of the vector<PointIndices> we have to iterate through clusters_idxs,
    create a new PointCloud for each entry and write all points of the current cluster in the PointCloud.
    Compute euclidean distance wrt the ego vehicle.
    **/
    unsigned id = 0;
    for (const auto &cluster_idxs : clusters_idxs) {
        pcl::PointCloud<Pxyz>::Ptr cloud_cluster(new pcl::PointCloud<Pxyz>());

        for (const auto &idx : cluster_idxs.indices) {
            cloud_cluster->push_back ((*cloud_filtered)[idx]); 
        }

        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        auto red = Color(1, 0, 0);
        auto green = Color(0, 1, 0);
        auto is_dangerous = utils::is_dangerous_obstacle(cloud_cluster);
        Color vehicle_color = is_dangerous ? red : green;

        //Here we create the bounding box on the detected clusters
        pcl::PointXYZ minPt, maxPt;
        pcl::getMinMax3D(*cloud_cluster, minPt, maxPt);
        Box box{minPt.x, minPt.y, minPt.z, maxPt.x, maxPt.y, maxPt.z};

        const bool discard = utils::is_building(minPt, maxPt) or utils::is_tree(minPt, maxPt);
        if (discard) continue;
        
        renderer.RenderPointCloud(cloud_cluster, "Cluster" + std::to_string(id), vehicle_color);
        renderer.RenderBox(box, id, vehicle_color);

        // 8) Here you can plot the distance of each cluster w.r.t ego vehicle
        auto distance = utils::distance_from_origin(*cloud_cluster);
        auto distance_label_stream = std::ostringstream();
        distance_label_stream << std::setprecision(3) << distance;
        renderer.addText(minPt.x, (maxPt.y+minPt.y)/2, maxPt.z, distance_label_stream.str() + " m", "Text " + std::to_string(id));

        ++id;
    }

}


int main(int argc, char* argv[])
{
    auto dataset_folder = argv[1];
    std::string params_filename = argv[2];
    Renderer renderer;

    renderer.InitCamera(CameraAngle::XY);
    // Clear viewer
    renderer.ClearViewer();

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    std::vector<boost::filesystem::path> stream(
        boost::filesystem::directory_iterator{dataset_folder},
        boost::filesystem::directory_iterator{}
    );

    auto &parameters = params::Params::getInstance();
    parameters.loadFromJson(params_filename);


    // sort files in ascending (chronological) order
    std::sort(stream.begin(), stream.end());

    auto streamIterator = stream.begin();

    while (not renderer.WasViewerStopped())
    {
        renderer.ClearViewer();

        pcl::PCDReader reader;
        reader.read(streamIterator->string(), *input_cloud);

        auto startTime = std::chrono::steady_clock::now();
        ProcessAndRenderPointCloud(renderer,input_cloud);
        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        
        std::cout
            << "[PointCloudProcessor<PointT>::ReadPcdFile] Loaded "
            << input_cloud->points.size() << " data points from " << streamIterator->string()
            <<  " plane segmentation took " << elapsedTime.count() << " milliseconds"
            << std::endl;

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        renderer.SpinViewerOnce();
    }
}
