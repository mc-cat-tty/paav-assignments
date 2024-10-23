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
#include <Renderer.hpp>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <chrono>
#include <unordered_set>
#include <tree_utilities.hpp>
#include <boost/filesystem.hpp>
#include <params.hpp>

#define USE_PCL_LIBRARY
using namespace lidar_obstacle_detection;

typedef std::unordered_set<int> my_visited_set_t;

//This function sets up the custom kdtree using the point cloud
void setupKdtree(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, my_pcl::KdTree* tree, int dimension)
{
    //insert point cloud points into tree
    for (int i = 0; i < cloud->size(); ++i)
    {
        tree->insert({cloud->at(i).x, cloud->at(i).y, cloud->at(i).z}, i);
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
void proximity(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int target_ndx, my_pcl::KdTree* tree, float distanceTol, my_visited_set_t& visited, std::vector<int>& cluster, int max)
{
	if (cluster.size() < max)
    {
        cluster.push_back(target_ndx);
        visited.insert(target_ndx);

        std::vector<float> point {cloud->at(target_ndx).x, cloud->at(target_ndx).y, cloud->at(target_ndx).z};
    
        // get all neighboring indices of point
        std::vector<int> neighborNdxs = tree->search(point, distanceTol);

        for (int neighborNdx : neighborNdxs)
        {
            // if point was not visited
            if (visited.find(neighborNdx) == visited.end())
            {
                proximity(cloud, neighborNdx, tree, distanceTol, visited, cluster, max);
            }

            if (cluster.size() >= max)
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
TODO: Complete the function
*/
std::vector<pcl::PointIndices> euclideanCluster(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, my_pcl::KdTree* tree, float distanceTol, int setMinClusterSize, int setMaxClusterSize)
{
	my_visited_set_t visited{};                                                          //already visited points
	std::vector<pcl::PointIndices> clusters;                                             //vector of PointIndices that will contain all the clusters
    std::vector<int> cluster;                                                            //vector of int that is used to store the points that the function proximity will give me back
	//for every point of the cloud
    //  if the point has not been visited (use the function called "find")
    //    find clusters using the proximity function
    //
    //    if we have more clusters than the minimum
    //      Create the cluster and insert it in the vector of clusters. You can extract the indices from the cluster returned by the proximity funciton (use pcl::PointIndices)   
    //    end if
    //  end if
    //end for
	return clusters;	
}

void ProcessAndRenderPointCloud (Renderer& renderer, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    using pxyz = pcl::PointXYZ;
    auto &parameters = params::Params::getInstance();
    
    // 1) Downsample the dataset
    std::cerr
        << "[DOWNSAMPLING] Before downsampling - PC size: "
        << cloud->size()
        << " (" << pcl::getFieldsList(*cloud) << ") "
        << std::endl;

    pcl::PointCloud<pxyz>::Ptr cloud_filtered(new pcl::PointCloud<pxyz>());
    pcl::VoxelGrid<pxyz> voxel_filterer;
    voxel_filterer.setInputCloud(cloud);
    voxel_filterer.setLeafSize(parameters.voxel_leaf_size);
    voxel_filterer.filter(*cloud_filtered);

    std::cerr
        << "[DOWNSAMPLING] After downsampling - PC size: "
        << cloud_filtered->size()
        << " (" << pcl::getFieldsList(*cloud) << ") "
        << std::endl;
    
    // 2) Crop the points that are far away from us, in which we are not interested
    pcl::CropBox<pxyz> cb(true);
    cb.setInputCloud(cloud_filtered);
    cb.setMin(parameters.crop_box_min);
    cb.setMax(parameters.crop_box_max);
    cb.filter(*cloud_filtered);

    // 3) Apply RANSAC to segment ground plane    
    pcl::SACSegmentation<pxyz> segmentation;
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
    pcl::PointCloud<pxyz>::Ptr ground_plane (new pcl::PointCloud<pxyz>());
    pcl::ExtractIndices<pxyz> extract_filterer;
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
    pcl::PointCloud<pxyz>::Ptr ego_vehicle (new pcl::PointCloud<pxyz>());
    pcl::CropBox<pxyz> ego_box(true);
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
    

    // TODO: 6) Create the KDTree and the vector of PointIndices
    


    // TODO: 7) Set the spatial tolerance for new cluster candidates (pay attention to the tolerance!!!)
    std::vector<pcl::PointIndices> cluster_indices;

    if (parameters.render_raw_pc) renderer.RenderPointCloud(cloud, "originalCloud");
    if (parameters.render_filtered_pc) renderer.RenderPointCloud(cloud_filtered, "filteredCloud");

    #ifdef USE_PCL_LIBRARY

        //PCL functions
        //HERE 6)
    #else
        // Optional assignment
        my_pcl::KdTree treeM;
        treeM.set_dimension(3);
        setupKdtree(cloud_filtered, &treeM, 3);
        cluster_indices = euclideanCluster(cloud_filtered, &treeM, clusterTolerance, setMinClusterSize, setMaxClusterSize);
    #endif

    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1), Color(1,0,1), Color(0,1,1)};


    /**Now we extracted the clusters out of our point cloud and saved the indices in cluster_indices. 

    To separate each cluster out of the vector<PointIndices> we have to iterate through cluster_indices, create a new PointCloud for each entry and write all points of the current cluster in the PointCloud.
    Compute euclidean distance
    **/
    int j = 0;
    int clusterId = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        cloud_cluster->push_back ((*cloud_filtered)[*pit]); 
        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        renderer.RenderPointCloud(cloud,"originalCloud"+std::to_string(clusterId),colors[2]);
        // TODO: 7) render the cluster and plane without rendering the original cloud 
        //<-- here
        //----------

        //Here we create the bounding box on the detected clusters
        pcl::PointXYZ minPt, maxPt;
        pcl::getMinMax3D(*cloud_cluster, minPt, maxPt);

        //TODO: 8) Here you can plot the distance of each cluster w.r.t ego vehicle
        Box box{minPt.x, minPt.y, minPt.z,
        maxPt.x, maxPt.y, maxPt.z};
        //TODO: 9) Here you can color the vehicles that are both in front and 5 meters away from the ego vehicle
        //please take a look at the function RenderBox to see how to color the box
        renderer.RenderBox(box, j);

        ++clusterId;
        j++;
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
