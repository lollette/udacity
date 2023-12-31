// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstaclePoints(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr planePoints(new pcl::PointCloud<PointT>);

    // Extract the inliers (plane or road)
    typename pcl::ExtractIndices<PointT> extract;

    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*planePoints);

    // Extract outliers (obstacles)
    extract.setNegative (true);
    extract.filter (*obstaclePoints);
    
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstaclePoints, planePoints);
    return segResult;
}

//starter code provided by udacity, I just filled in the TODOs exercises
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());

    // create segmentation object
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    // Segment the largest planar component from the cloud
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation with PCL's RANSAC implementation took " << elapsedTime.count() << " milliseconds" << std::endl;

    //separating point clouds
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

// --- OWN IMPLEMENTATION PLANE SEGMENTATION using RANSAC--- //

//Own implementation of RANSAC for Plane fitting
template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	// TODO: Fill in this function
	// For max iterations 
	// Randomly sample subset and fit line
	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier
	// Return indicies of inliers from fitted line with most inliers
	
    // Probability of choosing an inlier (worst case)
	float inliersProba = 0.;
	int nbrIteration =  maxIterations;
	
	//Ransac iteration
	while(nbrIteration--)
	{
        std::cout<<nbrIteration<<"\n";
		// Randomly sample subset
		std::unordered_set<int> sample;
		while(sample.size() < 3) sample.insert(rand() % cloud->size());

		// fit line
		auto itr = sample.begin();
		int idx1 = *itr;
		int idx2 = *(++itr);
		int idx3 = *(++itr);
		
		float coefA = (cloud->points[idx2].y - cloud->points[idx1].y)*(cloud->points[idx3].z - cloud->points[idx1].z)
					-(cloud->points[idx2].z - cloud->points[idx1].z)*(cloud->points[idx3].y - cloud->points[idx1].y);
		float coefB = (cloud->points[idx2].z - cloud->points[idx1].z)*(cloud->points[idx3].x - cloud->points[idx1].x)
					-(cloud->points[idx2].x - cloud->points[idx1].x)*(cloud->points[idx3].z - cloud->points[idx1].z);
		float coefC = (cloud->points[idx2].x - cloud->points[idx1].x)*(cloud->points[idx3].y - cloud->points[idx1].y)
					-(cloud->points[idx2].y - cloud->points[idx1].y)*(cloud->points[idx3].x - cloud->points[idx1].x);
		float coefD = -(coefA*cloud->points[idx1].x + coefB*cloud->points[idx1].y + coefC*cloud->points[idx1].z);
		
		// Distance
		double distance;
		for(size_t idx=0; idx<cloud->size();idx++)
		{
			if(sample.count(idx)) continue;
			
			distance = fabs((coefA*cloud->points[idx].x + coefB*cloud->points[idx].y + coefC*cloud->points[idx].z + coefD) / sqrt(coefA*coefA + coefB*coefB + coefC*coefC));
			
			// If distance is smaller than threshold count it as inlier
			if (distance < distanceTol) sample.insert(idx);
		}

		// Return indicies of inliers from fitted line with most inliers
        std::cout<<sample.size() <<"   "<<inliersResult.size()<<"\n";
		if(sample.size() > inliersResult.size())
		{
			inliersResult = sample;

			// as mentioned in the course, The more inliers our data contains 
			// the higher the probability of selecting inliers to fit the model to, 
			// and the fewer iterations we need to get a high probability of selecting a good model.
			// so I've implemented the adaptive version to update the number of iteration according to the ratio of inliers.
			
			// Adaptive Parameter Calculation
			float inliersProba = (float)inliersResult.size()/ cloud->size();
            std::cout<<inliersProba<<"\n";

			// if(inliersRatio > inliersProba)
			// {
				// inliersProba = inliersRatio;

				// 99% chance of getting a pure-inlier sample
				nbrIteration = ceil(log(1 - 0.99) / log(1 - pow(inliersProba,3)));
			// }
		}
	}
	return inliersResult;
}

//Modification of SegmentPlane function to use RansacPlane function
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::MySegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    //segment the planar component from the cloud
    std::unordered_set<int> planeInliers = RansacPlane(cloud, maxIterations, distanceThreshold);

    // separate obstacles from plane
    typename pcl::PointCloud<PointT>::Ptr obstaclePoints(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr planePoints(new pcl::PointCloud<PointT>);

	for(int idx = 0; idx < cloud->size(); idx++)
	{
		if(planeInliers.count(idx))
			planePoints->points.push_back(cloud->points[idx]);
		else
			obstaclePoints->points.push_back(cloud->points[idx]);
	}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation with my own RANSAC inplementation took " << elapsedTime.count() << " milliseconds" << std::endl;

    //separating point clouds
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstaclePoints, planePoints);
    return segResult;
}

// --- end of "OWN IMPLEMENTATION PLANE SEGMENTATION using RANSAC"--- //

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}