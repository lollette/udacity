/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

/* quiz solution's author : Lilia Chorfi-Belhadj */

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <math.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

//Own implementation of RANSAC for line fitting
std::unordered_set<int> RansacLine(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	// TODO: Fill in this function

	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers

	int nbrIteration =  maxIterations;
	
	//Ransac iteration
	while(nbrIteration--)
	{
		// Randomly sample subset
		std::unordered_set<int> sample;
		while(sample.size() < 2) sample.insert(rand() % cloud->size());

		// fit line
		auto itr = sample.begin();
		int idx1 = *itr;
		int idx2 = *(++itr);
		
		float coefA = cloud->points[idx1].y - cloud->points[idx2].y;
		float coefB = cloud->points[idx2].x - cloud->points[idx1].x;
		float coefC = (cloud->points[idx1].x * cloud->points[idx2].y) - (cloud->points[idx2].x * cloud->points[idx1].y);

		// Distance
		double distance;
		for(size_t idx=0; idx<cloud->size();idx++)
		{
			if(sample.count(idx)) continue;
			
			distance = fabs((coefA*cloud->points[idx].x + coefB*cloud->points[idx].y + coefC) / sqrt(coefA*coefA + coefB*coefB));
			
			// If distance is smaller than threshold count it as inlier
			if (distance < distanceTol) sample.insert(idx);
		}

		// Return indicies of inliers from fitted line with most inliers
		if(sample.size() > inliersResult.size())  inliersResult = sample;
	}

	return inliersResult;
}

//Own implementation of RANSAC for Plane fitting
std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
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
	int nbrIteration =  maxIterations;
	
	//Ransac iteration
	while(nbrIteration--)
	{
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
		if(sample.size() > inliersResult.size()) inliersResult = sample;
	}

	return inliersResult;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data 2D
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();

	// Create data 3D
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	// std::unordered_set<int> inliers = RansacLine(cloud, 50, 0.5);
	std::unordered_set<int> inliers = RansacPlane(cloud, 50, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
}
