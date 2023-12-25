/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

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

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
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
		// Randomly sample subset
		std::unordered_set<int> sample;
		while(sample.size() < 2) sample.insert(rand() % cloud->size());

		// fit line
		auto idxA = sample.begin();
		auto idxB = idxA++;
		
		float coefA = cloud->points[*idxA].y - cloud->points[*idxB].y;
		float coefB = cloud->points[*idxB].x - cloud->points[*idxA].x;
		float coefC = (cloud->points[*idxA].x * cloud->points[*idxB].y) - (cloud->points[*idxB].x * cloud->points[*idxA].y);

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
		if(sample.size() > inliersResult.size())
		{
			inliersResult = sample;

			// Adaptive Parameter Calculation
			float inliersRatio = (float)inliersResult.size()/ cloud->size();

			if(inliersRatio > inliersProba)
			{
				inliersProba = inliersRatio;

				// 99% chance of getting a pure-inlier sample
				int newNbrIteration = ceil(log(1 - 0.99) / log(1 - pow(inliersProba,2)));
				if(newNbrIteration < nbrIteration) nbrIteration = newNbrIteration;
			}
		}
	}

	return inliersResult;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 50, 0.5);

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
