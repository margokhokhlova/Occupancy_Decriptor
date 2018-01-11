


#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <vector>
#include <ctime>
#include <cstdlib>
#include <iostream>
#include <unordered_map>



#include <pcl/console/parse.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/mls.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/search.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "Descriptor_op.h"

// for dirs
#include "boost/filesystem.hpp"
#include <cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <io.h>
#include "opencv2/imgproc/imgproc.hpp"




bool MSR3D_dataset = true;

cv::Mat  LoadImage(std::string i1){
	cv::Mat depth_float, depth1;
	//First depth image
	depth1 = cv::imread(i1, -1);
	if (depth1.empty())
	{
		cout<<"\nThe first depth image " << i1<<" cannot be found, please check that it is in the correct folder"<<endl;
	}
	if (!MSR3D_dataset)
	depth1.convertTo(depth_float, CV_32FC1, 1.0 / 5000.0);
	else
     depth1.convertTo(depth_float, CV_32FC1, 1.0); //only for MSR3D dataset
	return depth1;
}

inline pcl::PointCloud<pcl::PointXYZ>::Ptr MatToPoinXYZ(cv::Mat1f& const depthMat)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr ptCloud (new pcl::PointCloud<pcl::PointXYZ>);
	float const fx_d=5.2743327820291904e+002/2.0;
	float const cx_d= 3.0676795108161230e+002/2.0;
	float const fy_d=5.2743327820291904e+002/2.0;
	float const cy_d=2.2258224007926410e+002/2.0;


// (fot float mat)
	int numberPoints=0;
#pragma omp parallel private (i) shared (Mi)

for(int i = 0; i < depthMat.rows; i++)
{
    const float* Mi = depthMat.ptr<float>(i);
    for(int j = 0; j < depthMat.cols; j++) {
	pcl::PointXYZ point;
	if (Mi[j]>0) {
	point.z = 0.001 * Mi[j]; //direct depth value 
    point.x = point.z*(j - cx_d)  / fx_d;
    point.y = point.z *(cy_d - i) / fy_d;
	ptCloud->points.push_back(point);
	numberPoints++;
		}
	//else {cout<<"NAN values in the point cloud!";}
	}
}
	ptCloud->width=numberPoints;
    ptCloud->height = 1; 
//	cout<<"converted to the point cloud of the size: "<<ptCloud->size()<<endl;
    return ptCloud;
}





int 
main (int argc, char *argv[])
{
std::string name;
cout << "\n test version of Occupancy descriptor for a 3D point cloud \n";
cout << "Entre a point cloud name"<<std::endl;
cin >>name; 
cv::Mat1f depth_float;

	float H_adjustable=0.30; // H
	float R_adjustable=0.18; // R

LARGE_INTEGER Total= {0}; // for time computation
cout<<"image "<<name<<endl;
depth_float=LoadImage(name);
// convert to PC
pcl::PointCloud<pcl::PointXYZ>::Ptr PointClXYZ=MatToPoinXYZ(depth_float); //convert image to depth using the calibration parameters for the MSR3d dataset

// Descriptor Part
Descriptor_op Test(12, 10, 12, R_adjustable, H_adjustable); //(int n_section, int n_sector, int n_circle, int faces, float radius, float height)
if (Test.addCloud(*PointClXYZ)!=true) {
			cout<<"error! No cloud loaded"<<endl;
			return 0;
}
cout<<Test.Initialize()<<" - is descriptor itinialized?" <<endl;
LARGE_INTEGER StartingTime, EndingTime, ElapsedMicroseconds; // to compute the time? 
LARGE_INTEGER Frequency;
QueryPerformanceFrequency(&Frequency); 
QueryPerformanceCounter(&StartingTime);

Test.computeDescriptor();	//actual computation
QueryPerformanceCounter(&EndingTime);
ElapsedMicroseconds.QuadPart = EndingTime.QuadPart - StartingTime.QuadPart;
ElapsedMicroseconds.QuadPart *= 1000000;
ElapsedMicroseconds.QuadPart /= Frequency.QuadPart;
Total.QuadPart+=ElapsedMicroseconds.QuadPart;
cout<<endl<<endl<<"Done! Elapsed Microseconds "<<Total.QuadPart/54<<endl<<endl;
//save  the descripto
std::string name_save = name.substr(0, name.size()-3)+ "txt";
cout<<"Saving the descriptor: "<<name_save<<endl;
Test.writeDescriptor(name_save);
Test.deleteDescriptor();



return 0;
}






































