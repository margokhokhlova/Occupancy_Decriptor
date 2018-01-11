#include <stdlib.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/common/pca.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/distances.h>
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/features/normal_3d.h>
#include<pcl/point_types_conversion.h>
#include <pcl/common/angles.h>
#include <iostream>
#include <pcl/filters/voxel_grid.h>

#include "Histogram.h" //file for the histograms




using std::vector;


using namespace pcl;
using namespace std;

//typedef boost::shared_ptr<pcl::PointXYZ> PointCloudPtr;

class Descriptor_op {
private:
std::vector<float> ***descriptor;
int ***descriptorPointCounter;

typedef pcl::PointXYZ PointT;
PointCloud<PointT> Cloud; //cloud
PointCloud<PointT>::Ptr CloudPtr; //pointer to my cloud
int ncols;
int nrows;


// parameters for division
	int n_sections;
	int n_sectors;
	int n_circles;


//parameter for cylinder on the floor projection
	float cylinder_radius;
	float cylinder_height;
	//caculated parameters
	float section_length;
	float circle_interval;
	float sector_angleDeg;






// floor estimation on the dataset
	Eigen::Vector4f ground;
// constant front face direction
	Eigen::Vector3f front_face_normal;

Eigen::Vector4f  Center;
PointT MinPt,   MaxPt;

public:


//alternative consturctor to use
Descriptor_op::Descriptor_op(int n_section, int n_sector, int n_circle, float radius, float height){
	this->n_sections =n_section;
	this->n_sectors = n_sector;
	this->n_circles = n_circle;


	this->cylinder_radius = radius;
	this->cylinder_height=height;
	section_length = cylinder_height/n_sections;
	circle_interval = cylinder_radius/n_circles;
	sector_angleDeg = 360/n_sectors;
	int b_size = n_sections*n_sectors*n_circles;
}



void normalize(); /** \brief  normalize.*/
bool Initialize(); /* zero all the bin of the histogramm to initialize the counter */
void deleteDescriptor();
void writeDescriptor(std::string fileName);
//void CalculateDescriptor(/*std::string fileName*/);
bool computeDescriptor(); //modified by me
void SaveDescriptor();
 bool addCloud(const PointCloud<PointT> &Cl);

//void drawDescriptor(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);

};
