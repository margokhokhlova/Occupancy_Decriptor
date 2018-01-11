#include "Descriptor_op.h"




// FUNCTION TO TRANSLATE THE POINT CLOUD BY Z coordinate only
template <typename PointT> inline void TranslatePointCloud (const pcl::PointCloud<PointT> &cloud,  pcl::PointCloud<PointT> &transformed_cloud, Eigen::Vector4f centroid,    PointXYZ min_point) {
 	// TRANSLATE ONE POINT CLOUD TO THE OTHER
   Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity(); //define an identity matrix
    // Define a translation for z axis only *to be adapted in case of using the other axes*.
    transform_1 (1,3) = centroid[1]+min_point.y;
	//transform the cloud
	pcl::transformPointCloud (cloud, transformed_cloud, transform_1);
}

template <typename Point_T> inline void TranslatePointCloudToZero (const pcl::PointCloud<Point_T> &cloud,  pcl::PointCloud<Point_T> &transformed_cloud, Eigen::Vector4f centroid) {
 	// TRANSLATE ONE POINT CLOUD TO THE OTHER
   Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity(); //define an identity matrix
    // Define a translation for z axis only *to be adapted in case of using the other axes*.
	transform_1 (0,3) = -centroid[0]; //x y z translation
    transform_1 (2,3) = -centroid[2];
	//transform the cloud
	pcl::transformPointCloud (cloud, transformed_cloud, transform_1);
}








bool Descriptor_op::Initialize() {
//***************************************
// zero all the bin of the histogramm 
/*  to initialize the counter */
{ 
	if(n_sections <= 0 || n_sectors <= 0 || n_circles <= 0 )
	{
		cout <<"Parameters initialization: "<< n_sections << " " << n_sectors << " " << n_circles << " " <<  "\n"; 
		return false;
	}

	descriptor = new std::vector<float>**[n_sections];
	descriptorPointCounter = new int**[n_sections];

	for(int i=0;i<n_sections;i++)
	{
		descriptor[i] = new std::vector<float>*[n_sectors];
		descriptorPointCounter[i] = new int*[n_sectors];

		for(int j=0;j<n_sectors;j++)
		{
			descriptor[i][j] = new std::vector<float>[n_circles];
			descriptorPointCounter[i][j] = new int[n_circles];

			for(int d=0;d<n_circles;d++)
			{
				descriptor[i][j][d].resize(1);
				descriptorPointCounter[i][j][d] = 0;
			}
		}
	}

	
	return true;
}




}



void Descriptor_op::deleteDescriptor() {
	for (int i = 0; i < n_sections; ++i) 
	{
		for (int j = 0; j < n_sectors; ++j)
		{
			for(int d=0;d<n_circles;d++)
				std::vector<float>(descriptor[i][j][d]).swap(descriptor[i][j][d]);

			delete [] descriptor[i][j];
		}

		delete [] descriptor[i];
	}
	delete [] descriptor;

// CloudPtr.reset(); 	// delete CloudPtr; deallocated automatically    //Cloud.~PointCloud();

}



void Descriptor_op::writeDescriptor(std::string fileName)
{
	std::ofstream myfile;
	myfile.open (fileName);


	

	int index = 0;
	for(int i=0;i<n_sections;i++)
		for(int j=0;j<n_sectors;j++)
			for(int d=0;d<n_circles;d++)
				
					myfile << index << ":" << descriptor[i][j][d][0] << " ";
					index++;
				

				myfile << "\n";
				myfile.close();
}

void Descriptor_op::normalize()
//***************************************
/* zero all the bin of the histogramm 
*  to initialize the counter */
{
	float sum=CloudPtr->size();
	for(int i=0;i<n_sections;i++)
		for(int j=0;j<n_sectors;j++)
			for(int d=0;d<n_circles;d++)
				descriptor[i][j][d][0] = descriptor[i][j][d][0]/sum;
			

		
}


//void Descriptor_op::CalculateDescriptor(/*std::string fileName*/) {
//
//// to initialize all the variables and translate the point cloud to the begining of the coordinates
//  
//  //estimate the mean point in order to estimate the floor
//   PointT current_min_point, max_point, center; //to store the min and max points
//   Eigen::Vector4f centroid, centroid_init; //to save the coordinates of the first center
//   pcl::compute3DCentroid<PointT>(* CloudPtr, centroid);
//  
//   //translate point cloud to zero coordinates
//   TranslatePointCloudToZero(* CloudPtr, * CloudPtr, centroid);
//
//   // Create a set of planar coefficients with X=Y=0,Z=1
//	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
//	coefficients->values.resize (4); 
//	coefficients->values[0] = 0 ;coefficients->values[1] = 1.0;  coefficients->values[2] = 0; //originally only 2nd is 1
//	coefficients->values[3] = 0;
//
//
//   //  project cloud to the ground floor
//	 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>); //point cloud used for the projection to the ground surface
//    // Create new point cloud of type XYZ to make all the projections etc etc
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cl(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::copyPointCloud(*CloudPtr,*cl);
//
//////// Create the filtering object
//  pcl::ProjectInliers<pcl::PointXYZ> proj;
//  proj.setModelType (pcl::SACMODEL_PLANE);
//  proj.setInputCloud (cl);
//  proj.setModelCoefficients (coefficients);
//  proj.filter (*cloud_projected); //project the cloud to the floor
// //
//  pcl::compute3DCentroid<pcl::PointXYZ>(*cloud_projected, centroid_init); //compute the centroid of the projection 
// //// //move the projection to the level of the ground
//  pcl::PointXYZ Min_P;
//  getMinMax3D(*CloudPtr,  current_min_point, max_point); //get the current min max, move projection to the min
//  Min_P.x=current_min_point.x;  Min_P.y=current_min_point.y;  Min_P.z=current_min_point.z; 
//  TranslatePointCloud(*cloud_projected, *cloud_projected, centroid_init,   Min_P );
// // for visualisation of the center, to be removed
//
//  // //recalculate the sphere for the centroid
//  pcl::compute3DCentroid<pcl::PointXYZ>(*cloud_projected, centroid);
//  center.x=centroid.x();   center.y=centroid.y();   center.z=centroid.z(); //because the projection is now on the ground, recalculation of the coord is made
// 
//  // // cylinder visualisation for later
//  pcl::ModelCoefficients model_cylinder_coeffs;
//  model_cylinder_coeffs.values.resize (7);    // We need 7 values
//  model_cylinder_coeffs.values[0] =centroid.x(); model_cylinder_coeffs.values[1] =centroid.y(); model_cylinder_coeffs.values[2] =centroid.z(); 
//  model_cylinder_coeffs.values[3]=0; model_cylinder_coeffs.values[4]=abs(centroid[1])+abs(max_point.y); model_cylinder_coeffs.values[5]=0; 
//  model_cylinder_coeffs.values[6]=cylinder_radius; //1 meter
//
// // // ****************************************************************************************************************************************//
// // //                                                     INITIALIZATION FOR THE HON DESCRIPTOR                                               //
// //
// // // coefficient should be translated towards the Y axis
//  coefficients->values[3] =-centroid[1]; //take the ground model before and correct the Y (so it is centered on the correct plane
// // // attempt to calculate ground based on the projection used
//  ground(0)=coefficients->values[0];  ground(1)=coefficients->values[1];  ground(2)=coefficients->values[2];  ground(3)=coefficients->values[3];  //set the ground floor parametes data
//
//	//// my attempt to change the normal -> normal is just a vector pointed to the viewing point, of the length of the radius of a cylinder
//	
//	front_face_normal(0)=0; front_face_normal(1) =ground(0); front_face_normal(2)=cylinder_radius; front_face_normal=front_face_normal/front_face_normal.norm();
//
//	////set min and max point for a given point cloud
//    MinPt=current_min_point;   MaxPt=max_point;
//
//
//
// // //display all the parameters
// // cout<<"Point cloud size: "<<Cloud.size()<<endl<<" and ground data: "<<endl<<ground<<endl<<" and front face: "<<endl<<front_face_normal<<endl<<"and Min and Max data: "<<MinPt<<" "<<MaxPt<<endl<<"and cylinder rad and height data: "<<cylinder_radius<<" "<<cylinder_height<<endl;
//
//  // ************************************************************************************************************************************************* //
//  //                                            EXAMPLE OF VISUALIZATION                                                                               //
//  //  visualization mine
//  pcl::visualization::PCLVisualizer viewer;
//  pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color( CloudPtr/*PointCl*/, rand()%255-50, rand()%255-50, rand()%255-50);
//  viewer.addPointCloud( CloudPtr, single_color, "name1");
//  //viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal> ( CloudPtr,  cloud_normals, 10, 0.5, "normals1");
//  viewer.addPointCloud(cloud_projected, "name2");
//  viewer.addSphere(center, 0.05, 255, 0, 0);
//  viewer.addCylinder(model_cylinder_coeffs);
// viewer.addPlane(*coefficients);
//  viewer.addCoordinateSystem (2.0/*, current_min_point.x, current_min_point.y, current_min_point.z*/);
// viewer.addArrow(PointXYZ(0,0,0), PointXYZ(front_face_normal(0), front_face_normal(1), front_face_normal(2)), 0, 100, 255, 1.0);
//
//  //viewer.addArrow(PointXYZ(0,0,0),PointXYZ(N(0),N(1), N(2)), 255, 255, 255, 10.0, "arrow2");
//  viewer.spinOnce(10000); 
//
//
//};
//

void Descriptor_op::SaveDescriptor() {};

bool Descriptor_op::addCloud(const PointCloud<PointT> &Cl) {
	bool allFine=false;
	if (Cl.size()>1)
	{
		Cloud=Cl;
		CloudPtr.reset (new pcl::PointCloud<PointT> (Cloud));
		allFine=true;
	}
	
	return allFine;

};



//// *********************************************************************************************************************************** //
////                       My modified version of the initial descriptor.
bool Descriptor_op::computeDescriptor()
{

	if(!this->Initialize())
	{
		cout << "error initialize()\n";
		return false;
	}
	

	
	
	//ground things initialization and plot

	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ()); 	coefficients->values.resize (4); //visualization only

     pcl::ModelCoefficients model_cylinder_coeffs; //visualisation only
     model_cylinder_coeffs.values.resize (7);    // We need 7 values


	 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>); //point cloud used for the projection to the ground surface
	
	pcl::compute3DCentroid<pcl::PointXYZ>(*CloudPtr, Center);
	TranslatePointCloudToZero(*CloudPtr, *CloudPtr, Center); 	//translate point cloud to zero coordinates

    getMinMax3D(*CloudPtr,  MinPt,   MaxPt); //get the current min max, move projection to the min
	
	//project to a plane 
	coefficients->values[0] = 0 ;coefficients->values[1] = 1.0;	coefficients->values[2] = 0; coefficients->values[3] = 0;
	pcl::ProjectInliers<pcl::PointXYZ> proj;
	proj.setModelType (pcl::SACMODEL_PLANE);
	proj.setInputCloud ( CloudPtr);
	proj.setModelCoefficients (coefficients);
	proj.filter (*cloud_projected); //project the cloud to the floor

	 pcl::compute3DCentroid<pcl::PointXYZ>(*cloud_projected, Center); //compute the centroid of the projection 
     //move the projection to the level of the ground
     getMinMax3D(*CloudPtr, MinPt,   MaxPt); //get the current min max, move projection to the min
     TranslatePointCloud(*cloud_projected, *cloud_projected, Center,  MinPt );     // for visualisation of the center, to be removed
	 //recalculate the sphere for the centroid
     pcl::compute3DCentroid<pcl::PointXYZ>(*cloud_projected, Center);
	 coefficients->values[3] =abs(Center[1]);

	 //set ground coefficients
	 ground(0)=coefficients->values[0];  ground(1)=coefficients->values[1];  ground(2)=coefficients->values[2];  ground(3)=coefficients->values[3];  //set the ground floor parametes data
	 Eigen::Vector4f base_plane(ground(0),ground(1),ground(2),ground(3));
	
	
	//get centroid point
	//============================
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*CloudPtr, centroid);
	


	//
	// base centriod point
	//======================
	pcl::PointXYZ centroid_point, cylinderBase_centroid;
	centroid_point.x = centroid(0); 
	centroid_point.y = centroid(1); 
	centroid_point.z = centroid(2);
	
	pcl::projectPoint(centroid_point,base_plane,cylinderBase_centroid); //here the center of the cylinder for projection
	Eigen::Vector3f base_centroid(cylinderBase_centroid.x,cylinderBase_centroid.y,cylinderBase_centroid.z);
	cout<<"Centroid values"<<cylinderBase_centroid<<endl;
	model_cylinder_coeffs.values[0] =cylinderBase_centroid.x; model_cylinder_coeffs.values[1] =cylinderBase_centroid.y; model_cylinder_coeffs.values[2] =cylinderBase_centroid.z; 
    model_cylinder_coeffs.values[3]=0; model_cylinder_coeffs.values[4]=cylinder_height; model_cylinder_coeffs.values[5]=0;  // abs(centroid[1])
    model_cylinder_coeffs.values[6]=cylinder_radius; 

	//base Normal
	//=======================
	Eigen::Vector3f base_normal( ground(0), ground(1), ground(2));
   //	base_normal = base_normal/base_normal.norm();
	// front face normal
	front_face_normal(0)=0.1;
	front_face_normal(1)=cylinderBase_centroid.y;
	front_face_normal(2)=0;





	//***** My part to substitute cloud with normals **** //
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud (CloudPtr);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch (0.005); //search 5 sm
	 // Compute the features
	 ne.compute (*cloud_normals);
	 //** concatenate my cloud with normals ** //
	 pcl::PointCloud<pcl::PointNormal> p_n_cloud_c;
	 pcl::concatenateFields (*CloudPtr, *cloud_normals, p_n_cloud_c); 	// cout<<"Cloud with normales size "<<p_n_cloud_c.size()<<endl;
	 pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals_p(&p_n_cloud_c);
    
	 cout<<"size with normales "<<cloud_normals_p->size()<<endl;
	
	clock_t begin = clock();
	for(int i=0;i<cloud_normals_p->size();i++)
	{

		pcl::PointNormal point = cloud_normals_p->points[i];//points[i];

		//get point height
		double point_height = pcl::pointToPlaneDistanceSigned(point,base_plane);
				
		//project point on the base
		pcl::PointNormal projected_point;
		pcl::projectPoint(point,base_plane,projected_point);
		Eigen::Vector3f projected_point_vector(projected_point.x - cylinderBase_centroid.x,projected_point.y-cylinderBase_centroid.y,projected_point.z-cylinderBase_centroid.z);
		float length = projected_point_vector.norm();
		projected_point_vector = projected_point_vector/length;
		

		//find angel 
		double dot = projected_point_vector.dot(front_face_normal);
		double det = base_normal.dot(projected_point_vector.cross(front_face_normal));
		float angle = std::atan2(det,dot);
		int angle_deg = pcl::rad2deg(angle);
		if(angle_deg < 0)
			angle_deg = angle_deg + 360;

		
		int section = point_height/section_length;
		int sector = angle_deg/sector_angleDeg;
		int circle = length/circle_interval;

	
		if(   (0 <= section) && ( section < n_sections) &&  (0 <= sector) &&  (sector < n_sectors) && (0 <= circle) && ( circle < n_circles) )
		{

				descriptor[section][sector][circle][0] += 1;

			descriptorPointCounter[section][sector][circle]++;

		}

	}

	clock_t end = clock();

//	cout<<"Visualization"<<endl;
//	vector<ModelCoefficients> Circles;
//	for (int y=0; y<n_sections-1; y++) {
//     ModelCoefficients::Ptr sect(new ModelCoefficients);
//	 sect->values.resize(7);
//      sect->values[0]=cylinderBase_centroid.x; sect->values[1]=cylinderBase_centroid.y; sect->values[2] =cylinderBase_centroid.z;
//	  sect->values[3] =0; sect->values[4]=section_length*(1+y); sect->values[5]=0; sect->values[6]=cylinder_radius; 
//	  Circles.push_back(*sect);
//	}
//	cout<<Circles.size()<<endl;
//	//// Visualization //
//	pcl::visualization::PCLVisualizer viewer;
//	for (auto it=Circles.begin(); it!=Circles.end(); it++)
//		viewer.addCylinder(*it, to_string(static_cast<long double>(it->values[4])));
//    viewer.addArrow(cylinderBase_centroid, PointXYZ(front_face_normal(0), front_face_normal(1), front_face_normal(2)), 255.0, 105.0, 0, 1.0);
////	viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal> ( CloudPtr,  cloud_normals, 1.0, 1.0, "normals1");
//	//viewer.addPointCloudNormals<PointT> (CloudPtr, 10, 1.0, "normals");
//	pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color( CloudPtr/*PointCl*/, rand()%255-50, rand()%255-50, rand()%255-50);
//    viewer.addPointCloud( CloudPtr, single_color, "name1");
//	//coefficients->values[3]=lower_d;
//	viewer.addPlane(*coefficients);
//	viewer.addSphere(cylinderBase_centroid, 0.005, 255, 0, 0);
//	viewer.addCylinder(model_cylinder_coeffs);
//	viewer.spinOnce(100); 

	this->normalize();


	return true;
}



