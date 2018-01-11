#include "Histogram.h"


//****************************   20 Faces  ***********************************
std::vector<float> add_element_counter_icosahedron(float nx,float ny,float nz)
//***************************************************************************
/* update the counter for the bins given by the projection of
*  the normal vector (nx,ny,nz) on an icosahedron */
{
	std::vector<float > bin;
	bin.resize(20);

	float norm = sqrt(nx*nx+ny*ny+nz*nz) ;
	float pond ;
	// list of the coordinates of the center for each face of the icosahedron
	float x_center[20] = {1,1,1,1,-1,-1,-1,-1,0,0,0,0,0.873,-0.873,0.873,-0.873,0.127,0.127,-0.127,-0.127} ;
	float y_center[20] = {1,1,-1,-1,1,1,-1,-1,0.127,0.127,-0.127,-0.127,0,0,0,0,0.873,-0.873,0.873,-0.873} ;
	float z_center[20] = {1,-1,1,-1,1,-1,1,-1,0.873,-0.873,0.873,-0.873,0.127,0.127,-0.127,-0.127,0,0,0,0} ;

	for(int b=0;b<20;b++)
	{
		pond = ( nx*x_center[b] + ny*y_center[b] + nz*z_center[b] ) / norm ;	// projection
		if(pond>0)
			bin[b] = bin[b] + pond ;
	}

	return bin;
}

//****************************   12 Faces  **********************************
std::vector<float> add_element_counter_dodecahedron(float nx,float ny,float nz)
//***************************************************************************
/* update the counter for the bins given by the projection of
*  the normal vector (nx,ny,nz) on a dodecahedron */
{
	std::vector<float > bin;
	bin.resize(12);

	float norm = sqrt(nx*nx+ny*ny+nz*nz) ;
	float pond ;
	// list of the coordinates of the center for each face of the dodecahedron
	float x_center[12] = {0.938,0.938,-0.938,-0.938,0,0,0,0,0.346,-0.346,0.346,-0.346} ;
	float y_center[12] = {0.346,-0.346,0.346,-0.346,0.938,0.938,-0.938,-0.938,0,0,0,0} ;
	float z_center[12] = {0,0,0,0,0.346,-0.346,0.346,-0.346,0.938,0.938,-0.938,-0.938} ;

	for(int b=0;b<12;b++)
	{
		pond = ( nx*x_center[b] + ny*y_center[b] + nz*z_center[b] ) / norm ;	// projection
		if(pond>0)
			bin[b] = bin[b] + pond ;
	}

	return bin;
}

//****************************   8 Faces   *********************************
std::vector<float> add_element_counter_octahedron(float nx,float ny,float nz)
//*************************************************************************
/* update the counter for the bins given by the projection of
*  the normal vector (nx,ny,nz) on an octahedron */
{
	std::vector<float > bin;
	bin.resize(8);

	float norm = sqrt(nx*nx+ny*ny+nz*nz) ;
	float pond ;
	// list of the coordinates of the center for each face of the octahedron
	float x_center[8] = {0.816,0.816,-0.816,-0.816,0,0,0,0} ;
	float y_center[8] = {0,0,0,0,0.816,0.816,-0.816,-0.816} ;
	float z_center[8] = {0.577,-0.577,0.577,-0.577,0.577,-0.577,0.577,-0.577} ;

	for(int b=0;b<8;b++)
	{
		pond = ( nx*x_center[b] + ny*y_center[b] + nz*z_center[b] ) / norm ;	// projection
		if(pond>0)
			bin[b] = bin[b] + pond ;
	}

	return bin;
}

//****************************   6 Faces   ********************************
std::vector<float> add_element_counter_cube(float nx,float ny,float nz)
//*******************************************************************
/* update the counter for the bins given by the projection of
*  the normal vector (nx,ny,nz) on a cube */
{
	std::vector<float > bin;
	bin.resize(6);

	float norm = sqrt(nx*nx+ny*ny+nz*nz) ;
	float pond ;
	// list of the coordinates of the center for each face of the cube
	float x_center[6] = {1,-1,0,0,0,0} ;
	float y_center[6] = {0,0,1,-1,0,0} ;
	float z_center[6] = {0,0,0,0,1,-1} ;

	for(int b=0;b<6;b++)
	{
		pond = ( nx*x_center[b] + ny*y_center[b] + nz*z_center[b] ) / norm ;	// projection
		if(pond>0)
			bin[b] = bin[b] + pond ;
	}

	return bin;
}

//****************************   4 Faces    **************************************
std::vector<float> add_element_counter_tetrahedron(float nx,float ny,float nz)
//**************************************************************************
/* update the counter for the bins given by the projection of
*  the normal vector (nx,ny,nz) on a tetrahedron */
{
	std::vector<float > bin;
	bin.resize(4);

	float norm = sqrt(nx*nx+ny*ny+nz*nz) ;
	float pond ;
	// list of the coordinates of the center for each face of the tetrahedron
	float x_center[4] = {0,0,0.816,-0.816} ;
	float y_center[4] = {0,0.943,-0.471,-0.471} ;
	float z_center[4] = {-1,0.333,0.333,0.333} ;

	for(int b=0;b<4;b++)
	{
		pond = ( nx*x_center[b] + ny*y_center[b] + nz*z_center[b] ) / norm ;	// projection
		if(pond>0)
			bin[b] = bin[b] + pond ;
	}

	return bin;
}
