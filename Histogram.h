#include <iostream>
#include <vector>

#ifndef HISTOGRAM_H
#define HISTOGRAM_H

std::vector<float> add_element_counter_dodecahedron(float nx,float ny,float nz);
std::vector<float> add_element_counter_icosahedron(float,float,float) ; // !
std::vector<float> add_element_counter_octahedron(float,float,float) ; // !
std::vector<float> add_element_counter_cube(float,float,float) ;
std::vector<float> add_element_counter_tetrahedron(float,float,float) ;

#endif HISTOGRAM_H