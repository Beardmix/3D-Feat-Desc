/*! \file main.cpp
    \brief File containing the main function
*/

/*!
	\mainpage This project creates 3D Feature Descriptors For Non-Rigid Objects Recognition
	
	Mathieu Bulliot \n
	Cardiff University \n
	Engineering School
*/

#include "defines.h"



/*! \brief Main function that calls the others
    \param [in] argc The number of arguments
    \param [in] argv The array containing the arguments
*/
int
main (int argc, char** argv)
{   
  // Parameters
  int a, b, c, d, e, f;
  char * str;
  if (argc > 1)
  {
    str = argv[1]; a = atoi(argv[2]); b = atoi(argv[3]); c = atoi(argv[4]); d = atoi(argv[5]); e = atoi(argv[6]); f = atoi(argv[7]);
  }
  else
  {
  	str = (char *) "cloud11"; a = 5; b = 10; c = 10; d = 50; e = 1; f = 5;
  }
  /* parameters 
  The array containing the 6 parameters (Number of Spheres/Cones/Slices / Radius of the biggest Sphere (mm) / order (0-1)) / Nb of bins in histograms
  */
  int parameters [6] = {a, b, c, d, e, f};
  std::cout << "syntax: <Cloud Name> <Nb Spheres> <Nb Cones> <Nb Slices> <Radius Max> <Ordering by Cones> <Nb Bins>" << std::endl;
  std::cout << "default: ./exe cloud11 5 10 10 50 1 5" << std::endl;
  std::cout << "parameters:    "<< str <<" "<< a <<" "<< b <<" "<< c <<" "<< d <<" "<< e <<" "<< f << std::endl;


	
  return (0);
}

// Right Arm: 121882
// Left Hand: 160408
// Left Leg: 173817
