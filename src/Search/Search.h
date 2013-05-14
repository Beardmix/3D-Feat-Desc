/*! \file Search.h
    \brief All the functions related to signatures Matching
*/

#ifndef SEARCH_H_INCLUDED
#define SEARCH_H_INCLUDED

#include "../defines.h"
#include "../Match/Match.h"

#include <vector>
#include <Eigen/Eigen>

/*! \class Search
   * \brief Class searching for the best point
   *
   *  The class searches for the point of interest
   */
class Search {
	public: 
		// Constructors
		
		/*! \brief Default Constructor */
		Search();
		/*! \brief Loaded Constructor
			\param cloud The cloud containing the point of interest
			\param parameters The array containing the parameters (number of sphere, cones and slices...)
		*/
		Search(PCLCloud cloud, int * parameters, long ind_picked);
		~Search();
		
		// Accessors
		void setPoint(long ind_start){ind = ind_start;}
		void setReference(long index);
		
		// Public Methods
		void generateGrid(int type, int density);
		void clearGrid();
		std::vector<long> runSA(long ind_start);
		std::vector<long> runGrid();
		
	protected:
		// Members
		PCLCloud cloud; /*!< cloud containing the point of interest */
		int	* parameters; /*!< parameters The array containing the 5 parameters (Number of Spheres/Cones/Slices / Radius of the biggest Sphere (mm) / order (0-1)) */
		long ind; /*!< ind The current point being compared */
  	Match * match;
		
		// SA Members
		long ind_old; /*!< ind_old The index of the last Picked Point */
		int max_dist; /*!< max_dist The maximum distance reachable */
		float temperature; /*!< temperature The current temperature */
		float deltaE; 
  	float ene;
  	float ene_old; 
		
		// Grid Members
		Eigen::Vector3d colorHSL;
		long best;
		float min;
		float diff;
		std::vector<long> grid;
		
		// Private Methods
};


#endif // SEARCH_H_INCLUDED
