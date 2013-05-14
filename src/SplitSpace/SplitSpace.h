/*! \file SplitSpace.h
    \brief All the functions related to Space Splitting
*/

#ifndef SPACESPLIT_H_INCLUDED
#define SPACESPLIT_H_INCLUDED

#include "../defines.h"
#include "../common/common.h"
#include "../PCA/PCA.h"

#include <Eigen/Eigen>


/*! \class SplitSpace
   * \brief Class for signature computation and storage
   *
   *  The class computes and stores a signature
   */
class SplitSpace {
	public: 
		// Constructors
		
		/*! \brief Loaded Constructor
			\param [in] centre Point of Interest
			\param [in] normal The normal of the point of interest
			\param [in] vec_base1 1st 3D vector used to generate a base
			\param [in] vec_base2 2nd 3D vector used to generate a base
			\param [in] parameters The array containing the parameters (number of sphere, cones and slices...)
		*/
		SplitSpace(PCLCloud cloud, int * parameters, int ind_centre, BoostVector2Int ind_sub_cloud);
		SplitSpace(PCLCloud cloud, int * parameters);
		SplitSpace();
		~SplitSpace();
		
		// Accessors
		BoostVector5Int getMap(){return space_map;}
		
		void setCentre(int ind_centre){centre = cloud->points[ind_centre];}
		void setSubCloud(BoostVector2Int ind_sub_cloud);
				
		// Public Methods
		/*! \brief Computes the indices thanks to the parameters set
		*/
		void compute();

	private:
		// Attributes
		PCLCloud cloud;
		int	* parameters; /*!< parameters The array containing the parameters */
		PCLPoint centre; /*!< centre Point of Interest */
		BoostVector2Int	ind_sub_cloud;
		int ind_sph; /*!< ind_sph Indice of the Sphere in which the point is */
		int ind_con; /*!< ind_con Indice of the Cone in which the point is */
		int ind_sli; /*!< ind_sli Indice of the Slice in which the point is */
		PCLPoint point; /*!< point Selected Point to check */
		Eigen::Vector3d normal; /*!< normal The normal of the point of interest */
		Eigen::Vector3d vec_base1; /*!< vec_base1 1st 3D vector used to generate a base */
		Eigen::Vector3d vec_base2; /*!< vec_base2 2nd 3D vector used to generate a base */
		Eigen::Vector3d vec_point;
		BoostVector5Int space_map; /*!< space_map The number of points in each slice */
		Vector1Dbl radius;
		Vector1Dbl centres_sph;
		int coeff_sph;
		Vector1Dbl alphas;
		Vector1Dbl centres_con;
		int coeff_con;
		PCA pca;
		
		// Private methods
		
		/*! \brief Get the indice of the sphere in which the given point is
		  \return An integer corresponding to the indice of the sphere in which the given point is
		*/
		void // Get the indice of the sphere
		coordToSph();
		
		/*! \brief Get the indice of the cone in which the given point is
    \return An int corresponding to the indice of the cone in which the given point is
		*/
		void // Get the indice of the cone
		coordToCon();
		
		/*! \brief Get the indice of the slice in which the given point is
    \return An int corresponding to the indice of the slice in which the given point is
		*/
		void // Get the indice of the slice
		coordToSli();
		
		
		void init();

};
		
#endif // SPACESPLIT_H_INCLUDED




