/*! \file signature.h
    \brief All the functions related to signatures
*/

#ifndef SIGNATURE_H_INCLUDED
#define SIGNATURE_H_INCLUDED

#include "../defines.h"
#include "../common/common.h"
#include "../PCA/PCA.h"
#include "../SplitSpace/SplitSpace.h"

#include <iostream>
#include <vector>

/*! \class Signature
   * \brief Class for signature computation and storage
   *
   *  The class computes and stores a signature
   */
class Signature {
	public: 
		// Constructors
		
		/*! \brief Default Constructor */
		Signature();
		/*! \brief Loaded Constructor
			\param cloud The cloud containing the point of interest
			\param parameters The array containing the parameters (number of sphere, cones and slices...)
		*/
		Signature(PCLCloud cloud, int * parameters);
		
		// Accessors
		
		/*! \brief Set the indice of the point of interest
		  \param [in] ind_point The indice of the point of interest
		*/
		void setPoint(long ind_point);
		
		/*! \brief Gives the signature previously computed
		  \return The signature of the specified point
		*/
		BoostVector1Dbl getLayer1();
		
		/*! \brief Gives the second layer signature previously computed
		  \return The second layer signature of the specified point
		*/
		BoostVector1Dbl getLayer2();
		
		double getHue(){return hue;}
		
		/*! \brief Sets the signature previously computed
		  \param layer1 The signature of the specified point
		*/
		void
		setLayer1(BoostVector1Dbl layer1);
		
		/*! \brief Sets the second layer signature previously computed
		  \param layer2 The second layer signature of the specified point
		*/
		void
		setLayer2(BoostVector1Dbl layer2);
		
		
		// Public Methods
		
		/*! \brief Computes a point's 1st layer signature
		*/
		void // Computes a point's first layer signature (layer 1 = nb points)
		computeLayer1();
		
		/*! \brief Computes a point's 2nd layer signature
		*/
		void // Computes a point's second layer signature (layer 2 = normals)
		computeLayer2();
		
		void computeHue();
		
		/*! \brief Saves the signature
		*/
		void // Saves the signature
		save();
		
	private:
		// Members
		
		long ind_point; /*!< ind_point Indice of the point of interest */
		PCLCloud cloud; /*!< cloud Cloud containing the point of interest */
		int	* parameters; /*!< parameters The array containing the 5 parameters (Number of Spheres/Cones/Slices / Radius of the biggest Sphere (mm) / order (0-1)) */
		BoostVector1Dbl layer1; /*!< layer1 The Layer1 of the given point of interest */
		BoostVector1Dbl layer2; /*!< layer2 The second layer signature of the given point of interest */
		Vector5Int pts_per_sli; /*!< pts_per_sli The number of points in each slice */
		BoostVector2Int	ind_sub_cloud;
		PCA pca; /*!< pca The result of the PCA applied to the point of interest */
		SplitSpace sp;
		double hue;
		
		// Private methods
		
		/*! \brief Transforms a repartition vector into signature
		*/
		void // Transforms a repartition vector into signature
		vecToSignature();	
		
		/*! \brief Clears & Resizes pts_per_sli
		*/
		void // Clears & Resizes pts_per_sli
		clearPtsSli();
		
		/*! \brief Counts the number of points into each slice
		*/
		void // Counts the number of points into each slice
		countPtsPerSli();
		
		/*! \brief Generates the sub-cloud around the point of interest
			\param [out] ind_sub_cloud Indices of the points of the sub-cloud
		*/
		void // Generates the sub-cloud around the point of interest
		generateSubCloud(BoostVector2Int ind_sub_cloud);
		
		/*! \brief Compute the normal of every slice (with more than 3 points)
		*/
		void // Compute the normal of every slice (with more than 3 points)
		normOfSli();
		
		/*! \brief Generates file name from date and time
		  \return The file name
		*/
		std::string // Generates file name from date and time
		generateFileName();
		
		
};

// External functions

/*! \brief Function used to sort a std::vector of std::vector<int> by comparing the size of two std::vector<int> \a i & \a j
  \return A bool corresponding to the result: true if \a i is strictly shorter than \a j
  \param [in] i The first std::vector<int>
  \param [in] j The second std::vector<int>
*/
bool // sort function
sortFunction (const std::vector<std::vector<int> > &i, const std::vector<std::vector<int> > &j);


#endif // SIGNATURE_H_INCLUDED







