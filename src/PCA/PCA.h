
#ifndef PCA_H_INCLUDED
#define PCA_H_INCLUDED

#include "../defines.h"

/*! \class PCA
   * \brief Class for Principal Component Analysis computation
   *
   *  The class computes the Principal Component Analysis
   */
class PCA {
	public: 
		// Constructors
		
		/*! \brief Empty Constructor
		*/
		PCA();
		
		/*! \brief Loaded Constructor
			\param cloud The cloud containing the point of interest
		*/
		PCA(PCLCloud cloud);
				
		// Accessors
		
		/*! \brief Set the sub cloud to compute the normal from
		  \param [in] sub_cloud_ind The vector of the indices of the sub cloud to compute the normal from
		*/
		void setSubCloud(std::vector<int> sub_cloud_ind);
		void setSubCloud(BoostVector2Int sub_cloud_ind);
		
		/*! \brief Gives the normal
		  \return The normal
		*/
		Eigen::Vector3d getNormal();
		
		/*! \brief Gives the second vector from the PCA
		  \return The vec_base1
		*/
		Eigen::Vector3d getVecBase1();
		
		/*! \brief Gives the third vector from the PCA
		  \return The vec_base2
		*/
		Eigen::Vector3d getVecBase2();
		
		// Public Methods
		/*! \brief Computes the normal to a set of points thanks to PCA method
		*/
		void // computes the normal to a set of points thanks to PCA method
		compute();
				
	private:
		// Members
		
		PCLCloud cloud; /*!< cloud Cloud containing the point of interest */
		std::vector<std::vector<int> > sub_cloud_ind; /*!< sub_cloud_ind Vector containing the indices of the sub cloud to use to compute the PCA */
		Eigen::Vector3d normal; /*!< normal Normal from the PCA */
		Eigen::Vector3d vec_base1; /*!< vec_base1 Second vector from the PCA */
		Eigen::Vector3d vec_base2; /*!< vec_base2 Third vector from the PCA */
		Eigen::Vector3d centre; /*!< centre The centre of the PCA */
		
		// Private methods
		/*! \brief Checks the sign of the vectors
		*/
		void // Checks the sign of the vectors
		signCheck();		
		
		/*! \brief Computes the covariance Matrix
		\param [out] covar The covariance Matrix
		*/
		void // Computes the covariance Matrix
		computeCovar(Eigen::Matrix3d & covar);
		
		/*! \brief Computes the centre of the sub cloud
		*/
		void // Computes the centre of the sub cloud
		computeCentre();
		
};



#endif // SIGNATURE_H_INCLUDED







