/*! \file Match.h
    \brief All the functions related to signatures Matching
*/

#ifndef MATCH_H_INCLUDED
#define MATCH_H_INCLUDED

#include "../defines.h"
#include "../Signature/Signature.h"

#include <iostream>

/*! \class Match
   * \brief Class dealing with signatures matching
   *
   *  The class stores and match signatures
   */
class Match {
	public: 
		// Constructors
		
		/*! \brief Default Constructor */
		Match();
		/*! \brief Loaded Constructor
			\param cloud The cloud containing the point of interest
			\param parameters The array containing the parameters (number of sphere, cones and slices...)
		*/
		Match(PCLCloud cloud, int * parameters);
		
		// Accessors
		/*! \brief Gives the indice of the reference point
		  \return The indice of the reference point
		*/
		long
		getIndRef(){return (ind_reference);}
		
		/*! \brief Gives the single value of the difference between the two signatures - Layer1
		  \return The single value difference
		*/
		float
		getDiff1(){return diff1;}
		
		/*! \brief Gives the single value of the difference between the two signatures - Layer2
		  \return The single value difference
		*/
		float
		getDiff2(){return diff2;}
		
		float
		getDiffHue(){return diffHue;}
		
		/*! \brief Gives the single value of the difference between the two signatures
			\param layer Layer to get the difference from
		  \return The single value difference
		*/
		float
		getDiff(int layer);
		
		/*! \brief Gives the signature of the reference point
		  \return The signature of the reference point
		*/
		Signature * getSigReference(){return &sig_reference;}
		BoostVector1Dbl getSigRefL1(){return sig_reference.getLayer1();}
		BoostVector1Dbl getSigRefL2(){return sig_reference.getLayer2();}
		
		/*! \brief Gives the signature of the comparing point
		  \return The signature of the comparing point
		*/
		Signature * getSigPoint(){return &sig_point;}
		BoostVector1Dbl getSigPointL1(){return sig_point.getLayer1();}
		BoostVector1Dbl getSigPointL2(){return sig_point.getLayer2();}
		
		
		/*! \brief Sets the comparing point
		  \param [in] ind_point The comparing point
		*/
		void // Sets the comparing point
		setPoint(long ind_point);


		/*! \brief Sets the reference point
		  \param [in] ind_reference The reference point
		*/
		void // Sets the reference point
		setReference(long ind_reference);
		
		/*! \brief Sets the reference point from a saved file
	  \param [in] filename The name of the file to load the signature from
		*/	
		int // Sets the reference point from a saved file
		setReference(std::string filename);
		
		/*! \brief Saves the reference point
		*/
		void // Saves the reference point
		saveReference();
		

		// Public Methods
		
		/*! \brief Compares two points signatures
		*/		
		void // Compares two points signatures
		compare();
		/*! \brief Compares two points signatures
		*/		
		void // Compares two points signatures
		compareLayer1();
		/*! \brief Compares two points signatures
		*/		
		void // Compares two points signatures
		compareLayer2();
		/*! \brief Compares two points signatures
			\param layer Layer to compare (0 = all)
		*/		
		void // Compares two points signatures
		compareLayer(int layer);
		void // Compares two points hues
		compareHue();
		
	private:
		// Attributes
		PCLCloud cloud_; /*!< cloud containing the point of interest */
		int	* parameters_; /*!< parameters The array containing the 5 parameters (Number of Spheres/Cones/Slices / Radius of the biggest Sphere (mm) / order (0-1)) */
		long ind_reference; /*!< ind_reference indice of the reference point */
		long ind_point; /*!< ind_point indice of the comparing point */
		Signature sig_reference; /*!< sig_reference Signature of the reference point */
		Signature sig_point; /*!< sig_reference Signature of the comparing point */
		float diff1; /*!< diff1 Single value of difference between the two signatures - Layer 1 */
		float diff2; /*!< diff2 Single value of difference between the two signatures - Layer 2 */
		float diffHue; /*!< diffHue Single value of difference between the two signatures - Hue */
		
		// Private Methods
};


#endif // MATCH_H_INCLUDED
