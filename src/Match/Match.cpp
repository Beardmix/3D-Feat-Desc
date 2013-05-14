#include "Match.h"

// Constructors
Match::Match() : ind_reference(0), ind_point(0), diff1(0.0), diff2(0.0)
{
}
Match::Match(PCLCloud cloud, int * parameters) : cloud_(cloud), parameters_(parameters), ind_reference(0), ind_point(0), diff1(0.0), diff2(0.0)
{
	sig_reference = Signature (cloud, parameters);
	sig_point = Signature (cloud, parameters);
}

float
Match::getDiff(int layer)
{
	switch (layer){
		case 1:
			return diff1;
		case 2:
			return diff2;
		default:
			return 0.0;
	}
}
		
void // Sets the comparing point
Match::setPoint(long ind_point)
{
	this->ind_point = ind_point;
	sig_point.setPoint(ind_point);
}


void // Sets the reference point
Match::setReference(long ind_reference)
{
	this->ind_reference = ind_reference;
	sig_reference.setPoint(ind_reference);
	sig_reference.computeLayer1();
	sig_reference.computeLayer2();
	sig_reference.computeHue();
}
void // Saves the reference point
Match::saveReference()
{
	sig_reference.save();
}
int // Sets the reference point from a saved file
Match::setReference(std::string filename)
{
	filename = "../../RefSignatures/" + filename;
	BoostVector1Dbl layer1 = BoostVector1Dbl (new Vector1Dbl);
	BoostVector1Dbl layer2 = BoostVector1Dbl (new Vector1Dbl);
  std::string line;
  std::ifstream myfile (filename.c_str());
  if (myfile.is_open())
  {
  	// Header
		for (int i = 0; i < 6; i ++)
		{
    	getline (myfile, line);
    	if ( atoi(line.c_str()) != parameters_[i] )
    	{
  			std::cout << "Wrong Parameters: " << filename << std::endl; 
				myfile.close();
				return 1;
    	}
		}
		// Separator
    getline (myfile, line);
		// Layer 1
		for (int i = 0; i < parameters_[0] * parameters_[1] * parameters_[2]; i ++)
		{
			getline (myfile, line);
			layer1->push_back(atof(line.c_str()));
		}
		// Separator
    getline (myfile,line);
		// Layer 2
		for (int i = 0; i < parameters_[0] * parameters_[1] * parameters_[5]; i ++)
		{
			getline (myfile, line);
			layer2->push_back(atof(line.c_str()));
		}
    myfile.close();
  }
  else
  {
  	std::cout << "Unable to open signature file: " << filename << std::endl; 
  	return 2;
  }
  
  
  sig_reference.setLayer1(layer1);
  sig_reference.setLayer2(layer2);
 	return 0;
}

void // Compares two points signatures
Match::compare()
{
	compareHue();
	compareLayer1();
	compareLayer2();
}
void Match::compareLayer(int layer)
{
	switch (layer){
		case 1:
			compareLayer1();
			break;
		case 2:
			sig_point.computeLayer1();
			compareLayer2();
			break;
		default:
			compareLayer1();
			compareLayer2();
	}
}
void // Compares two points signatures
Match::compareLayer1()
{
	sig_point.computeLayer1();
	Vector1Dbl signaturea = *(sig_reference.getLayer1());
	Vector1Dbl signatureb = *(sig_point.getLayer1());
	float integral1 = 0, integral2 = 0;	
	float decim = 1000.0;
	for (size_t i = 0; i < signaturea.size() ; i++)
	{
		integral1 += signaturea[i] + signatureb[i];
		integral2 += fabs( decim * (signaturea)[i] - decim * (signatureb)[i]) / decim;
	}	
	diff1 = 100.0 * integral2 / integral1;
}
void // Compares two points signatures
Match::compareLayer2()
{
	sig_point.computeLayer2();
	Vector1Dbl signaturea = *(sig_reference.getLayer2());
	Vector1Dbl signatureb = *(sig_point.getLayer2());
	float integral1 = 0, integral2 = 0;	
	float decim = 1000.0;
	for (size_t i = 0; i < signaturea.size() ; i++)
	{
		integral1 += signaturea[i] + signatureb[i];
		integral2 += fabs( decim * (signaturea)[i] - decim * (signatureb)[i]) / decim;
	}	
	diff2 = 100.0 * integral2 / integral1;
}
void // Compares two points hues
Match::compareHue()
{
	sig_point.computeHue();
	double hue_point = sig_point.getHue();
	double hue_ref = sig_reference.getHue();
	if (hue_point < 0 || hue_ref < 0)
	{
		if (hue_point == hue_ref )
		{
			diffHue = 0;
		}
		else
		{
			diffHue = 1;
		}
	}
	else 
	{
		diffHue = (float)fabs(hue_point - hue_ref);
		if (diffHue > 0.5)
			diffHue = 1 - diffHue;
	}
}





