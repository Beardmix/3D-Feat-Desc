#include "Signature.h"
#include "../common/common.h"

// Constructors
Signature::Signature()
{ 
}
Signature::Signature(PCLCloud cloud, int * parameters) : ind_point(0), cloud(cloud), parameters(parameters)
{
	layer1 = BoostVector1Dbl (new Vector1Dbl);
	layer2 = BoostVector1Dbl (new Vector1Dbl);
	ind_sub_cloud = BoostVector2Int (new Vector2Int); // Declaration of the ind_sub_cloud that keeps the indices of the subcloud 
	pca = PCA (cloud);
	sp = SplitSpace(cloud, parameters);
	pts_per_sli.resize(parameters[0]);
	for (int i = 0; i < parameters[0]; i++)
	{
	  pts_per_sli[i].resize(parameters[1]);
	  for (int j = 0; j < parameters[1]; j++)
	  {
	    pts_per_sli[i][j].resize(parameters[2]);
			for (int k = 0; k < parameters[2]; k++)
			{
			  pts_per_sli[i][j][k].resize(2);
			}
	  }
	}	
}

// Accessors
BoostVector1Dbl Signature::getLayer1(){return layer1;}
BoostVector1Dbl Signature::getLayer2(){return layer2;}
void Signature::setPoint(long ind_point){this->ind_point = ind_point;}
void Signature::setLayer1(BoostVector1Dbl layer1){this->layer1 = layer1;}
void Signature::setLayer2(BoostVector1Dbl layer2){this->layer2 = layer2;}

void // Transforms a repartition vector into signature
Signature::vecToSignature()
{
	layer1->clear();
	float max=0;
	double value = 0;
	for (int sph = 0; sph < parameters[0]; sph++)
	{
		for (int con = 0; con < parameters[1]; con++)
		{
  		if(parameters[4]!=0)
  			std::sort(pts_per_sli[sph][con].begin(), pts_per_sli[sph][con].end(), sortFunction);
			for (int sli = 0; sli < parameters[2]; sli++)
			{
				value = 0;
				for (int ind = 0; ind < (int)pts_per_sli[sph][con][sli][0].size(); ind ++)
				{
					value += pts_per_sli[sph][con][sli][1][ind] / 100.0;
				}
				if (pts_per_sli[sph][con][sli][0].size() != 0)
					value /= (double) pts_per_sli[sph][con][sli][0].size();
				layer1->push_back(value);
				if (value > max)
					max = value;
			}
		}
	}
	if (max != 0)
		for (size_t i = 0; i < layer1->size(); i++)
			(*layer1)[i] /= max;
}

void // Clears & Resizes pts_per_sli
Signature::clearPtsSli()
{
	for (int i = 0; i < parameters[0]; i++)
	{
	  for (int j = 0; j < parameters[1]; j++)
	  {
			for (int k = 0; k < parameters[2]; k++)
			{
				pts_per_sli[i][j][k][0].clear();
				pts_per_sli[i][j][k][1].clear();
			}
		}
	}
}

void // Generates the sub-cloud around the point of interest
Signature::generateSubCloud(BoostVector2Int ind_sub_cloud)
{	
	BoostVector1Int	ind_1st_sub_cloud (new Vector1Int);
	PCLPoint centre = cloud->points[ind_point]; // Declaration of the point of interest
	PCLPoint point; // Declaration of a point used to compute
	
	float delta_x_2, delta_y_2, delta_z_2, r_max_2 = pow( parameters[3] / 1000.0, 2 );
		
	// Study of only a piece of the cloud
	
	long dist = 0;
	do{
		dist += 2;
		if (ind_point - (dist+2) * (cloud->width +1) < 0 || ind_point + (dist+2) * (cloud->width +1) > (long)cloud->size())
			dist = 0;	
	}while(	dist != 0 && 
					(	euclDist2(cloud->points[ind_point - dist], cloud->points[ind_point]) < r_max_2 ||
						euclDist2(cloud->points[ind_point - dist * cloud->width], cloud->points[ind_point]) < r_max_2));
	if (dist != 0)
		dist += 2;
	
	for (long i = - dist; i < dist; i++)
	{		
		for (long j = - dist; j < dist; j++)
		{
			long indice = (ind_point + i + j * cloud->width) % (long)cloud->size();
			(*ind_1st_sub_cloud).push_back( indice );
		}
	}
	for (size_t i = 0; i < ind_1st_sub_cloud->size(); i+=10)
	{		
		point = cloud->points[(*ind_1st_sub_cloud)[i]];
		
		if (point.z == point.z){
			if (	(delta_x_2=pow(centre.x-point.x, 2)) < r_max_2 && 
						(delta_y_2=pow(centre.y-point.y, 2)) < r_max_2 && 
						(delta_z_2=pow(centre.z-point.z, 2)) < r_max_2 &&
						(delta_x_2 + delta_y_2 + delta_z_2 ) < r_max_2 )
			{
				(*ind_sub_cloud)[0].push_back( (*ind_1st_sub_cloud)[i] );	
				(*ind_sub_cloud)[1].push_back( 100 - 0.5 * floor(100 * (delta_x_2 + delta_y_2 + delta_z_2 ) / r_max_2) );	
			}
		}
	}	
	
}
void // Counts the number of points into each slice
Signature::countPtsPerSli()
{

	ind_sub_cloud->clear();
	ind_sub_cloud->resize(2);
	
	// Generate a sub-cloud of interest
	generateSubCloud(ind_sub_cloud);	
	
	// If the sub-cloud generated is not null
	if (ind_sub_cloud->at(0).size() > 0)
	{
		pca.setSubCloud(ind_sub_cloud);
		pca.compute();
		
		// Put the points into the right slices
		sp.setCentre (ind_point);
		sp.setSubCloud(ind_sub_cloud);
		sp.compute();
		pts_per_sli = *sp.getMap();
	}
}

void // Compute the normal to every slice (with more than 3 points)
Signature::normOfSli()
{	
	Vector1Dbl ring;
	double mu = 0;
	double max = 0.1;
	Eigen::Vector3d normal = pca.getNormal();
	PCA pcaLocal(cloud);
	for (int sph = 0; sph < parameters[0]; sph++)
	{
		for (int con = 0; con < parameters[1]; con++)
		{
			ring.clear();
			ring.resize(parameters[5], 0);
			for (int sli = 0; sli < parameters[2]; sli++)
			{
				if (pts_per_sli[sph][con][sli][0].size()>=3)
				{
					pcaLocal.setSubCloud(pts_per_sli[sph][con][sli][0]);
					pcaLocal.compute();
					mu = acos( pcaLocal.getNormal().dot( normal ) ); // Angle between Normal and local bin Normal
					ring[ floor( parameters[5] * mu / (M_PI + 0.0001) ) ] ++; // Angle Quantification
				}
			}
			// sort the angles
			std::sort(ring.begin(),ring.end());
			// max for normalization
			for (int i = 0; i < parameters[5]; i ++)
				if (ring[i] > max)
					max = ring[i];
			// copy of ring
			for (int i = 0; i < parameters[5]; i ++)
				layer2->push_back( ring[i]/max );
		}
	}
}

void // Computes a point's 1st layer signature
Signature::computeLayer1()
{
	if (cloud->points[ind_point].z == cloud->points[ind_point].z)
	{
		clearPtsSli();
		countPtsPerSli();
		vecToSignature();	
	}
	else
	{
		std::cerr << "Warning Signature::computeLayer1 NaN depth Point" << std::endl; 
	}
}

void // Computes a point's 2nd layer signature
Signature::computeLayer2()
{
	layer2->clear();
	normOfSli();
}

void Signature::computeHue()
{
	int nb_points = 0;
	Eigen::Vector3d colorHSL;
	for (int i = -1; i < 2; i ++)
	{
		for (int j = -1; j < 2; j ++)
		{
			long index = ind_point + i + j * cloud->width;
			if (index > 0 && index < (long)cloud->size())
			{
				nb_points++; 
				colorHSL = rgb_to_hsl( cloud->points[index].getRGBVector3i() );
				hue += colorHSL(0);
			}
		}
	}
	hue = hue / nb_points;
	colorHSL = rgb_to_hsl( cloud->points[ind_point].getRGBVector3i() );
	hue = colorHSL(0);
	if (isGrey(colorHSL))
		hue = -1;
	if (isWhite(colorHSL))
		hue = -2;
	if (isBlack(colorHSL))
		hue = -3;
}

std::string // Generates file name from date and time
Signature::generateFileName() {
	char buf[80];
	time_t now = time(0);
	strftime(buf, sizeof(buf), "../../RefSignatures/Sig-%d:%m:%Y-%X.txt", localtime(&now));
	return buf;
}

void // Saves the signature
Signature::save()
{
	std::ofstream myfile;
  myfile.open (generateFileName().c_str());
  // Header
  for (int i = 0; i < 6; i ++)
  {
  	myfile << parameters[i] <<"\n";
  }
	// Separator
	myfile << "******** Layer 1 ********" <<"\n";
  // Layer 1
  for (int i = 0; i < parameters[0] * parameters[1] * parameters[2]; i ++)
  {
  	myfile << (*layer1)[i] <<"\n";
  }
	// Separator
	myfile << "******** Layer 2 ********" <<"\n";
  // Layer 2
  for (int i = 0; i < parameters[0] * parameters[1] * parameters[5]; i ++)
  {
  	myfile << (*layer2)[i] <<"\n";
  }
  myfile.close();
}

bool // sort function
sortFunction (const std::vector<std::vector<int> > &i, const std::vector<std::vector<int> > &j) 
{ 
	return (i[0].size() < j[0].size()); 
}


