#include "SplitSpace.h"

SplitSpace::SplitSpace(PCLCloud cloud, int * parameters, int ind_centre, BoostVector2Int ind_sub_cloud) 
: cloud(cloud), parameters(parameters), ind_sub_cloud(ind_sub_cloud), ind_sph(1), ind_con(1), ind_sli(0)
{
	centre = cloud->points[ind_centre];
	pca = PCA (cloud);
	pca.setSubCloud(ind_sub_cloud);
	pca.compute();
	normal = pca.getNormal();
	vec_base1 = pca.getVecBase1();
	vec_base2 = pca.getVecBase2();
	space_map = BoostVector5Int (new Vector5Int);
	init();
}
SplitSpace::SplitSpace(PCLCloud cloud, int * parameters) 
: cloud(cloud), parameters(parameters), ind_sph(1), ind_con(1), ind_sli(0)
{
	pca = PCA (cloud);
	space_map = BoostVector5Int (new Vector5Int);
	init();
}
SplitSpace::SplitSpace(){
	radius = Vector1Dbl(1);
	centres_sph = Vector1Dbl(1);
	alphas = Vector1Dbl(1);
	centres_con = Vector1Dbl(1);
}
SplitSpace::~SplitSpace()
{
}

void SplitSpace::setSubCloud(BoostVector2Int ind_sub_cloud)
{
	this->ind_sub_cloud = ind_sub_cloud;
	pca.setSubCloud(ind_sub_cloud);
	pca.compute();
	normal = pca.getNormal();
	vec_base1 = pca.getVecBase1();
	vec_base2 = pca.getVecBase2();
}

void SplitSpace::init()
{
	space_map->resize(parameters[0]);
	for (int i = 0; i < parameters[0]; i++)
	{
	  (*space_map)[i].resize(parameters[1]);
	  for (int j = 0; j < parameters[1]; j++)
	  {
	    (*space_map)[i][j].resize(parameters[2]);
			for (int k = 0; k < parameters[2]; k++)
			{
			  (*space_map)[i][j][k].resize(2);
			}
	  }
	}
	radius = Vector1Dbl(parameters[0]+1);
	centres_sph = Vector1Dbl(parameters[0]+1);
	alphas = Vector1Dbl(parameters[1]+1);
	centres_con = Vector1Dbl(parameters[1]+1);
	radius[0] = 0;
	centres_sph[0] = 0;
	for (int i = 1; i < parameters[0]+1; i ++)
	{
		radius[i] = pow( (i) / (double)parameters[0], 1 / 3.0 ) * parameters[3] / 1000.0;
		centres_sph[i] = (radius[i] + radius[i-1]) / 2.0;
	}
	coeff_sph = 0;
	alphas[0] = 0;
	centres_con[0] = 0;
	for (int i = 1; i < parameters[1]+1; i ++)
	{
		alphas[i] = M_PI/2.0 + asin(2 * i / (float)parameters[1] - 1);
		centres_con[i] = (alphas[i] + alphas[i-1]) / 2.0;
	}
	coeff_con = 0;
}

void SplitSpace::compute()
{
	for (int i = 0; i < parameters[0]; i++)
	{
	  for (int j = 0; j < parameters[1]; j++)
	  {
			for (int k = 0; k < parameters[2]; k++)
			{
			  (*space_map)[i][j][k][0].clear();
			  (*space_map)[i][j][k][1].clear();
			}
	  }
	}
	int ind_point;
	for (int i = 0; i < (int)ind_sub_cloud->at(0).size(); i ++)
	{
		ind_point = (*ind_sub_cloud)[0][i];
		point = cloud->points[ind_point];
		coordToSph();
		coordToCon(); // Get the indice of the cone
		if(parameters[2] > 1)
			coordToSli(); // Get the indice of the slice
		if (ind_sph > 1)
		{
			if (ind_con > 1)
			{
				(*space_map)[ind_sph-1][ind_con-1][ind_sli][0].push_back(ind_point);
				(*space_map)[ind_sph-2][ind_con-1][ind_sli][0].push_back(ind_point);
				(*space_map)[ind_sph-1][ind_con-2][ind_sli][0].push_back(ind_point);
				(*space_map)[ind_sph-2][ind_con-2][ind_sli][0].push_back(ind_point);
				(*space_map)[ind_sph-1][ind_con-1][ind_sli][1].push_back(coeff_sph * coeff_con / 100.0);
				(*space_map)[ind_sph-2][ind_con-1][ind_sli][1].push_back((100-coeff_sph) * coeff_con / 100.0);
				(*space_map)[ind_sph-1][ind_con-2][ind_sli][1].push_back(coeff_sph * (100-coeff_con) / 100.0);
				(*space_map)[ind_sph-2][ind_con-2][ind_sli][1].push_back((100-coeff_sph) * (100-coeff_con) / 100.0);
			}
			else
			{
				(*space_map)[ind_sph-1][ind_con-1][ind_sli][0].push_back(ind_point);
				(*space_map)[ind_sph-2][ind_con-1][ind_sli][0].push_back(ind_point);
				(*space_map)[ind_sph-1][ind_con-1][ind_sli][1].push_back(coeff_sph );
				(*space_map)[ind_sph-2][ind_con-1][ind_sli][1].push_back(100-coeff_sph);
			}
		}
		else
		{
			if (ind_con > 1)
			{
				(*space_map)[ind_sph-1][ind_con-1][ind_sli][0].push_back(ind_point);
				(*space_map)[ind_sph-1][ind_con-2][ind_sli][0].push_back(ind_point);
				(*space_map)[ind_sph-1][ind_con-1][ind_sli][1].push_back(coeff_con);
				(*space_map)[ind_sph-1][ind_con-2][ind_sli][1].push_back(100-coeff_con);
			}
			else
			{
			
				(*space_map)[ind_sph-1][ind_con-1][ind_sli][0].push_back(ind_point);
				(*space_map)[ind_sph-1][ind_con-1][ind_sli][1].push_back(100);
				
			}
		}
	}
}

void // Get the indice of the sphere
SplitSpace::coordToSph()
{
	ind_sph = 1;
	double distance = sqrt(euclDist2(point, centre));
	while (distance > centres_sph[ind_sph] && ind_sph < parameters[0])
		ind_sph++;
	coeff_sph = floor( 100 * (1 + cos(fabs(centres_sph[ind_sph] - distance) / (centres_sph[ind_sph] - centres_sph[ind_sph-1]) * M_PI) ) /2.0 );
	//coeff_sph = floor( 100 * fabs(centres_sph[ind_sph] - distance) / (centres_sph[ind_sph] - centres_sph[ind_sph-1]));
}

void // Get the indice of the cone
SplitSpace::coordToCon()
{
	ind_con = 1;
	vec_point << point.x - centre.x, point.y - centre.y, point.z - centre.z;
	if ( vec_point.norm() != 0 )
	{
		double alpha = acos( normal.dot( vec_point / vec_point.norm() ));
		while (alpha > centres_con[ind_con] && ind_con < parameters[1])
			ind_con++;
		coeff_con = floor( 100 * (1 + cos(fabs(centres_con[ind_con] - alpha) / (centres_con[ind_con] - centres_con[ind_con-1]) * M_PI) ) /2.0 );
	}
}
void // Get the indice of the slice
SplitSpace::coordToSli()
{
	int indice = 0;
	double d1 = 0, d2 = 0, beta = 0, norm = 0;
	vec_point << point.x - centre.x, point.y - centre.y, point.z - centre.z;
	d1 = vec_point.dot( vec_base1 );
	d2 = vec_point.dot( vec_base2 );
	vec_point = d1 * vec_base1 + d2 * vec_base2;
	if ( (norm = vec_point.norm()) != 0 )
	{
		beta = acos( round( d1 * 10000.0 /norm ) / 10000.0 );
		indice = (floor( parameters[2] * beta / (2 * M_PI) ));
		if ( vec_point.dot( vec_base2 ) < 0 )
			indice = parameters[2] - indice - 1;
	}
	ind_sli = indice;
}

/*
void // Get the indice of the cone
SplitSpace::coordToCon()
{
	ind_con = 1;
	vec_point << point.x - centre.x, point.y - centre.y, point.z - centre.z;
	if ( vec_point.norm() != 0 )
	{
		double alpha = acos( my_dot( normal, vec_point ) / my_norm(vec_point));
		while (alpha > centres_con[ind_con] && ind_con < parameters[1])
			ind_con++;
		coeff_con = floor( 100 * (1 + cos(fabs(centres_con[ind_con] - alpha) / (centres_con[ind_con] - centres_con[ind_con-1]) * M_PI) ) /2.0 );
	}
}
void // Get the indice of the slice
SplitSpace::coordToSli()
{
	int indice = 0;
	double d1 = 0, d2 = 0, beta = 0, norm = 0;
	vec_point << point.x - centre.x, point.y - centre.y, point.z - centre.z;
	d1 = my_dot( vec_point, vec_base1 );
	d2 = my_dot( vec_point, vec_base2 );
	vec_point = d1 * vec_base1 + d2 * vec_base2;
	if ( (norm = my_norm(vec_point)) != 0 )
	{
		beta = acos( round( d1 * 10000.0 / norm ) / 10000.0 );
		indice = (floor( parameters[2] * beta / (2 * M_PI) ));
		if ( my_dot( vec_point, vec_base2 ) < 0 )
			indice = parameters[2] - indice - 1;
	}
	ind_sli = indice;
}
*/


