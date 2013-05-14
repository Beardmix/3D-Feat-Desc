#include "PCA.h"

// Constructors

PCA::PCA()
{
}
PCA::PCA(PCLCloud cloud)
{
	this->cloud = cloud;
}

// Accessors
void PCA::setSubCloud(std::vector<int> sub_cloud_ind)
{
	this->sub_cloud_ind.clear();
	this->sub_cloud_ind.resize(2);
	for (size_t i = 0; i < sub_cloud_ind.size(); i ++)
	{
		this->sub_cloud_ind[0].push_back( sub_cloud_ind[i] );
		this->sub_cloud_ind[1].push_back( 1 );
	}
	if (this->sub_cloud_ind[0].size() == 0)
		std::cerr << "Warning PCA::setSubCloud empty output vector" << std::endl; 
}
void PCA::setSubCloud(BoostVector2Int sub_cloud_ind)
{
	this->sub_cloud_ind.clear();
	this->sub_cloud_ind.resize(2);
	for (size_t i = 0; i < sub_cloud_ind->size(); i ++)
	{
		this->sub_cloud_ind[0].push_back( (*sub_cloud_ind)[0][i] );
		this->sub_cloud_ind[1].push_back( (*sub_cloud_ind)[1][i] );
	}
	if (this->sub_cloud_ind[0].size() == 0)
		std::cerr << "Warning PCA::setSubCloud empty output vector" << std::endl; 
}

Eigen::Vector3d PCA::getNormal()
{
	return normal;
}

Eigen::Vector3d PCA::getVecBase1()
{
	return vec_base1;
}

Eigen::Vector3d PCA::getVecBase2()
{
	return vec_base2;
}

// Public Methods
void // computes the normal to a set of points thanks to PCA method
PCA::compute()
{
	computeCentre();
	
	Eigen::Matrix3d covar;
	computeCovar( covar );
	
	// Compute Eigenvectors & Eigenvalues
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(covar);

	// Order the vectors by eigenvalues
	int order [3] = {0,1,2};
	int temp = 0;
	for(size_t i = 0; i < 3; i++)
	{
		for (size_t j = i; j < 3; j++)
		{
			if( (eigensolver.eigenvalues())(i) > (eigensolver.eigenvalues())(j) )
			{
				temp = order [j];
				order [j] = order [i];
				order [i] = temp;
			}
		}
	}
	normal = ( eigensolver.eigenvectors() ).col( order[0] );
	
	vec_base1 = ( eigensolver.eigenvectors() ).col( order[1] ) ;
	
	signCheck();
		
	vec_base2 = normal.cross( vec_base1 );
	
	if (normal.norm()!=normal.norm()){
		std::cerr << "Error PCA::compute NaN Normal: " << std::endl; 
		std::cerr << normal << std::endl; 
	}
	if (normal.norm() == 0)
		std::cerr << "Error PCA::compute Null Normal" << std::endl; 
}


void // Computes the centre of the sub cloud
PCA::computeCentre()
{
	int nb_points = 0;
	double sum = 0;
	// Compute the means
	for(size_t i = 0; i < sub_cloud_ind[0].size(); i++)
	{
		PCLPoint point = (*cloud).points[sub_cloud_ind[0][i]];
		if(point.z == point.z)
		{
			centre(0) += point.x * sub_cloud_ind[1][i];
			centre(1) += point.y * sub_cloud_ind[1][i];
			centre(2) += point.z * sub_cloud_ind[1][i];
			sum += sub_cloud_ind[1][i];
			nb_points ++;
	  }
	}
	if (nb_points == 0)
	{
		std::cerr << "Error PCA::computeCentre no points for center computation -> division by zero " << std::endl;
		centre << 0, 0, 0;
	}
	centre /= (double)(nb_points * sum);
	
	if (centre.norm()!=centre.norm())
		std::cerr << "Error PCA::computeCentre NaN Centre" << std::endl; 
}

void // Computes the covariance Matrix
PCA::computeCovar(Eigen::Matrix3d & covar)
{
	// Subtract the center & Compute the covariance Matrix
	double covxx = 0, covyy = 0, covzz = 0, covxy = 0, covxz = 0, covyz = 0;
	for(size_t i = 0; i < sub_cloud_ind[0].size(); i++)
	{
		PCLPoint point = (*cloud).points[sub_cloud_ind[0][i]];
		if(point.z == point.z)
		{
			point.x -= centre(0);
			point.y -= centre(1);
			point.z -= centre(2);
			covxx += point.x * point.x * (sub_cloud_ind[1][i] / 100.0);
			covyy += point.y * point.y * (sub_cloud_ind[1][i] / 100.0);
			covzz += point.z * point.z * (sub_cloud_ind[1][i] / 100.0);
			covxy += point.x * point.y * (sub_cloud_ind[1][i] / 100.0);
			covxz += point.x * point.z * (sub_cloud_ind[1][i] / 100.0);
			covyz += point.y * point.z * (sub_cloud_ind[1][i] / 100.0);
  	}
	}
	
	covar << 	covxx, covxy, covxz,
						covxy, covyy, covyz,
						covxz, covyz, covzz;
}

void // Checks the sign of the vectors
PCA::signCheck()
{	
	float signNorm = 0, signVecBase = 0;
	PCLPoint pcl_point;
	Eigen::Vector3d vec_point;
	for (size_t i = 0; i < sub_cloud_ind[0].size(); i ++)
	{
		pcl_point = (*cloud).points[sub_cloud_ind[0][i]];
		if(pcl_point.z == pcl_point.z)
		{
			pcl_point.x -= centre(0);
			pcl_point.y -= centre(1);
			pcl_point.z -= centre(2);
			vec_point << pcl_point.x, pcl_point.y, pcl_point.z;
			signNorm += normal.dot(vec_point);
			signVecBase += vec_base1.dot(vec_point);
		}
	}
	if (signNorm!=0)
		normal = normal * signNorm / fabs(signNorm);
		
	if (signVecBase!=0)
		vec_base1 = vec_base1 * signVecBase / fabs(signVecBase);
}




