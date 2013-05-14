/*! \file defines.h
    \brief All the defines needed & the typedef definitions
*/

#ifndef DEFINES_H_INCLUDED
#define DEFINES_H_INCLUDED

#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <vector>
#include <boost/spirit/include/classic.hpp>
#include <Eigen/Eigen>

/*! 5D Boost vector of int */
typedef boost::shared_ptr<std::vector<std::vector<std::vector<std::vector<std::vector<int> > > > > > BoostVector5Int; 
/*! 5D vector of int */
typedef std::vector<std::vector<std::vector<std::vector<std::vector<int> > > > > Vector5Int; 

/*! 4D Boost vector of int */
typedef boost::shared_ptr<std::vector<std::vector<std::vector<std::vector<int> > > > > BoostVector4Int; 
/*! 4D vector of int */
typedef std::vector<std::vector<std::vector<std::vector<int> > > > Vector4Int; 

/*! 1D Boost vector of double */
typedef boost::shared_ptr<std::vector<double> > BoostVector1Dbl;
/*! 1D vector of double */
typedef std::vector<double> Vector1Dbl;

/*! 2D Boost vector of Integer */
typedef boost::shared_ptr<std::vector<std::vector<int> > > BoostVector2Int;
/*! 2D vector of Integer */
typedef std::vector<std::vector<int> > Vector2Int;

/*! 1D Boost vector of Integer */
typedef boost::shared_ptr<std::vector<int> > BoostVector1Int;
/*! 1D vector of Integer */
typedef std::vector<int> Vector1Int;


/*! 3D Boost vector of Eigen::Vector3f */
typedef boost::shared_ptr<std::vector<std::vector<std::vector<Eigen::Vector3f> > > > BoostVector3EigVect3;
/*! 3D vector of Eigen::Vector3f */
typedef std::vector<std::vector<std::vector<Eigen::Vector3f> > > Vector3EigVect3;

/*! PCL XYZRGB point */
typedef pcl::PointXYZRGBA PCLPoint;

/*!	PCL pointCloud of PCLPoint points */
typedef pcl::PointCloud<PCLPoint>::Ptr PCLCloud;

/*!	PCL pointCloud of PCLPoint points */
typedef const pcl::PointCloud<PCLPoint>::ConstPtr PCLConstCloud;

/*! Boost visualization Visualizer */
typedef boost::shared_ptr<pcl::visualization::PCLVisualizer> BoostVisuVisu;


#endif // DEFINES_H_INCLUDED
