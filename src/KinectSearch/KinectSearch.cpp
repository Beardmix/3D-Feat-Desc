
#include "KinectSearch.h"

#include <pcl/io/openni_grabber.h>

#include "../common/common.h"


KinectSearch::KinectSearch (int * parameters) : 
	cloud_visu (loadCloud((char *) "Mathieu2/cloud1")), 
	cloud_comp (loadCloud((char *) "Mathieu2/cloud1")), 
	parameters(parameters), 
	nb_frames(0),
	copy_cloud_flag(false),
	being_copied_cloud_comp(false),
	copy_cloud_visu(false),
	being_copied_cloud_visu(false),
	pause_flag(false),
	ind_picked (180215)
{ 			
	search = new Search (cloud_comp, parameters, ind_picked);
	cloud_visu->points[ind_picked].rgb = color( 0, 255, 0);
	
	viewer = BoostVisuVisu (new pcl::visualization::PCLVisualizer("Scene Viewer"));
	viewer->setCameraPosition(0,0,-4,0,-1,0,0);
	viewer->addPointCloud(cloud_visu, "cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
	viewer->addSphere(cloud_visu->points[0], parameters[3]/1000.0, 1, 0, 0, "best");
	viewer->registerKeyboardCallback (bind(&KinectSearch::keyboardEventOccurred, this, _1));
	viewer->registerMouseCallback (bind(&KinectSearch::mouseEventOccurred, this, _1));
	viewer->setSize(1024,700); 
  viewer->setPosition(1366,0); 
	
	
	pthread_t id;
	pthread_create(&id, NULL, &threadSearch, this);
	
	pthread_t id2;
	pthread_create(&id2, NULL, &threadVisu, this);
				
} 
KinectSearch::~KinectSearch () 
{ 
	delete search;
} 

void 
KinectSearch::cloud_cb_ (const pcl::PointCloud<PCLPoint>::ConstPtr &cloud) 
{ 
	if (!viewer->wasStopped() && !pause_flag)
	{
		being_copied_cloud_comp = true;
		while(!copy_cloud_flag)
 			boost::this_thread::sleep (boost::posix_time::microseconds (10));   			
		pcl::copyPointCloud<PCLPoint, PCLPoint>(*cloud, *cloud_comp); 			
		being_copied_cloud_comp = false;
	}
} 

void 
KinectSearch::run () 
{ 
	pcl::Grabber* interface = new pcl::OpenNIGrabber(); 

	boost::function<void (const pcl::PointCloud<PCLPoint>::ConstPtr&)> f =  boost::bind (&KinectSearch::cloud_cb_, this, _1); 

	interface->registerCallback (f); 

	interface->start (); 

	while (!viewer->wasStopped()) 
	{
		boost::this_thread::sleep (boost::posix_time::milliseconds (1000));
	} 
	interface->stop (); 
} 

std::vector<long> KinectSearch::bestRes()
{
	long max = 0;
	for (long i = 0; i < (long)res.size(); i ++)
	{
		if (res[i][1] < res[max][1])
		{
			max = i;
		}
	}
	return res[max];
}


void* threadSearch (void* thread_arg)
{
	class KinectSearch * kin;
	kin = (class KinectSearch *)thread_arg;
	std::cout<<"Search Thread Opened"<<std::endl;
	
	size_t index = 0;
	std::vector<long> init;
	std::vector<long> best;
	init.push_back(0);
	init.push_back(100);
	kin->res.resize(5, init);
	
	while(!kin->viewer->wasStopped()){
		if (!kin->pause_flag)
		{
	 		kin->search->clearGrid();
	 		kin->search->generateGrid(1,5);
	 		kin->search->generateGrid(2,10);
			if(!kin->being_copied_cloud_comp){
				kin->copy_cloud_flag = false;
				kin->res[index] = kin->search->runGrid();
				kin->copy_cloud_flag = true;
			}
			kin->res[index] = kin->search->runSA(kin->res[index][0]);
			/*
			// Tracking
			best = kin->bestRes();
			if (best[1] < 10)
				kin->search->setReference(best[0]);
			*/
			kin->nb_frames++;
			index = (index +1) % 5;
		 	boost::this_thread::sleep (boost::posix_time::microseconds (1000));
		}
		else{
			boost::this_thread::sleep (boost::posix_time::milliseconds (100));
		}
	}

	std::cout<<"Closing Search Thread"<<std::endl;
	int ret_val = 2;
	pthread_exit(&ret_val);
	return 0;
}

void* threadVisu (void* thread_arg)
{
	class KinectSearch * kin;
	kin = (class KinectSearch *)thread_arg;
	std::cout<<"Visu Thread Opened"<<std::endl;
	double time_comp;
	double time_visu;
	int nb_loops = 0;
	double diff = 0;
	std::vector<long> best;
	Eigen::Vector3d colorHSL;
	Eigen::Vector3i colorRGB;
	
	while(!kin->viewer->wasStopped()){
		boost::this_thread::sleep (boost::posix_time::microseconds (5000));
		if (!kin->pause_flag)
		{
			while(!kin->being_copied_cloud_comp)
	 			boost::this_thread::sleep (boost::posix_time::microseconds (10));   			
			pcl::copyPointCloud<PCLPoint, PCLPoint>(*kin->cloud_comp, *kin->cloud_visu); 
			/*
			for (int i = 0; i < kin->cloud_visu->size(); i ++)
			{
				colorRGB = kin->cloud_visu->points[i].getRGBVector3i();
				colorRGB(0) += 10;
				if (colorRGB(0) > 255)
					colorRGB(0) = 255;
				colorRGB(2) -= 10;
				if (colorRGB(2) < 0)
					colorRGB(2) = 0;
				colorHSL = rgb_to_hsl(colorRGB);
				if ( isGrey(colorHSL) )
					kin->cloud_visu->points[i].rgb = color(255,0,0);
				if ( isWhite(colorHSL) )
					kin->cloud_visu->points[i].rgb = color(0,255,0);
				if ( isBlack(colorHSL) )
					kin->cloud_visu->points[i].rgb = color(0,0,255);
			}
			*/
		}
		
		best = kin->bestRes();
		diff = best[1] / 100.0;
		if (nb_loops > 2)
		{			
 			kin->viewer->updatePointCloud(kin->cloud_visu, "cloud");
			if (nb_loops > 10)
			{
				std::cout << "display: " << round(nb_loops / (double)(pcl::getTime() - time_visu))<< " Hz "; 
				std::cout << " |  compute: " << round(kin->nb_frames /(float) (pcl::getTime() - time_comp)) << " Hz" << std::endl; 
				time_comp = pcl::getTime();
				nb_loops = 0;
				kin->nb_frames = 0;
				time_visu = pcl::getTime(); 
			}
		}
		kin->viewer->updateSphere(kin->cloud_visu->points[best[0]], kin->parameters[3]/1000.0, diff,  (1 - 2 * diff) * (1 - floor(diff / 0.501)), diff, "best");
		kin->viewer->spinOnce (0.1);
		nb_loops ++;
	}

	std::cout<<"Closing Visu Thread"<<std::endl;
	int ret_val = 2;
	pthread_exit(&ret_val);
	return 0;
}

void KinectSearch::keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event)
{	
	int coeff = 1;
	if ( event.isAltPressed() ) 
		coeff = 10;
	if (event.keyUp() && event.getKeySym () != "Alt_L"){		
		if (pause_flag)
		{
			cloud_visu->points[ind_picked].rgb = color(255,255,255);
			if (event.getKeySym () == "Up")
				ind_picked -= cloud_visu->width * coeff;
			if (event.getKeySym () == "Down")
				ind_picked += cloud_visu->width * coeff;
			if (event.getKeySym () == "Left")
				ind_picked -= 1 * coeff;
			if (event.getKeySym () == "Right")
				ind_picked += 1 * coeff;
			Eigen::Vector3i colorRGB = cloud_visu->points[ind_picked].getRGBVector3i();
			cloud_visu->points[ind_picked].rgb = color(255-colorRGB(0),255-colorRGB(1),255-colorRGB(2));
			if (event.getKeySym () == "Return")
			{
				search->setReference( ind_picked );
				pause_flag = false;
			}
		}
  }
}
void KinectSearch::mouseEventOccurred (const pcl::visualization::MouseEvent &event)
{
  if (event.getButton () == pcl::visualization::MouseEvent::RightButton && event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease)
		pause_flag = true;
  if (event.getButton () == pcl::visualization::MouseEvent::LeftButton && event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease && pause_flag)
		pause_flag = false;
}

