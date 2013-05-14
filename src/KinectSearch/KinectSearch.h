
#include "../defines.h"

#include "../Search/Search.h"

void* threadSearch (void* thread_arg);
void* threadVisu (void* thread_arg);

class KinectSearch 
{ 
	public: 
		KinectSearch (int * parameters);
		~KinectSearch ();

		void cloud_cb_ (const pcl::PointCloud<PCLPoint>::ConstPtr &cloud);

		void run ();
		
		std::vector<long> bestRes();

		friend void* threadSearch (void* thread_arg);
		friend void* threadVisu (void* thread_arg);

  private: 
		BoostVisuVisu viewer;
		Search * search;
		PCLCloud cloud_visu;
		PCLCloud cloud_comp;
		std::vector< std::vector<long> > res;
		int * parameters;
		int nb_frames;
    bool copy_cloud_flag, being_copied_cloud_comp, copy_cloud_visu, being_copied_cloud_visu;
    bool pause_flag;
    long ind_picked;
    
  	void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event);
		void mouseEventOccurred (const pcl::visualization::MouseEvent &event);
}; 

