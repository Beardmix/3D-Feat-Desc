#include "common2_0.h"

PCLCloud // Fills the cloud sent in parameters, following the resquested number
loadCloud(char * number)
{
	PCLCloud cloud (new pcl::PointCloud<PCLPoint>);
	
  char buffer [50];
  sprintf (buffer, "../../clouds/%s.pcd", number);
	if (pcl::io::loadPCDFile(buffer, *cloud) == -1) //* load the file
	{
	  PCL_ERROR ("Couldn't read file\n");
	}
	(*cloud).points.resize ((*cloud).width * (*cloud).height);
	
	// Gaussian Filter
	PCLCloud cloud_filtered (new pcl::PointCloud<PCLPoint>(*cloud));
	//gaussFilter( cloud, cloud_filtered);
	
	// Remove White Black Grey
	//removeWBG( cloud_filtered );
		
	// Pick one color
	//pickColor( cloud_filtered, 0.6 );
	
	return cloud_filtered;
}

float // Returns a float corresponding to the rgb code sent in parameters
color(int r, int g, int b)
{ 
	uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
	return *reinterpret_cast<float*>(&rgb);
}

float // Returns the squared euclidien distance between two points
euclDist2(PCLPoint & p1,PCLPoint & p2)
{
	return (pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2));
}

bool
rgb_to_hsl2 (Eigen::Vector3i rgb)
{
	Eigen::Vector3d hsl = rgb_to_hsl (rgb);
	
	return ( hsl(0) > 0.5 && hsl(0) < 0.8 );
	
	//return ( (h < 0.2 || h > 0.9 || s<0.02));
	//return ( (h < 0.1 || h > 0.9));
	//return ( (h < 0.1 || h > 0.9) && s > 0.003 );
	//return (h > 0.001 && h < 0.1 && s > 0.008 );
	//return (h > 0.007843137 && h < 0.08627451 && s > 0.005 );

}

void
pickColor(PCLCloud &in, float hue)
{
	Eigen::Vector3d colorHSL;
	for (size_t i = 0; i < in->size(); i ++)
	{
		colorHSL = rgb_to_hsl( in->points[i].getRGBVector3i() );
		if (!(colorHSL(0) > hue-0.1 && colorHSL(0) < hue+0.1))
			in->points[i].rgb = 0;
			//in->points[i].rgb = color(255,0,255);
	}
}

Eigen::Vector3d
rgb_to_hsl (Eigen::Vector3i rgb)
{
  double red, green, blue;
  double h, s, l;
  double min, max;
  double delta;
  Eigen::Vector3d hsl;

  red   = rgb(0) / 255.0;
  green = rgb(1) / 255.0;
  blue  = rgb(2) / 255.0;

  h = rand()/(RAND_MAX + 0.0);

  if (red > green)
  {
    max = std::max (red,   blue);
    min = std::min (green, blue);
  }
  else
  {
    max = std::max (green, blue);
    min = std::min (red,   blue);
  }

  l = (max + min) / 2.0;

  if (max == min)
  {
   s = 0.0;
   h = rand()/(RAND_MAX + 0.0);
  }
  else
  {
    if (l <= 0.5)
      s = (max - min) / (max + min);
    else
      s = (max - min) / (2.0 - max - min);

    delta = max - min;

    if (delta == 0.0)
      delta = 1.0;

    if (red == max)
    {
      h = (green - blue) / delta;
    }
    else if (green == max)
    {
      h = 2.0 + (blue - red) / delta;
    }
    else if (blue == max)
    {
      h = 4.0 + (red - green) / delta;
    }

    h /= 6.0;

    if (h < 0.0)
      h += 1.0;
  }

  hsl(0) = h;
  hsl(1) = s;
  hsl(2) = l;
  
  return hsl;
}

Eigen::Vector3i
hsl_to_rgb (Eigen::Vector3d hsl)
{
	double hue = hsl(0);
	double saturation = hsl(1);
	double luma = hsl(2);
	Eigen::Vector3i rgb;

  if (saturation == 0.0)
  {
    hue        = luma;
    saturation = luma;
    luma       = luma;
  }
  else
  {
    double m1, m2;

    if (luma <= 0.5)
      m2 = luma * (1.0 + saturation);
    else
      m2 = luma + saturation - luma * saturation;

    m1 = 2.0 * luma - m2;

    rgb(0) = round (hsl_value (m1, m2, hue * 6.0 + 2.0) * 255.0);
    rgb(1) = round (hsl_value (m1, m2, hue * 6.0)       * 255.0);
    rgb(2) = round (hsl_value (m1, m2, hue * 6.0 - 2.0) * 255.0);
  }
  return rgb;
}

double
hsl_value (double n1, double n2, double hue)
{
  double val;

  if (hue > 6.0)
    hue -= 6.0;
  else if (hue < 0.0)
    hue += 6.0;

  if (hue < 1.0)
    val = n1 + (n2 - n1) * hue;
  else if (hue < 3.0)
    val = n2;
  else if (hue < 4.0)
    val = n1 + (n2 - n1) * (4.0 - hue);
  else
    val = n1;

  return val;
}

void removeWBG(PCLCloud cloud)
{
	Eigen::Vector3i colorRGB;
	for (size_t i = 0; i < cloud->size(); i ++)
	{
		colorRGB = cloud->points[i].getRGBVector3i();
		if (!isColor(colorRGB))
		{
			cloud->points[i].rgb = 0;
			//cloud->points[i].rgb = color(0,255,0);
		}
	}
}

bool isColor(Eigen::Vector3i colorRGB)
{
	Eigen::Vector3d colorHSL = rgb_to_hsl( colorRGB );
	return isColor(colorHSL);
}
bool isColor(Eigen::Vector3d colorHSL)
{
	if ( isWhite(colorHSL) ) // White
		return false;
	if ( isBlack(colorHSL) ) // Black
		return false;
	if ( isGrey(colorHSL) ) // Grey
		return false;
	return true;
}
bool isWhite(Eigen::Vector3d colorHSL)
{	
	if ( colorHSL(2) > 0.5 + 0.25 * ( colorHSL(1) + 1 ) ) 
		return true;
	return false;
}
bool isBlack(Eigen::Vector3d colorHSL)
{	
	if ( colorHSL(2) < 0.05 )
		return true;
	if ( colorHSL(1) < 0.25 && colorHSL(2) < 0.25 * ( 0.25 - colorHSL(1) + 0.2) )
		return true;
	return false;
}
bool isGrey(Eigen::Vector3d colorHSL)
{	
	if( colorHSL(1) < 0.05 * ( fabs(colorHSL(2) - 0.5) * 0.2 + 1 ) )
		return true;
	return false;
}

void
gaussFilter(PCLCloud in, PCLCloud cloud_filtered){
	// Gaussian Filter
	for (long x = 2; x < (cloud_filtered->width-2); x++){
		for (long y = 2; y < (cloud_filtered->height-2); y++){
			bool isOk = true;
			long ind = x + y * in->width;
			cloud_filtered->points[ind].z = 0;
			for (long i = -1; i < 2; i++){
				for (long j = -1; j < 2; j++){
					float coeff = filterCoeff(i) * filterCoeff(j);
					long ind2 = ind + i + j * (long) in->width;
					if(	in->points[ind2].z != in->points[ind2].z ||
							fabs(in->points[ind].z - in->points[ind2].z) > 0.2 )
						isOk = false;
					cloud_filtered->points[ind].z += in->points[ind2].z * coeff;
				}
			}
			cloud_filtered->points[ind].z /= 16.0;
			if (isOk == false)
				cloud_filtered->points[ind].z = in->points[ind].z;
		}
	}
}

int 
filterCoeff(long i){
	return (2 - abs(i));
	//return 1 + 3 * (2 - abs(i));
}

float rnd(int choice)
{
	switch (choice)
	{
		case 0:
			return ( rand () / (RAND_MAX + 1.0f) );
		case 1:
			return ( 2 * (rand () / (RAND_MAX + 1.0f) - 0.5) );
		default:
			return ( rand () / (RAND_MAX + 1.0f) );
	}
}

double my_dot(Eigen::Vector3d &vec1, Eigen::Vector3d &vec2)
{
	return vec1(0) * vec2(0) + vec1(1) * vec2(1) + vec1(2) * vec2(2);
}
double my_norm(Eigen::Vector3d &vec1)
{
	return sqrt(vec1(0) * vec1(0) + vec1(1) * vec1(1) + vec1(2) * vec1(2));
}
