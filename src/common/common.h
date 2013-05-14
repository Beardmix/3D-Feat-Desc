/*! \file common.h
    \brief All the common functions
*/

#ifndef COMMON_H_INCLUDED
#define COMMON_H_INCLUDED

#include "../defines.h"

/*! \brief Fills the cloud sent in parameters, following the resquested number
    \param [in] number The number of the cloud to load
*/
PCLCloud loadCloud(char * number);

void gaussFilter(PCLCloud in, PCLCloud cloud_filtered);

int filterCoeff(long i);

/*! \brief Returns a float corresponding to the rgb code sent in parameters.
    \return A float number corresponding to the rgb code sent in parameters
    \param [in] r The red value (0 to 255)
    \param [in] g The green value (0 to 255)
    \param [in] b The green value (0 to 255)
*/
float color(int r, int g, int b);

/*! \brief Returns the squared euclidien distance between two points
    \return A float corresponding to the squared euclidien distance between the two given points
    \param [in] p1 The first point
    \param [in] p2 The second point
*/
float euclDist2(PCLPoint & p1, PCLPoint & p2);

bool rgb_to_hsl2 (Eigen::Vector3i rgb);

void pickColor(PCLCloud &in, float hue);

Eigen::Vector3d rgb_to_hsl (Eigen::Vector3i rgb);

double hsl_value (double n1, double n2, double hue);

Eigen::Vector3i hsl_to_rgb (Eigen::Vector3d hsl);

void removeWBG(PCLCloud cloud);

bool isColor(Eigen::Vector3i colorRGB);
bool isColor(Eigen::Vector3d colorHSL);
bool isWhite(Eigen::Vector3d colorHSL);
bool isBlack(Eigen::Vector3d colorHSL);
bool isGrey(Eigen::Vector3d colorHSL);

float rnd(int choice);

double my_dot(Eigen::Vector3d &vec1, Eigen::Vector3d &vec2);
double my_norm(Eigen::Vector3d &vec1);

#endif // COMMON_H_INCLUDED
