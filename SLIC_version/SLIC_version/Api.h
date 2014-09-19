#if !defined (_API_H_INCLUDE)
#define API_H_INCLUDE

#include<iostream>
#include<opencv2/opencv.hpp>
#include<fstream>
#include"Holefilling.h"
#include"SLIC.h"
#include "PlaneParamEstimator.h"
#include "Ransac.h"

using namespace std;
using namespace cv;

class Api{

public:
	/*
	  parameter depth  the depth image we get from the Kinect sensors
	  parameter color  the color image we get from the Kinect sensors
	  parameter sigma  the sigma is used to get the final depth image
	  parameter m_spcount the desired number of segment you want to generate using the SLIC algorithm
	  parameter m_compacness the argument of the SLIC algorithm

	  Author :LLJ
	*/
	void Do_Depthimage_Repairing(cv::Mat& depth,cv::Mat& color,int sigma,int m_spcount,double m_compactness);

private:

};
#endif