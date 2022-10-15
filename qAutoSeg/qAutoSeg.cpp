//##########################################################################
//#                                                                        #
//#                CLOUDCOMPARE PLUGIN: qAutoSeg                           #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#                         Cyberbuild,2020                                #
//#								 v1.2                                      #
//#                                                                        #
//##########################################################################

#include "qAutoSeg.h"

//Local
#include "profileImportDlg.h"

//PCL
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

//Qt
#include <QtGui>
#include <QSettings>
#include <QFileInfo>
#include <QFile>
#include <QMessageBox>
#include <QTextStream>
#include <QMainWindow>

//qCC_db
#include <ccFileUtils.h>
#include <ccHObjectCaster.h>
#include <ccPointCloud.h>
#include <ccProgressDialog.h>
#include <ccMesh.h>
#include <ccPolyline.h>
#include <ccCone.h>
#include <ccScalarField.h>
#include <ccGLWindow.h>

//qCCIO
#include <PlyFilter.h>

//OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

//System
#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <fstream>
#include <unordered_set>
#include <random>
#include <algorithm>
#include <list>
#include <numeric>
#include <random>
#include <vector>
#include <ctime>

//CCCoreLib
#include <CloudSamplingTools.h>
#include <DistanceComputationTools.h>
#include <DgmOctreeReferenceCloud.h>
#include <GenericProgressCallback.h>
#include <Neighbourhood.h>
#include <PointCloud.h>
#include <ReferenceCloud.h>
#include <ScalarField.h>
#include <ScalarFieldTools.h>
#include <SimpleMesh.h>

ccAutoSeg::ccAutoSeg( QObject *parent )
	: QObject( parent )
	, ccStdPluginInterface( ":/CC/plugin/qAutoSeg/info.json" )
	, m_action( nullptr )
{
}

using namespace std;
using namespace cv;

//Calculate std
double stddev(std::vector<double> const & func)
{
	double mean = std::accumulate(func.begin(), func.end(), 0.0) / func.size();
	double sq_sum = std::inner_product(func.begin(), func.end(), func.begin(), 0.0,
		[](double const & x, double const & y) { return x + y; },
		[mean](double const & x, double const & y) { return (x - mean)*(y - mean); });
	return std::sqrt(sq_sum / (func.size() - 1));
}

string dateStamp() {

	time_t rawtime;
	struct tm * timeinfo;
	char buffer[80];

	time(&rawtime);
	timeinfo = localtime(&rawtime);

	strftime(buffer, sizeof(buffer), "%H:%M:%S", timeinfo);
	std::string str(buffer);

	string stamp("[");
	stamp += str;
	stamp += "]";

	return stamp;

}

double optRotY(ccPointCloud *&cloud) {
	
	pcl::PointCloud<pcl::PointXYZRGB> ::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl_cloud->width = cloud->size();
	pcl_cloud->height = 1;
	pcl_cloud->points.resize(pcl_cloud->width * pcl_cloud->height);

	std::vector<int> v(cloud->size());
	std::iota(v.begin(), v.end(), 0);
	std::random_shuffle(v.begin(), v.end());
	v.erase(v.begin(), v.begin() + (int)floor(cloud->size()));

	for (size_t i = 0; i < cloud->size(); i++)
	{
		pcl_cloud->points[i].x = cloud->getPoint(i)->x;
		pcl_cloud->points[i].y = cloud->getPoint(i)->y;
		pcl_cloud->points[i].z = cloud->getPoint(i)->z;

	}

	pcl::PointXYZRGB bbMin0, bbMax0;
	pcl::getMinMax3D(*pcl_cloud, bbMin0, bbMax0);

	pcl::PointCloud<pcl::PointXYZRGB> ::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	transformed_cloud->width = cloud->size();
	transformed_cloud->height = 1;
	transformed_cloud->points.resize(transformed_cloud->width * transformed_cloud->height);

	double th0 = 0;
	double area0 = (bbMax0.x - bbMin0.x)*(bbMax0.z - bbMin0.z);
	for (int th = -45; th < 46; th++) { //Apply this to a sparser cloud to make the process lighter
		double thD = (double)th * M_PI / 180.0;
		
		Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

		// Define a translation
		transform_2.translation() << 0, 0.0, 0.0;

		// The same rotation matrix as before; theta radians around Y axis
		transform_2.rotate(Eigen::AngleAxisf(thD, Eigen::Vector3f::UnitY()));

		pcl::transformPointCloud(*pcl_cloud, *transformed_cloud, transform_2);

		pcl::PointXYZRGB bbMin, bbMax;
		pcl::getMinMax3D(*transformed_cloud, bbMin, bbMax);
		double area = (bbMax.x - bbMin.x)*(bbMax.z - bbMin.z);
		if (area < area0) {
			area0 = area;
			th0 = thD;
		}

		Eigen::Affine3f transform_inv = transform_2.inverse();

		pcl::transformPointCloud(*transformed_cloud, *pcl_cloud, transform_inv);
	}

	return th0;

}

void rotY(ccPointCloud *&cloud, double th0) {
	pcl::PointCloud<pcl::PointXYZRGB> ::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl_cloud->width = cloud->size();
	pcl_cloud->height = 1;
	pcl_cloud->points.resize(pcl_cloud->width * pcl_cloud->height);

	std::vector<int> v(cloud->size());
	std::iota(v.begin(), v.end(), 0);
	std::random_shuffle(v.begin(), v.end());
	v.erase(v.begin(), v.begin() + (int)floor(cloud->size()));

	for (size_t i = 0; i < cloud->size(); i++)
	{
		pcl_cloud->points[i].x = cloud->getPoint(i)->x;
		pcl_cloud->points[i].y = cloud->getPoint(i)->y;
		pcl_cloud->points[i].z = cloud->getPoint(i)->z;

		pcl_cloud->points[i].r = cloud->getPointColor(i).r;
		pcl_cloud->points[i].g = cloud->getPointColor(i).g;
		pcl_cloud->points[i].b = cloud->getPointColor(i).b;
	}


	//Bring cloud to (0,0,0)
	Eigen::Vector4d centroid;
	pcl::compute3DCentroid(*pcl_cloud, centroid);

	Eigen::Affine3f t0 = Eigen::Affine3f::Identity();

	t0.translation() << -centroid[0], -centroid[1], -centroid[2];

	pcl::PointCloud<pcl::PointXYZRGB> ::Ptr transformed0(new pcl::PointCloud<pcl::PointXYZRGB>);
	transformed0->width = pcl_cloud->size();
	transformed0->height = 1;
	transformed0->points.resize(transformed0->width * transformed0->height);

	pcl::transformPointCloud(*pcl_cloud, *transformed0, t0);

	//Rotate around Y axis
	Eigen::Affine3f transform0 = Eigen::Affine3f::Identity();
	transform0.translation() << 0.0, 0.0, 0.0;
	transform0.rotate(Eigen::AngleAxisf(th0, Eigen::Vector3f::UnitY()));
	pcl::transformPointCloud(*transformed0, *transformed0, transform0);

	//To original UCS
	Eigen::Affine3f t1 = Eigen::Affine3f::Identity();
	t1.translation() << centroid[0], centroid[1], centroid[2];
	pcl::transformPointCloud(*transformed0, *transformed0, t1);

	//From PCL to cc object
	ccPointCloud* cloudAligned = new ccPointCloud("Aligned");
	for (int i = 0; i < transformed0->size(); i++)
	{
		cloudAligned->reserveThePointsTable(1);
		cloudAligned->reserveTheRGBTable();
		CCVector3 p(transformed0->points[i].x, transformed0->points[i].y, transformed0->points[i].z);
		cloudAligned->addPoint(p);

		uchar rojo = transformed0->points[i].r;
		uchar verde = transformed0->points[i].g;
		uchar azul = transformed0->points[i].b;
		const ccColor::Rgb col{ rojo, verde, azul };
		cloudAligned->addColor(col);
	}

	cloud = cloudAligned;

}

void cloud2binary(ccPointCloud* cloud, Mat &corrMatS, vector<Point> &idxPxS, vector<int> &idxPx1DS, Mat &imageBWS) {

	//Correspondence 2D->1D. Each px has an index instead of two coords
	int cntC = 0;
	for (int y = 0; y < corrMatS.rows; y++) {
		for (int x = 0; x < corrMatS.cols; x++)
		{
			corrMatS.at<int>(y, x) = cntC;
			cntC++;
		}
	}

	//Binary
	int s = cloud->size();
	const CCVector3* pointC0;
	for (int i = 0; i < s; i++) {
		//depth
		pointC0 = cloud->getPoint(i);
		int z = floor(pointC0->z);
		int x = floor(pointC0->x);
		imageBWS.at<uchar>(z, x) = 255;

		//idxPx per 3D point

		idxPxS.push_back(Point(z, x));
		idxPx1DS.push_back(corrMatS.at<int>(z, x));
	}

}

//Extract Skeleton from Binary (CV_8U)
Mat skeleton(Mat I, bool flagInv) {
	Mat toSkel;
	if (flagInv) { //If black-white inversion is required
		toSkel = 255 * cv::Mat::ones(I.rows, I.cols, CV_8U) - I;
	}
	else {
		toSkel = I;
	}
		


	cv::Mat skel(toSkel.size(), CV_8U, cv::Scalar(0));
	cv::Mat temp;
	cv::Mat eroded;

	cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
	int iterations = 0;

	bool done;
	do
	{
		cv::erode(toSkel, eroded, element);
		cv::dilate(eroded, temp, element); 
		cv::subtract(toSkel, temp, temp);
		cv::bitwise_or(skel, temp, skel);
		eroded.copyTo(toSkel);

		done = (cv::countNonZero(toSkel) == 0);
		iterations++;
	} while (!done && (iterations < 100));

	return skel;
}


//Extract elements of a vector given the indices
vector<Point> extractVectorFromIdx(vector<Point> v, vector<int> idx) {
	vector<Point> subset;

	for (int i = 0; i<idx.size(); i++) {
		Point x(v.at(idx.at(i)).x, v.at(idx.at(i)).y);
		subset.push_back(x);
	}
	return subset;
}

//Populate a matrix with a given value
void populateMat(Mat &populated, vector<Point> pixList, int value) {
	for (int i = 0; i < pixList.size(); i++) {
		populated.at<uchar>((pixList.at(i).x), (pixList.at(i).y)) = value;
	}
}

//Calculate difference set in two pixel lists (Matlab setdiff equivalent). Returns pixels in listOld that are not in listNew
vector<Point> setdiffPixs(vector<Point> listNew, vector<Point> listOld) {
	vector<Point> setdiffList;
	int cnt;
	for (int i = 0; i < listOld.size(); i++) {
		cnt = 0;
		for (int j = 0; j < listNew.size(); j++) {
			if (listOld.at(i).x == listNew.at(j).x && listOld.at(i).y == listNew.at(j).y) {
				cnt++;
			}
		}
		if (cnt == 0) {
			Point x(listOld.at(i).x, listOld.at(i).y);
			setdiffList.push_back(x);
		}

	}
	return setdiffList;
}

//Calculate intersection set in two pixel lists. Returns idx pixels in listNew that are in listOld
vector<int> setIntersectIdxPixs(vector<Point> listNew, vector<Point> listOld) {
	vector<int> commonList;

	//Common
	for (int i = 0; i < listNew.size(); i++) {
		for (int j = 0; j < listOld.size(); j++) {
			if (listNew.at(i).x == listOld.at(j).x && listNew.at(i).y == listOld.at(j).y) {
				commonList.push_back(i);
			}
		}
	}

	return commonList;
}


vector<int> setIntersectIdxPixs1D(vector<int> listNew, vector<int> listOld) {
	vector<int> commonList;

	//Common
	int cnt = 0;
	for (auto &e : listNew) {
		for (auto &ee : listOld) {
			if ( e == ee) {
				commonList.push_back(cnt);
				break;
			}
		}
		cnt++;
	}

	return commonList;
}


//Calculate exclusive OR in two pixel lists (Matlab setxor equivalent)
vector<Point> setxorPixs(vector<Point> listNew, vector<Point> listOld) {
	vector<Point> setxorList;
	vector<int> commonList;

	//Common
	for (int i = 0; i < listNew.size(); i++) {
		for (int j = 0; j < listOld.size(); j++) {
			if (listNew.at(i).x == listOld.at(j).x && listNew.at(i).y == listOld.at(j).y) {
				Point x(listNew.at(i).x, listNew.at(i).y);
				/*setxorList.push_back(x);*/
				commonList.push_back(i);
			}
		}
	}

	//Setxor
	int nLabels = listNew.size();
	vector<int> v(nLabels);
	iota(std::begin(v), std::end(v), 0);
	vector<int> setxorIdx;
	std::set_difference(std::begin(v), std::end(v), std::begin(commonList), std::end(commonList), std::back_inserter(setxorIdx));

	for (int j = 0; j < setxorIdx.size(); j++) {
			Point x(listNew.at(setxorIdx.at(j)).x, listNew.at(setxorIdx.at(j)).y);
			setxorList.push_back(x);
		
	}

	return setxorList;
}

//Extract values of pixels per labelled segment in a binary image
vector<int> pixelValues(Mat inputM, vector<Point> &pixelList) {
	vector<int> pixelV;
	for (int i = 0; i < pixelList.size(); i++) {
		int val = inputM.at<uchar>(pixelList.at(i).x, pixelList.at(i).y);
		pixelV.push_back(val);
	}
	return pixelV;
}

//Extract pixels per labelled segment in a binary image
vector<vector<Point> > pixelListMat(Mat inputM, int nLabels) {
	vector<vector<Point> > pixelSegments(nLabels);
		for (int i = 0; i < inputM.rows; i++) {
			for (int j = 0; j < inputM.cols; j++) {
				int pixelValue = inputM.at<int>(i, j);
				if (pixelValue > 0) {
					Point x(i, j);
					pixelSegments[pixelValue-1].push_back(x); //-1 to correct value 0 for background
				}

			}
		}
	return pixelSegments;
}

//Find values over/under a value in an array. op = 1: lt; op = 2: leq; op = 3: gt; op = 4: geq; op = 5: eq
vector<int> findMatlab(vector<double> inputV, int op, double thres)
{
	vector<int> indices;

	if (op == 1){
		for (int i = 0; i < inputV.size(); i++) {
			if (inputV.at(i) < thres)
				indices.push_back(i);
		}
	}
	if (op == 2) {
		for (int i = 0; i < inputV.size(); i++) {
			if (inputV.at(i) <= thres)
				indices.push_back(i);
		}
	}
	if (op == 3) {
		for (int i = 0; i < inputV.size(); i++) {
			if (inputV.at(i) > thres)
				indices.push_back(i);
		}
	}
	if (op == 4) {
		for (int i = 0; i < inputV.size(); i++) {
			if (inputV.at(i) >= thres)
				indices.push_back(i);
		}
	}
	if (op == 5) {
		for (int i = 0; i < inputV.size(); i++) {
			if (inputV.at(i) == thres)
				indices.push_back(i);
		}
	}
	return indices;
}


vector<int> findIdx(vector<int> inputV, int thres)
{
	vector<int> indices;

		for (int i = 0; i < inputV.size(); i++) {
			if (inputV.at(i) == thres)
				indices.push_back(i);
		}
	
	return indices;
}

//Dilation
void dilationCC(int dilation_elem, int dilation_size, Mat &src, Mat &dilation_dst)
{
	int dilation_type = 0;
	if (dilation_elem == 0) { dilation_type = MORPH_RECT; }
	else if (dilation_elem == 1) { dilation_type = MORPH_CROSS; }
	else if (dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }
	Mat element = getStructuringElement(dilation_type,
		Size(2 * dilation_size + 1, 2 * dilation_size + 1),
		Point(dilation_size, dilation_size));
	dilate(src, dilation_dst, element);
}

// Erosion
void erosionCC(int erosion_elem, int erosion_size, Mat &src, Mat &erosion_dst){
	int erosion_type = 0;
	if (erosion_elem == 0) { erosion_type = MORPH_RECT; }
	else if (erosion_elem == 1) { erosion_type = MORPH_CROSS; }
	else if (erosion_elem == 2) { erosion_type = MORPH_ELLIPSE; }
	Mat element = getStructuringElement(erosion_type,
		Size(2 * erosion_size + 1, 2 * erosion_size + 1),
		Point(erosion_size, erosion_size));
	erode(src, erosion_dst, element);
}


//Stones division in smaller segments (when joint by narrow areas)
Mat stonesDivision(vector<vector<Point>> pixsLabel, Mat labels, vector<int> bigStones){
	//smallIndices
	int nLabels = pixsLabel.size();
	vector<int> v(nLabels); 
	iota(std::begin(v), std::end(v), 0); 

	std::vector<int> noBigStones;
	std::set_difference(std::begin(v), std::end(v),std::begin(bigStones), std::end(bigStones),std::back_inserter(noBigStones));
	
	//smallMatrix
	Mat smallStones = Mat::zeros(labels.size(), CV_8U);
	for (int i = 0; i < noBigStones.size(); i++) {
		for (int j = 0; j < pixsLabel[noBigStones.at(i)].size(); j++) {
			smallStones.at<uchar>(pixsLabel[noBigStones.at(i)].at(j).x, pixsLabel[noBigStones.at(i)].at(j).y) = 255;
		}
	}

	//bigMatrix
	Mat input = Mat::zeros(labels.size(), CV_8U);
	for (int i = 0; i < bigStones.size(); i++) {
		for (int j = 0; j < pixsLabel[bigStones.at(i)].size(); j++) {
					input.at<uchar>(pixsLabel[bigStones.at(i)].at(j).x, pixsLabel[bigStones.at(i)].at(j).y) = 255;
		}
	}

	//Erosion of big stones (in matlab was open: erosion + dilation, but here we dilated before) 
	int erosion_elem = 2;
	int erosion_size = 1;
	Mat output;
	erosionCC(erosion_elem, erosion_size, input, output);


	//Combine small + eroded big stones
	Mat combined = output+smallStones;

	return combined; //new segments

}

// Gets only the biggest segments
Mat threshSegments(Mat &src, double threshSize) {
	// FindContours:
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	Mat srcBuffer, output;
	src.copyTo(srcBuffer);
	findContours(srcBuffer, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_TC89_KCOS);

	vector<vector<Point> > allSegments;

	// For each segment:
	for (size_t i = 0; i < contours.size(); ++i) {
		cv::drawContours(srcBuffer, contours, i, Scalar(200, 0, 0), 1, 8, hierarchy, 0, Point());
		cv::Rect brect = cv::boundingRect(contours[i]);
		cv::rectangle(srcBuffer, brect, Scalar(255, 0, 0));

		int result;
		vector<Point> segment;
		for (unsigned int row = brect.y; row < brect.y + brect.height; ++row) {
			for (unsigned int col = brect.x; col < brect.x + brect.width; ++col) {
				result = pointPolygonTest(contours[i], Point(col, row), false);
				if (result == 1 || result == 0) {
					segment.push_back(Point(col, row));
				}
			}
		}
		allSegments.push_back(segment);
	}

	output = Mat::zeros(src.size(), CV_8U);
	int totalSize = output.rows*output.cols;
	for (int segmentCount = 0; segmentCount < allSegments.size(); ++segmentCount) {
		vector<Point> segment = allSegments[segmentCount];
		if (segment.size() > threshSize) {
			for (int idx = 0; idx < segment.size(); ++idx) {
				output.at<uchar>(segment[idx].y, segment[idx].x) = 255;
			}
		}
	}

	return output;
}

// Returns the type of a matrix as a string
string type2str(int type) {
	string r;

	uchar depth = type & CV_MAT_DEPTH_MASK;
	uchar chans = 1 + (type >> CV_CN_SHIFT);

	switch (depth) {
	case CV_8U:  r = "8U"; break;
	case CV_8S:  r = "8S"; break;
	case CV_16U: r = "16U"; break;
	case CV_16S: r = "16S"; break;
	case CV_32S: r = "32S"; break;
	case CV_32F: r = "32F"; break;
	case CV_64F: r = "64F"; break;
	default:     r = "User"; break;
	}

	r += "C";
	r += (chans + '0');

	return r;
}

// Get stats and additional info from a binary matrix
void extractInfo(Mat &input, vector<vector<Point> > &pixsLabel, Mat &labels, Mat &stats, vector<vector<Point> > &contours0) {
	//Get connected components and stats
	const int connectivity_8 = 8;
	Mat centroids;

	int nLabels = connectedComponentsWithStats(input, labels, stats, centroids, connectivity_8, CV_32S); //Note that component 0 is the background (i.e. nLabels = nsegments+1)


	//Extract the contours 
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(input, contours0, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

	string ty = type2str(labels.type());
	pixsLabel = pixelListMat(labels, nLabels - 1);
}


//Fill in the holes inside segments
void fillEdgeImage(cv::Mat edgesIn, cv::Mat& filledEdgesOut, int z) {

	cv::Mat edgesNeg = edgesIn.clone();
	string hola = type2str(edgesNeg.type());

	////////Check that seed is in mortar joints and not inside a stone
	Mat labels, stats;
	const int connectivity_8 = 8;
	Mat centroids;
	//invert 1s and 0s to label black areas from CWT instead of whites. In this way, we know that label 1 (the bigger segment) corresponds to mortar joints
	bitwise_not(edgesIn, edgesIn);

	int nLabels = connectedComponentsWithStats(edgesIn, labels, stats, centroids, connectivity_8, CV_32S); //Note that component 0 is the background (i.e. nLabels = nsegments+1)

	
	bitwise_not(edgesIn, edgesIn);


	int stSize = (int)stats.at<int>(1, 4); //0 is stone area, largest (not considering that one) is continuous mortar
	int	cntBiggestSize = 1;
	for (int i = 2; i < stats.rows; i++) {
		int loopSize = (int)stats.at<int>(i, 4);

		if (loopSize > stSize) {
			stSize = loopSize;

			cntBiggestSize = i;
		}
	}


	int f, c;
	for (int i = 0; i < edgesNeg.rows; i++) {
		for (int j = 0; j < edgesNeg.cols; j++) {
			int h = (int)edgesNeg.at<uchar>(i, j);
			int l = (int)labels.at<int>(i, j);
			if (h == 0 && l == cntBiggestSize) { // seed is a 0 in CWT and belongs to mortar joints
				f = i;
				c = j;
				goto stop;
			}
		}
	}

stop:

		

	cv::floodFill(edgesNeg, cv::Point(c, f), CV_RGB(255, 255, 255)); //The seed must be a black pixel
;
	bitwise_not(edgesNeg, edgesNeg);

	filledEdgesOut = (edgesNeg | edgesIn);

	return;
}



// Calculates the fft2
void fft2(const Mat in, Mat &complexI) {
	Mat padded;
	int m = getOptimalDFTSize(in.rows);
	int n = getOptimalDFTSize(in.cols);
	copyMakeBorder(in, padded, 0, m - in.rows, 0, n - in.cols, BORDER_CONSTANT, Scalar::all(0));

	Mat planes[] = { Mat_<float>(padded), Mat::zeros(padded.size(), CV_32F) };
	merge(planes, 2, complexI);
	dft(complexI, complexI);
}

//Calculates the conjugate of a complex matrix
Mat conjugate(Mat_<std::complex<double>> M1, bool *flag)
{
	using CP = std::complex<double>;
	cv::Mat_<CP> M3(M1.rows, M1.cols);
	//When there is no complex part, half of the values of the real part are considered as complex
	try {
		double a = M1.at<CP>(M1.rows - 1, M1.cols - 1).real();
		for (int i = 0; i < M1.rows; ++i) {
			for (int j = 0; j < M1.cols; ++j) {

				M3.at<CP>(i, j) = CP(M1.at<CP>(i, j).real(), -M1.at<CP>(i, j).imag());
			}
		}
		
		*flag = true;
		return M3;
	} //If there is no complex part
	catch (const std::exception) {
		M3 = M1;
		*flag = false;
		return M3;
	}

}

// Multiplies two complex matrices
Mat multiplicationCC(Mat_<std::complex<double>> M1, Mat_<std::complex<double>> M2)
{
	using CP = std::complex<double>;
	cv::Mat_<CP> M3(M1.rows, M1.cols);
	for (int i = 0; i < M3.rows; ++i) {
		for (int j = 0; j < M3.cols; ++j) {
			M3.at<CP>(i, j) = CP(M1.at<CP>(i, j).real()*M2.at<CP>(i, j).real() - M1.at<CP>(i, j).imag()*M2.at<CP>(i, j).imag(), M1.at<CP>(i, j).real()*M2.at<CP>(i, j).imag() + M1.at<CP>(i, j).imag()*M2.at<CP>(i, j).real());
		}
	}
	return M3;
}

//Calculates the Continuous Wavelet Transform
void cwt2d(double sca, Mat image, Mat &cwtcfs, Mat &bincfs)
{
	cv::Mat I = imread("depthMap.bmp", cv::IMREAD_GRAYSCALE);
	cv::Mat fImage = image; //(if there is 'image' matrix)


	// FFT
	std::cout << "Direct transform...\n";
	cv::Mat fourierTransform;
	fft2(fImage, fourierTransform);

	
	//Create frequency plane
	Size S = fourierTransform.size();
	int H = S.height;
	int W = S.width;

	int W2 = floor((W - 1) / 2);
	int H2 = floor((H - 1) / 2);

	std::vector<int> w1(W2 + 1); // vector with W2 ints.
	std::iota(std::begin(w1), std::end(w1), 0);
	std::vector<int> w2(W2 + 1); // vector with W2 ints.
	std::iota(std::begin(w2), std::end(w2), (W2 - W + 1));

	std::vector<double> v;
	v.reserve(w1.size() + w2.size()); // preallocate memory
	v.insert(v.end(), w1.begin(), w1.end());
	v.insert(v.end(), w2.begin(), w2.end());

	std::vector<double> W_puls(v.size());
	std::transform(v.begin(), v.end(), W_puls.begin(),
		[W](int i) { return i * 2 * M_PI / W; });

	std::vector<int> h1(H2 + 1); // vector with H2 ints.
	std::iota(std::begin(h1), std::end(h1), 0);
	std::vector<int> h2(H2 + 1); // vector with H2 ints.
	std::iota(std::begin(h2), std::end(h2), (H2 - H + 1));

	std::vector<double> v2;
	v2.reserve(h1.size() + h2.size()); // preallocate memory
	v2.insert(v2.end(), h1.begin(), h1.end());
	v2.insert(v2.end(), h2.begin(), h2.end());

	std::vector<double> H_puls(v2.size());
	std::transform(v2.begin(), v2.end(), H_puls.begin(),
		[H](int i) { return i * 2 * M_PI / H; });

	Mat matW = Mat(W_puls);
	Mat matH = Mat(H_puls);

	cv::Mat1d xx, yy;
	cv::repeat(matW.reshape(1, 1), matH.total(), 1, xx); //Matlab's Meshgrid equivalent
	cv::repeat(matH.reshape(1, 1).t(), 1, matW.total(), yy);

	double dxxyy = abs(xx.at<double>(0, 1) - xx.at<double>(0, 0)) * (yy.at<double>(1, 0) - yy.at<double>(0, 0));

	double wav_norm = 0;

	
	double ang = 0;

	Mat nxx = sca * (cos(ang)*xx - sin(ang)*yy);
	Mat nyy = sca * (sin(ang)*xx + cos(ang)*yy);


	int sigmax = 1;
	int sigmay = 1;
	int order = 2;

	Mat nxx2;
	pow(nxx, 2, nxx2);
	Mat nyy2;
	pow(nyy, 2, nyy2);

	Mat sum = nxx2 + nyy2;
	Mat sumPow;
	pow(sum, order / 2, sumPow);



	Mat prod1 = sigmax * nxx;
	Mat prod12;
	pow(prod1, 2, prod12);
	Mat prod2 = sigmay * nyy;
	Mat prod22;
	pow(prod2, 2, prod22);

	Mat ex = -(prod12 + prod22) / 2;
	Mat ex2;
	cv::exp(ex, ex2);


	Mat waveft2 = -(2 * M_PI) * sumPow.mul(ex2);


	//Complex mask
	using CP = std::complex<double>;
	cv::Mat_<CP> mask(Size(waveft2.rows, waveft2.cols));
	mask = sca * waveft2;

	bool flag = false;
	Mat mask_conj = mask;

	mask.convertTo(mask_conj, CV_32F);


	Mat g, mask2;
	vector<Mat> channels;
	if (flag) { //Mask is complex. TBC if there is the case

	}
	else {
		g = Mat::zeros(Size(mask_conj.cols, mask_conj.rows), CV_32FC1);
		mask2 = Mat::zeros(Size(mask_conj.cols, mask_conj.rows), CV_32FC2);

		channels.push_back(mask_conj);
		channels.push_back(g);

		merge(channels, mask2);
	}



	//Matrices product
	Mat M3;
	M3 = multiplicationCC(fourierTransform, mask2);

	// IFFT
	std::cout << "Inverse transform...\n";
	cv::dft(M3, cwtcfs, cv::DFT_INVERSE | cv::DFT_REAL_OUTPUT);


	// Back to 8-bits. angle values in Matlab code
	cwtcfs.convertTo(bincfs, CV_8U);

	
}



// Returns the contour of a stone as a polyline 

ccPolyline* contourPoly(ccPointCloud* stone)
{


	std::vector<CCCoreLib::PointProjectionTools::IndexedCCVector2> point;
	std::list<CCCoreLib::PointProjectionTools::IndexedCCVector2*> hullPoint2;


	for (unsigned i = 0; i < stone->size();++i)

	{
		PointCoordinateType x = stone->getPoint(i)->x;
		PointCoordinateType z = stone->getPoint(i)->z;
		CCCoreLib::PointProjectionTools::IndexedCCVector2 P(x, z, i);

		point.push_back(P);
	}


	CCCoreLib::PointProjectionTools::extractConcaveHull2D(point, hullPoint2, 0.0001);

	vector<unsigned> Vec;
	for (int i = 0; i < (hullPoint2.size());++i)
	{

		list<CCCoreLib::PointProjectionTools::IndexedCCVector2*>::iterator it = hullPoint2.begin();
		std::advance(it, i);
		CCCoreLib::PointProjectionTools::IndexedCCVector2* P2 = *it;
		int k = P2->index;
		Vec.push_back(k);
	}

	ccPointCloud* contour = new ccPointCloud(stone->getName() + " - Contour Cloud");

	for (auto e : Vec)
	{
		contour->reserveThePointsTable(1);
		contour->reserveTheRGBTable();
		CCVector3 p(stone->getPoint(e)->x, stone->getPoint(e)->y, stone->getPoint(e)->z); //kike added "y" value (it was 0)
		contour->addPoint(p);

		const ccColor::Rgb col = stone->getPointColor(e);
		contour->addColor(col);

	}

	int cnt = Vec.size();

	ccPolyline* pContour = new ccPolyline(contour);

	if (pContour->reserve(cnt))
	{
		pContour->addPointIndex(0, cnt);
		pContour->setClosed(true);
		pContour->setVisible(true);
		pContour->setName(stone->getName() +" - Contour");
		pContour->addChild(contour);
		pContour->setColor(ccColor::cyan);
		pContour->showColors(true);
		pContour->showVertices(true);

	}

	return pContour;
	
}



ccPolyline* contourPoly2(ccPointCloud* cloud0, vector<int> V, QString name )
{


	std::vector<CCCoreLib::PointProjectionTools::IndexedCCVector2> point;
	std::list<CCCoreLib::PointProjectionTools::IndexedCCVector2*> hullPoint2;


	for (unsigned i =0; i<V.size();++i)

	{
		PointCoordinateType x = cloud0->getPoint(V[i])->x;
		PointCoordinateType z = cloud0->getPoint(V[i])->z;
		CCCoreLib::PointProjectionTools::IndexedCCVector2 P(x, z, i);

		point.push_back(P);
	}


	CCCoreLib::PointProjectionTools::extractConcaveHull2D(point, hullPoint2, 0.0001);

	vector<unsigned> Vec;
	for (int i = 0; i < (hullPoint2.size());++i)
	{

		list<CCCoreLib::PointProjectionTools::IndexedCCVector2*>::iterator it = hullPoint2.begin();
		std::advance(it, i);
		CCCoreLib::PointProjectionTools::IndexedCCVector2* P2 = *it;
		int k = P2->index;
		Vec.push_back(k);
	}

	ccPointCloud* contour = new ccPointCloud(name + " - Contour Cloud");

	for (auto e : Vec)
	{
		contour->reserveThePointsTable(1);
		contour->reserveTheRGBTable();
		CCVector3 p(cloud0->getPoint(V[e])->x, cloud0->getPoint(V[e])->y, cloud0->getPoint(V[e])->z); 
		contour->addPoint(p);

		const ccColor::Rgb col = cloud0->getPointColor(V[e]);
		contour->addColor(col);

	}

	int cnt = Vec.size();

	ccPolyline* pContour = new ccPolyline(contour);

	if (pContour->reserve(cnt))
	{
		pContour->addPointIndex(0, cnt);
		pContour->setClosed(true);
		pContour->setVisible(true);
		pContour->setName(name + " - Contour");
		pContour->addChild(contour);
		pContour->setColor(ccColor::cyan);
		pContour->showColors(true);
		pContour->showVertices(true);

	}

	return pContour;

}

// Returns mortar maps (Depth and width)
ccPointCloud* getMortarMaps(ccPointCloud* f_cloudStones, ccPointCloud* f_cloudMortar)
{
	//Binary Mortar

	CCVector3 minBox;
	minBox = CCVector3(0, 0, 0);
	CCVector3 maxBox;
	maxBox = CCVector3(0, 0, 0);
	f_cloudMortar->scale(100, 1000, 100); //To cm mm cm
	f_cloudMortar->getBoundingBox(minBox, maxBox);
	int rowsM = ceil(maxBox.z); int colsM = ceil(maxBox.x);

	cv::Mat corrMatM = Mat::zeros(rowsM, colsM, CV_32F);
	vector<Point> idxPxM; vector<int> idxPxM1D;
	cv::Mat imageBWS = Mat::zeros(rowsM, colsM, CV_8U);

	cloud2binary(f_cloudMortar, corrMatM, idxPxM, idxPxM1D, imageBWS);

	//Skeleton
	Mat skel = skeleton(imageBWS, false);


	f_cloudMortar->scale(0.01, 0.001, 0.01); //Scale back


	//Skeleton 3D
	vector<int> pixelsSkel;
	for (int i = 0; i < skel.rows; i++) {
		for (int j = 0; j < skel.cols; j++) {
			unsigned char a = skel.at<char>(i, j);
			if (a == 255) {
				int idx = corrMatM.at<int>(i, j);
				pixelsSkel.push_back(idx);
			}
		}
	}

	vector<int> idxCloudMortarSkel = setIntersectIdxPixs1D(idxPxM1D, pixelsSkel);
	ccPointCloud* skelM = new ccPointCloud("Skeleton Mortar");
	for (auto e : idxCloudMortarSkel)
	{
		skelM->reserveThePointsTable(1);
		skelM->reserveTheRGBTable();
		CCVector3 p(f_cloudMortar->getPoint(e)->x, f_cloudMortar->getPoint(e)->y, f_cloudMortar->getPoint(e)->z);
		skelM->addPoint(p);
		const ccColor::Rgb col = f_cloudMortar->getPointColor(e);
		skelM->addColor(col);

	}


	CCCoreLib::CloudSamplingTools::SFModulationParams modParams(false); 	//Subsample result
	CCCoreLib::ReferenceCloud* refCloud = CCCoreLib::CloudSamplingTools::resampleCloudSpatially(skelM, 0.01, modParams, 0, 0); //1 point per cm^2

	ccPointCloud* f_skelMortar = skelM->partialClone(refCloud); 	//save output
	delete refCloud;
	delete skelM;
	refCloud = 0;
	skelM = 0;


	// Maps using PCL
	//Mortar depth map
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloudStones(new pcl::PointCloud<pcl::PointXYZ>);
	pcl_cloudStones->width = f_cloudStones->size();
	pcl_cloudStones->height = 1;
	pcl_cloudStones->points.resize(pcl_cloudStones->width * pcl_cloudStones->height);

	for (size_t i = 0; i < f_cloudStones->size(); ++i)
	{
		pcl_cloudStones->points[i].x = f_cloudStones->getPoint(i)->x;
		pcl_cloudStones->points[i].y = f_cloudStones->getPoint(i)->y;
		pcl_cloudStones->points[i].z = f_cloudStones->getPoint(i)->z;
	}

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

	kdtree.setInputCloud(pcl_cloudStones);


	vector<double> distStD, dist3St, distStD2;
	vector<double> distStWidth;
	vector<double> distStW;
	for (int i = 0; i < f_skelMortar->size(); i++) {

		pcl::PointXYZ searchPoint;

		searchPoint.x = f_skelMortar->getPoint(i)->x;
		searchPoint.y = f_skelMortar->getPoint(i)->y;
		searchPoint.z = f_skelMortar->getPoint(i)->z;


		std::vector<int> pointIdxRadiusSearch;
		std::vector<float> pointRadiusSquaredDistance;

		float radius = 0.1;	//10cm
		pcl::PointXYZ nnPoint;


		if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
		{

			nnPoint.x = pcl_cloudStones->points[pointIdxRadiusSearch[0]].x;
			nnPoint.y = pcl_cloudStones->points[pointIdxRadiusSearch[0]].y;
			nnPoint.z = pcl_cloudStones->points[pointIdxRadiusSearch[0]].z;

			double disty = abs(searchPoint.y - nnPoint.y); //Depth
			double distx = abs(searchPoint.x - nnPoint.x);
			double distz = abs(searchPoint.z - nnPoint.z);
			double dist3 = sqrt(distx*distx + distz * distz);//2D distance (XZ) will be used as a reference for the creation of mortar width map		

			distStD.push_back(disty * 1000); //depth (y axis) in mm
			dist3St.push_back(dist3);


			double dist1 = 1000; double dist2 = 1000; double dist1y = 1000; double dist2y = 1000;
			int idx1 = -1; int idx2 = -1;
			for (size_t j = 0; j < pointIdxRadiusSearch.size(); ++j) {
				double distx = abs(searchPoint.x - pcl_cloudStones->points[pointIdxRadiusSearch[j]].x); //2D distance
				double distz = abs(searchPoint.z - pcl_cloudStones->points[pointIdxRadiusSearch[j]].z);
				double disty = abs(searchPoint.y - pcl_cloudStones->points[pointIdxRadiusSearch[j]].y);
				double dist3 = sqrt(abs(distx)*abs(distx) + abs(distz)*abs(distz));

				int idxSt = f_cloudStones->getPointScalarValue(pointIdxRadiusSearch[j]);

				if (j == 0) { //Initialise NN stone 1
					dist1 = dist3;
					dist1y = disty;
					idx1 = idxSt;
				}
				else {
					if (idxSt == idx1) { //If point belongs to stone 1
						if (dist3 < dist1)
							dist1 = dist3;
						dist1y = disty;
					}
					if (idxSt != idx1 && idx2 == -1) { //Initialise stone 2
						idx2 = idxSt;
						dist2 = dist3;
						dist2y = disty;
					}
					if (idxSt == idx2) { //If point belongs to stone 2
						if (dist3 < dist2)
							dist2 = dist3;
						dist2y = disty;
					}
				}


			} //Points are ordered by radius. Idx 0 is the nearest neighbour

			//For each point, there is dist1 or dist1 and dist2
			if (idx1 > -1 && idx2 == -1 && dist1 < 1000) { //Only one stone (boundaries of wall)
				distStW.push_back(dist1 * 1000);
				distStD2.push_back(dist1y * 1000);
			}
			else if (idx1 > -1 && idx2 > -1) {
				distStW.push_back((dist1 + dist2) * 1000);
				distStD2.push_back((dist1y + dist2y) / 2 * 1000);
			}


		}
		else {
			distStD.push_back(100); //Add high values for non-calculated values (no neighbours available). These values can be modified according to the needs
			distStD2.push_back(100);
			dist3St.push_back(200);

			distStW.push_back(200);
		}



	}


	CCCoreLib::ScalarField* depthSF = nullptr; //Add depth as a scalarField to f_skelMortar

	int sfIdxD = f_skelMortar->getScalarFieldIndexByName("Mortar relative depth (mm)");
	if (sfIdxD < 0)
	{
		sfIdxD = f_skelMortar->addScalarField("Mortar relative depth (mm)");
	}
	if (sfIdxD < 0)
	{
		return nullptr;
	}

	depthSF = f_skelMortar->getScalarField(sfIdxD);

	for (unsigned int i = 0; i < distStD2.size(); i++)
	{
		double index = static_cast<double>(distStD2.at(i));
		depthSF->setValue(i, index);

	}

	f_skelMortar->setCurrentDisplayedScalarField(sfIdxD);

	depthSF->computeMinAndMax();



	CCCoreLib::ScalarField* widthSF = nullptr; //Add width as a scalarField to f_skelMortar

	int sfIdxW = f_skelMortar->getScalarFieldIndexByName("Mortar relative width (mm)");
	if (sfIdxW < 0)
	{
		sfIdxW = f_skelMortar->addScalarField("Mortar relative width (mm)");
	}
	if (sfIdxW < 0)
	{
		return nullptr;
	}

	widthSF = f_skelMortar->getScalarField(sfIdxW);

	for (unsigned int i = 0; i < distStW.size(); i++)
	{
		double index = static_cast<double>(distStW.at(i));
		widthSF->setValue(i, index);

	}

	f_skelMortar->setCurrentDisplayedScalarField(sfIdxW);

	widthSF->computeMinAndMax();
	//f_skelMortar->setPointSize(10);
	f_skelMortar->showSF(true);
	f_skelMortar->setName("Mortar Maps");
	f_skelMortar->setPointSize(7);
	return f_skelMortar;
}

float getDensity(ccPointCloud* cloud, vector<int> idx)
{
	ccPointCloud* stone = new ccPointCloud("stone");
	for (auto e : idx)
	{
		stone->reserveThePointsTable(1);
		CCVector3 p(cloud->getPoint(e)->x, cloud->getPoint(e)->y, cloud->getPoint(e)->z);
		stone->addPoint(p);
	}

	

	CCVector3 minBox;
	minBox = CCVector3(0, 0, 0);
	CCVector3 maxBox;
	maxBox = CCVector3(0, 0, 0);



	stone->getBoundingBox(minBox, maxBox);
	float area = (maxBox.x - minBox.x)*(maxBox.z - minBox.z);
	float density = idx.size() / area;
	return density;


}
// This method should enable or disable your plugin actions
// depending on the currently selected entities ('selectedEntities').
void ccAutoSeg::onNewSelection( const ccHObject::Container &selectedEntities )
{
	if (m_action)
	{
		//always active
	}

}

// This method returns all the 'actions' your plugin can perform.
// getActions() will be called only once, when plugin is loaded.
QList<QAction *> ccAutoSeg::getActions()
{

	if ( !m_action )
	{

		m_action = new QAction( getName(), this );
		m_action->setToolTip( getDescription() );
		m_action->setIcon(QIcon(QString::fromUtf8(":/CC/plugin/qAutoSeg/cyberbuildIcon.png")));
		
		// Connect appropriate signal
		connect( m_action, &QAction::triggered, this, &ccAutoSeg::doAction );
	}

	return QList<QAction *>{
		m_action,
	};
}


void ccAutoSeg::doAction()
{

	if (m_app == nullptr)
	{
		// m_app should have already been initialized by CC when plugin is loaded
		Q_ASSERT(false);

		return;
	}

	//Create log file
	
	time_t rawtime;
	struct tm * timeinfo;
	char buffer[80];

	time(&rawtime);
	timeinfo = localtime(&rawtime);

	strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", timeinfo);
	std::string str(buffer);

	string filename("APS_log_");
	filename += str;
	filename += ".txt";

	ofstream auto_seg_log;
	auto_seg_log.open(filename);
	

	time_t my_time = time(NULL);


	//Create pop-up window
	ProfileImportDlg piDlg(m_app->getMainWindow());




	if (!piDlg.exec())
		return;

	double joints = piDlg.jointsSpinBox->value(); //Mortar joint for CWT
	double segH = piDlg.segmentHSpinBox->value(); //Segmentation window
	double segV = piDlg.segmentVSpinBox->value();
	
	bool checkAlignment = piDlg.alignmentCheckBox->isChecked();
	bool checkSegment = piDlg.segmentCheckBox->isChecked();
	bool checkMortar = piDlg.mortarCheckBox->isChecked();

	//If mortar is selected but not segmentation, show an error message and ask for checking both
	if (checkMortar == true && checkSegment == false) {
		m_app->dispToConsole("Segmentation is required for the creation of mortar maps. Please, check both \"Automatic segmentation\" and \"Mortar maps\".", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}
	


	//Select point cloud (manually, from CC UI)
	const ccHObject::Container& selectedEntities = m_app->getSelectedEntities();

	if (!m_app->haveOneSelection() || !selectedEntities.front()->isA(CC_TYPES::POINT_CLOUD))
	{
		m_app->dispToConsole("Select one cloud!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}



	string stamp;
	stamp = dateStamp();
	auto_seg_log << stamp << " Importing point cloud" << endl;




	ccPointCloud* cloud0 = static_cast<ccPointCloud*>(selectedEntities.front());

	QString qName = cloud0->getName();


	string cloudName = qName.toLocal8Bit().constData();
	
	QString qFileName = cloud0->getParent()->getName();
	string name2 = qFileName.toLocal8Bit().constData();
	
	stamp = dateStamp();
	auto_seg_log << stamp << " File \""<< name2 << "\" loaded" << endl;
	auto_seg_log << stamp << " Cloud \"" << cloudName << "\" imported" << endl;


	//Add input parameters and tasks to be done to the log
	stamp = dateStamp();
	auto_seg_log << stamp << " Estimated mortar joints width: " << joints << "cm" <<endl;
	auto_seg_log << stamp << " Segmentation window. X: " << segH << "m and Y: " <<segV <<"m"<< endl;

	auto_seg_log << stamp << " Alignment: " << checkAlignment << endl;
	auto_seg_log << stamp << " Segmentation: " << checkSegment << endl;
	auto_seg_log << stamp << " Mortar maps: " << checkMortar << endl;
	auto_seg_log << " Starting process..." << endl;




	//Check plane orientation
	cv::Mat_<double> cldm(cloud0->size(), 3);
	for (unsigned int i = 0; i < cloud0->size(); i++) {
		cldm.row(i)(0) = cloud0->getPoint(i)->x;
		cldm.row(i)(1) = cloud0->getPoint(i)->y;
		cldm.row(i)(2) = cloud0->getPoint(i)->z;
	}

	cv::Mat_<double> mean;
	cv::PCA pca(cldm, mean, 150);

	cv::Vec3d nrm = pca.eigenvectors.row(2); nrm = nrm / norm(nrm);
	cv::Vec3d x0 = pca.mean;


	//Built-in function
	CCVector3 nrm2(nrm(0), nrm(1), nrm(2));
	CCVector3 a2(0, 1, 0);
	ccGLMatrix R2 = ccGLMatrix::FromToRotation(nrm2, a2);

	cloud0->applyRigidTransformation(R2); //Another rotation to aligned boundaries of the wall to axes X and Z is needed (rotate in Y axis and take angle linked to smallest bounding box)


	//Subsample cloud0 to obtain optimal rotY if size is over a threshold
	
	ccPointCloud* cloud2Rot = new ccPointCloud("cloud2Rot"); //Kike

	std::vector<int> v2(cloud0->size());
	std::iota(v2.begin(), v2.end(), 0);
	std::random_shuffle(v2.begin(), v2.end());

	std::vector<int> v;
	int nbSub;
	if (v2.size() > 1000000)
		nbSub = 1000000;
	else
		nbSub = v2.size();
		
	for (int i = 0; i < nbSub; i++) {
			v.push_back(v2.at(i));
	}


	for (int i = 0; i< v.size();i++)
	{
		cloud2Rot->reserveThePointsTable(1);
		cloud2Rot->reserveTheRGBTable();
		CCVector3 p(cloud0->getPoint(v.at(i))->x, cloud0->getPoint(v.at(i))->y, cloud0->getPoint(v.at(i))->z);
		cloud2Rot->addPoint(p);

		if (cloud0->hasColors())
		{
			const ccColor::Rgb col = cloud0->getPointColor(v.at(i));
			cloud2Rot->addColor(col);
		}
	}


	double th0 = optRotY(cloud2Rot);

	CCVector3 c1, c2, c3, c4;
	c1[0] = cos(th0);		c1[1] = 0; c1[2] = -sin(th0);	c1[3] = 0;
	c2[0] = 0;				c2[1] = 1; c2[2] = 0;			c2[3] = 0;
	c3[0] = sin(th0);		c3[1] = 0; c3[2] = cos(th0);	c3[3] = 0;
	c4[0] = 0;				c4[1] = 0; c4[2] = 0;			c4[3] = 1;
	ccGLMatrix R3(c1, c2, c3, c4);

	cloud0->applyRigidTransformation(R3);


	//If only alignment is required
	if (checkAlignment == true && checkSegment == false && checkMortar == false){
		ccGLWindow* win = m_app->getActiveGLWindow();
		win->setView(CC_FRONT_VIEW);
		return;
	}
	
	stamp = dateStamp();
	auto_seg_log << stamp << "Cloud aligned to XZ." << endl;

	//Bounding box
	CCVector3 minBox;
	minBox = CCVector3(0, 0, 0);
	CCVector3 maxBox;
	maxBox = CCVector3(0, 0, 0);

	cloud0->getBoundingBox(minBox, maxBox);

	//Move point cloud to origin
	cloud0->translate(-minBox);

	double axX = maxBox.x - minBox.x;
	double axY = maxBox.z - minBox.z; 

	// targeted window dimensions
	float winX = segH;
	float winY = segV;

	// Number of windows
	unsigned int nwinX = ceil(axX / winX);
	unsigned int nwinY = ceil(axY / winY);

	//real window dimensions without overlaping
	double rwinX = axX / nwinX;
	double rwinY = axY / nwinY;



	//boundaries of windows [(xmin1,xmax1,ymin1,ymax1),(xmin2,xmax2....)...]
	vector<vector<double>> boundaries;

	double X = 0;
	vector<int> global_idx;
	


	for (unsigned int i = 0; i < nwinX; i++) {
		double Y = 0; 
		for (unsigned int j = 0; j < nwinY; j++) {

			boundaries.push_back({ X, X + rwinX + 0.3, Y, Y + rwinY + 0.3 });

			Y += rwinY;
		}

		X += rwinX;
	}

	//spliting points by index
	vector<vector<int>> idxClouds(boundaries.size());
	vector<vector<int>> idxCloudsC;

	stamp = dateStamp();
	auto_seg_log << stamp << " Splitting cloud in patches (" << nwinX*nwinY << ")"<<endl;

	for (unsigned int i = 0; i < cloud0->size(); i++) {

		for (unsigned int j = 0; j < boundaries.size(); j++) {

			CCVector3 p(cloud0->getPoint(i)->x, cloud0->getPoint(i)->y, cloud0->getPoint(i)->z);
			if (p.x >= boundaries[j][0] && p.x <= boundaries[j][1] && p.z >= boundaries[j][2] && p.z <= boundaries[j][3]) //Kike. Not y, but z
			{
				idxClouds[j].push_back(i);
			}
			
		}
		global_idx.push_back(i);
	}

	// removing empty ones
	for (auto e : idxClouds)
		if (e.size() != 0 )
			idxCloudsC.push_back(e);

	idxClouds = idxCloudsC;


	stamp = dateStamp();
	auto_seg_log << stamp << " Splitting done" << endl;



	vector<ccPointCloud*> sub_clouds;


	stamp = dateStamp();
	auto_seg_log << stamp << " Creating subclouds" << endl;

	// adding points to the corresponding window

	for (unsigned int i = 0; i < idxClouds.size(); i++) 
	{
		string name = "part_" + to_string(i);
		QString str = QString::fromUtf8(name.c_str());
		ccPointCloud* win_cloud = new ccPointCloud(str); 
		if (idxClouds[i].size() > 0)
		{

			for (auto e : idxClouds[i])
			{
				win_cloud->reserveThePointsTable(1);
				win_cloud->reserveTheRGBTable();
				CCVector3 p(cloud0->getPoint(e)->x, cloud0->getPoint(e)->y, cloud0->getPoint(e)->z);
				win_cloud->addPoint(p);

				if (cloud0->hasColors())
				{
					const ccColor::Rgb col = cloud0->getPointColor(e);
					win_cloud->addColor(col);
				}
			}

			sub_clouds.push_back(win_cloud);
		}
		


	}


	

	// Going through the sub_clouds vector

	vector<vector<pair<int,int>>> stones;
	int stoneIdx = 0;

	

	for (unsigned int z = 0; z < sub_clouds.size(); ++z) {
		ccPointCloud* cloud = sub_clouds[z];

		stamp = dateStamp();
		auto_seg_log << stamp << " Cloud " << z << ": " << cloud->size() << " pts" << endl;



		if (cloud->size() > 0)
		{
			string cComment = "part_" + to_string(z) + " has " + to_string(cloud->size()) + " points";
			QString qComment = QString::fromUtf8(cComment.c_str());
			m_app->dispToConsole(qComment, ccMainAppInterface::STD_CONSOLE_MESSAGE);




			//Bounding box
			CCVector3 minBox_0;
			minBox_0 = CCVector3(0, 0, 0);
			CCVector3 maxBox_0;
			maxBox_0 = CCVector3(0, 0, 0);

			cloud->getBoundingBox(minBox_0, maxBox_0);

			//Move point cloud to origin
			cloud->translate(-minBox_0);


			//Create and populate maps

			//Bounding box
			CCVector3 minBox2;
			minBox2 = CCVector3(0, 0, 0);
			CCVector3 maxBox2;
			maxBox2 = CCVector3(0, 0, 0);
			const CCVector3* pointC;
			int b = cloud->size();

			cloud->scale(100, 1000, 100); //To cm mm cm

			cloud->getBoundingBox(minBox2, maxBox2);

			int rows = ceil(maxBox2.z);
			int cols = ceil(maxBox2.x);


			//Correspondence 2D->1D. Each px has an index instead of two coords
			cv::Mat corrMat = cv::Mat::zeros(rows, cols, CV_32F);
			int cntC = 0;
			for (int y = 0; y < corrMat.rows; y++) {
				for (int x = 0; x < corrMat.cols; x++)
				{
					corrMat.at<int>(y, x) = cntC;
					cntC++;
				}
			}

			vector<Point> idxPx;
			vector<int> idxPx1D;

			//Depth+colour
			cv::Mat image = cv::Mat::zeros(rows, cols, CV_32F);
			Mat imageR(rows, cols, CV_8U);
			Mat imageG(rows, cols, CV_8U);
			Mat imageB(rows, cols, CV_8U);
			for (int i = 0; i < b; i++) {
				//depth
				pointC = cloud->getPoint(i);
				int z = floor(pointC->z);
				int x = floor(pointC->x);
				image.at<float>(z, x) = pointC->y;
				//colour
				const ccColor::Rgb colorC = cloud->getPointColor(i);
				imageR.at<uchar>(z, x) = colorC.r;
				imageG.at<uchar>(z, x) = colorC.g;
				imageB.at<uchar>(z, x) = colorC.b;

				//idxPx per 3D point
				idxPx.push_back(Point(z, x));

				idxPx1D.push_back(corrMat.at<int>(z, x));
			}


			Mat imageC;
			vector<Mat> channels;

			channels.push_back(imageR);
			channels.push_back(imageG);
			channels.push_back(imageB);

			merge(channels, imageC);

			//Median filter to fill holes
			cv::Mat imageFilt = cv::Mat::zeros(rows, cols, CV_32F);
			cv::medianBlur(image, imageFilt, 3);


			for (int y = 0; y < image.rows; y++) {
				for (int x = 0; x < image.cols; x++)
				{
					if (image.at<float>(y, x) == 0) {
						image.at<float>(y, x) = imageFilt.at<float>(y, x);
					}
				}
			}

			//Check for black areas after filling holes. These are structural holes (windows, doors)
			vector<int> nullCols, nullRows;

			for (int y = 0; y < image.rows; y++) {
				for (int x = 0; x < image.cols; x++)
				{
					if (image.at<float>(y, x) == 0) {
						nullCols.push_back(x);
						nullRows.push_back(y);
					}
				}
			}


			cv::Mat imagePlot;
			image.convertTo(imagePlot, CV_8U);



			cloud->scale(0.01, 0.001, 0.01); //Scale back
			cloud->translate(minBox_0); //Translate back

			stamp = dateStamp();
			auto_seg_log << stamp << " Depth and colour maps created" << endl;

			//CWT
			double scaleCWT = joints*0.254;//1.27 for 5cm
			Mat cwtcfs;
			Mat bincfs = cv::Mat::zeros(cwtcfs.size(), CV_8UC1);
		
			cwt2d(scaleCWT, image, cwtcfs, bincfs);


			//Cropping borders added in cwt

			cv::Rect myROI(0, 0, image.cols, image.rows);
			bincfs = bincfs(myROI);

			Mat hola;

			stamp = dateStamp();
			auto_seg_log << stamp << " CWT calculated" << endl;

			//Clean areas from holes
			for (int i = 0; i < nullRows.size(); i++) {
				bincfs.at<uchar>(nullRows.at(i), nullCols.at(i)) = 0;
			}

	
			//Fill in the holes
			fillEdgeImage(bincfs, hola, z);

			//Remove small segments
			Mat output = threshSegments(hola, 20);



			//Erosion + Dilation
			int erosion_elem = 2; //0 = morph_rect, 1 = morph_cross, 2 = morph_ellipse (I think Matlab uses 0)
			int erosion_size = 1;
			Mat erodedM;
			erosionCC(erosion_elem, erosion_size, output, erodedM);
	

			///This has been added here (not in matlab)
			int dilation_elem = 2; //0 = morph_rect, 1 = morph_cross, 2 = morph_ellipse
			int dilation_size = 1;
			Mat dilatedM;
			dilationCC(dilation_elem, dilation_size, erodedM, dilatedM);


			stamp = dateStamp();
			auto_seg_log << stamp << " CWT post-processing done" << endl;

			//EXTRACT INFO FROM BINARY SEGMENTATION
			vector<vector<Point> > pixsLabel, contours;
			Mat labels, stats;

			extractInfo(dilatedM, pixsLabel, labels, stats, contours);


			//OPTIMISE CWT
			//areas
			vector<double> area(stats.rows - 1, 0); //nLabels-1 as i = 0 corresponds to background
			for (int i = 1; i < stats.rows; i++)
			{
				area.at(i - 1) = stats.at<int>(i, CC_STAT_AREA);
			}
			//mean
			double meanArea = std::accumulate(area.begin(), area.end(), 0) / (stats.rows - 1);

			//std
			std::vector<double> diff(area.size());
			std::transform(area.begin(), area.end(), diff.begin(), [meanArea](double x) { return x - meanArea; });
			double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
			double stdevArea = std::sqrt(sq_sum / area.size());

			int op = 3; //op = 1: lt; op = 2: leq; op = 3: gt; op = 4: geq;
			double thres = meanArea + stdevArea; //meanArea + 5*stdevArea
			vector<int> bigStones = findMatlab(area, op, thres);


			//Divide big stones. Erode big > new segments. New matrix (from dilateM) without big ones but adding new segments 
			Mat firstSegmentation;
			firstSegmentation = stonesDivision(pixsLabel, labels, bigStones);

			vector<vector<Point> > pixsLabel2, contours2;
			Mat labels2, stats2;
			extractInfo(firstSegmentation, pixsLabel2, labels2, stats2, contours2); //Extract info after big stone division


			stamp = dateStamp();
			auto_seg_log << stamp << " Big stones divided" << endl;


			Mat skel = skeleton(firstSegmentation, 1);

			//CONVEX IMAGES

			vector<Mat> channelsC(3);
			split(imageC, channelsC);
			Mat mapB = channelsC[0]; //3channel matrices are BGR
			Mat mapG = channelsC[1];
			Mat mapR = channelsC[2];

			// Find the convex hull object for each contour
			vector<vector<Point> >hull(contours2.size());
			for (int i = 0; i < contours2.size(); i++)
			{
				convexHull(Mat(contours2[i]), hull[i], false);
			}
			vector<vector<Point> > hull0;

			// Draw contours + hull results
			RNG rng(12345);
			vector<vector<Point> > pixsHull, contoursHull;
			vector<vector<Point> > pixsHullSt(contours2.size());
			Mat labelsHull, statsHull;
			Mat drawingAll = Mat::zeros(firstSegmentation.size(), CV_8UC3);

			// Segments growth
			vector<vector<Point>> segmentsColor;
			vector<Mat> matConvexSt;
			vector<int> pixsLabels(firstSegmentation.cols*firstSegmentation.rows, -1); //Store idxs in a vector instead of vector of vectors //FK2310
			vector<int> pixsLabels2(firstSegmentation.cols*firstSegmentation.rows, -1); //Store idxs in a vector instead of vector of vectors. Consider overlapping //FK2310
			for (int i = 0; i < contours2.size(); i++) //
			{
				Mat drawing = Mat::zeros(firstSegmentation.size(), CV_8UC1);

				Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
				Scalar color2 = Scalar(255, 255, 255);
				drawContours(drawing, hull, i, color2, -1, 8, vector<Vec4i>(), 0, Point()); //5th input param to -1 to colour segments
				drawContours(drawingAll, hull, i, color, -1, 8, vector<Vec4i>(), 0, Point()); //To plot

				extractInfo(drawing, pixsHull, labelsHull, statsHull, contoursHull); //Extract info from individual convex hulls

				pixsHullSt[i] = pixsHull[0];

				
				for (int j = 0; j < pixsHull[0].size(); j++) {
					int idx = corrMat.at<int>(pixsHull[0].at(j).x, pixsHull[0].at(j).y);
					if (pixsLabels.at(idx) == -1) {
						// Stones inside each other not considered as duplicated
						pixsLabels.at(idx) = i;
					}
					else {// Additional vector to consider pixels labelled as part of two stones (due to convex hull operation, for example)
						pixsLabels2.at(idx) = i;
					}
				}
			

				vector<Point> pixelsH = pixsHull[0]; //Segments base

				vector<int> coeffB = pixelValues(mapB, pixelsH);
				vector<int> coeffG = pixelValues(mapG, pixelsH);
				vector<int> coeffR = pixelValues(mapR, pixelsH);

				sort(coeffB.begin(), coeffB.end());		double medB = coeffB[coeffB.size() / 2];
				sort(coeffG.begin(), coeffG.end());		double medG = coeffG[coeffG.size() / 2];
				sort(coeffR.begin(), coeffR.end());		double medR = coeffR[coeffR.size() / 2];
				vector<double> coeff0 = { medB, medG, medR };

				int j = 1;
				int noPix = 0;
				double growthRate = 1000;
				Mat convexM, matNew;
				int dSize = 1;

				

			}

			stamp = dateStamp();
			auto_seg_log << stamp << " Convex hulls defined" << endl;

			// EXTRACT 3D POINTS PER SEGMENT


			for (int i = 0; i < contours2.size(); i++) //
			{
				vector<Point> pixels = pixsHullSt[i];

		
				////////// New 1D
				vector<int> pixels1d;
				for (auto e : pixels)
				{
					int idx = corrMat.at<int>(e.x, e.y);
					pixels1d.push_back(idx);
				}

			
				vector<int> idxCloud;
				for (int j = 0; j<idxPx.size(); j++) {
					if(pixsLabels.at(idxPx1D.at(j)) == i)
						idxCloud.push_back(j);
					if (pixsLabels2.at(idxPx1D.at(j)) == i) //Check potential second label (this will be cleaned afterwards)
						idxCloud.push_back(j);
				}
			

				vector<pair<int, int>> global_SglStoneIdx;


				int max_size = cloud->size() /1; // maximum size per single stone 
				float density = getDensity(cloud, idxCloud);
				float max_density = 40000;
				if (idxCloud.size() < max_size/10) 
				{
					for (auto e : idxCloud)
					{
						if (e < idxClouds[z].size())          
							global_SglStoneIdx.push_back(make_pair(idxClouds[z][e], stoneIdx));

						else
						{
							auto_seg_log <<"====> Error :" << z << "	" << e <<endl;
						}

					}


					stones.push_back(global_SglStoneIdx);
					stoneIdx++;

				}

				idxCloud.clear();
				global_SglStoneIdx.clear();
				pixels.clear();
				pixels1d.clear();
				


			}

			stamp = dateStamp();
			auto_seg_log << stamp << " 3D points per segment extracted" << endl;

			/*** HERE ENDS THE ACTION ***/

			//Deleting releasing memory ?
			stamp = dateStamp();
			auto_seg_log << stamp << " Point cloud processed. Releasing memory ... " << endl;

			cComment.erase();
			qComment.clear();

			
			
			corrMat.release();
			image.release();
			imageR.release();
			imageG.release();
			imageC.release();
			channels.clear();
			imageFilt.release();
			imagePlot.release();
			cwtcfs.release();
			bincfs.release();
			
			hola.release();
			output.release();
			erodedM.release();
			dilatedM.release();
			pixsLabel.clear();
			contours.clear();
			labels.release();
			stats.release();
			area.clear();
			diff.clear();
			bigStones.clear();
			firstSegmentation.release();
			pixsLabel2.clear();
			contours2.clear();
			labels2.release();
			skel.release();
			channelsC.clear();
			mapB.release();
			mapG.release();
			mapR.release();
			hull.clear();
			hull0.clear();
			pixsHull.clear();
			contoursHull.clear();
			pixsHullSt.clear();
			statsHull.release();
			drawingAll.release();
			segmentsColor.clear();
			matConvexSt.clear();
		

			pointC = nullptr;
			



		}
		stamp = dateStamp();

		cloud->removeMetaData("cloud");

		cloud->clear();
	
	}

	stamp = dateStamp();
	auto_seg_log << stamp << " Stitching stones" << endl;

	//concatenating all stone's cloudidx and stoneidx pairs into one vector instead of a vector of vectors with separate stone in each vector

	vector<pair<int, int>> allStones;
	
	

	if (stones.size()>0)
		allStones= stones[0];

	for (unsigned int i = 1; i < stones.size();i++)
	{
		allStones.insert(allStones.end(), stones[i].begin(), stones[i].end());
	}
	stamp = dateStamp();
	auto_seg_log << stamp << " Total number of unstitched stones: " << stones.size() << endl;


	//Start of stiching

	//indentifying and merging indentical stones after the stitching
	//This can be defined as a function


	// sorting a vector of pairs 
	sort(allStones.begin(), allStones.end(), [](auto &left, auto &right) {return left.first < right.first;});
	stamp = dateStamp();
	auto_seg_log << stamp << " Start looking for whole stones " << endl;


	unsigned int j = 0;
	unsigned int i = 0;
	while (i < allStones.size())
	{
		j = i+1;

		while (j< allStones.size() && allStones[i].first == allStones[j].first && allStones[i].second != allStones[j].second)

		{

			unsigned int oldIdx = allStones[j].second;
			unsigned int newIdx = allStones[i].second;

			for (unsigned int k = 0; k<allStones.size();k++)
			{
				if (allStones[k].second == oldIdx)
					allStones[k].second = newIdx;
			}

			j++;
			
		}

		i = j;
		
	}

	//End of stitching
	my_time = time(NULL);
	stamp = dateStamp();
	auto_seg_log << stamp << " Indices corrected " << endl;

	//eliminating repeated pairs ( originaly common points in stones)

	allStones.erase(unique(allStones.begin(), allStones.end()), allStones.end());

	stamp = dateStamp();
	auto_seg_log << stamp << " Repeated pairs removed " << endl;


	int b = cloud0->size();
	vector<int> allStones_Idx; // storing only cloud index of points that's within stones
	vector<int> stoneId; // saving all the existing and used indices for stones 
	vector<int> allMortar;

	stamp = dateStamp();
	auto_seg_log << stamp << " Getting all stones idx " << endl;

	for (auto e : allStones)
		allStones_Idx.push_back(e.first);


	sort(allStones_Idx.begin(), allStones_Idx.end()); 



	// Mortar idx
	std::set_difference(global_idx.begin(), global_idx.end(), allStones_Idx.begin(),	allStones_Idx.end(),std::inserter(allMortar, allMortar.end()));

	stamp = dateStamp();
	auto_seg_log << stamp << " Mortar idx identified " << endl;

	// correcting stone indices (make them 1,2,3 .... in order)

	for (auto e : allStones)
		stoneId.push_back(e.second);

	

	sort(stoneId.begin(), stoneId.end());

	stoneId.erase(unique(stoneId.begin(), stoneId.end()), stoneId.end());

	vector<vector<int>> vecStones(stoneId.size());

	

	sort(allStones.begin(), allStones.end(), [](auto &left, auto &right) {return left.second < right.second;});

	int old_idx = allStones[0].second;
	int new_idx = 0;

	for (unsigned i = 0; i < allStones.size(); i++)
	{
		if (allStones[i].second == old_idx)
		{
			allStones[i].second = new_idx;
			vecStones[new_idx].push_back(allStones[i].first);

		}


		else
		{
			new_idx++;
			old_idx = allStones[i].second;
			allStones[i].second = new_idx;
			vecStones[new_idx].push_back(allStones[i].first);

		}

	}



	stoneId.clear();

	allStones_Idx.clear(); // values were sorted


	for (auto e : allStones)
	{
		allStones_Idx.push_back(e.first);
		stoneId.push_back(e.second);
	}
		
	
	// suffling stone IDs for better visuals
	vector <int> stIdShuffle = stoneId;

	sort(stIdShuffle.begin(), stIdShuffle.end());

	stIdShuffle.erase(unique(stIdShuffle.begin(), stIdShuffle.end()), stIdShuffle.end());
	
	auto rng = std::default_random_engine{};
	std::shuffle(stIdShuffle.begin(), stIdShuffle.end(), rng);


	
	// generating final clouds
	stamp = dateStamp();
	auto_seg_log << stamp << " Writing final clouds " << endl;

	ccPointCloud* f_cloudStones = new ccPointCloud("f_cloudStones");
	ccPointCloud* f_cloudMortar = new ccPointCloud("f_cloudMortar");

	//stones
	if (allStones_Idx.size() > 0)
	{
		for (auto e : allStones_Idx)
		{
			f_cloudStones->reserveThePointsTable(1);
			f_cloudStones->reserveTheRGBTable();
			CCVector3 p(cloud0->getPoint(e)->x, cloud0->getPoint(e)->y, cloud0->getPoint(e)->z);
			f_cloudStones->addPoint(p);

			const ccColor::Rgb col = cloud0->getPointColor(e);
			f_cloudStones->addColor(col);

		}
	}

	stamp = dateStamp();
	auto_seg_log << stamp << " Stone cloud OK. " << allStones_Idx.size() << " pts. " << vecStones.size() << " stones." << endl;


	// adding scalar field of stoneId to stone point cloud

	CCCoreLib::ScalarField* stIdSF = nullptr;


	int sfIdx = f_cloudStones->getScalarFieldIndexByName("Stone Index");
	if (sfIdx < 0)
	{
		sfIdx = f_cloudStones->addScalarField("Stone Index");
	}
	if (sfIdx < 0)
	{
				return;
	}

	stIdSF = f_cloudStones->getScalarField(sfIdx);


	if (allStones_Idx.size() > 0)
	{
		for (unsigned int i = 0; i < stoneId.size();i++)
		{
			int index = static_cast<int>(stIdShuffle[stoneId[i]]);
			stIdSF->setValue(i, index);

		}
	}

	
	f_cloudStones->setCurrentDisplayedScalarField(sfIdx);

	stIdSF->computeMinAndMax();

	stamp = dateStamp();
	auto_seg_log << stamp << " Stone cloud SF OK " << endl;


	// Mortar

	if (allMortar.size() > 0)
	{
		for (auto e : allMortar)
		{
			f_cloudMortar->reserveThePointsTable(1);
			f_cloudMortar->reserveTheRGBTable();
			CCVector3 p(cloud0->getPoint(e)->x, cloud0->getPoint(e)->y, cloud0->getPoint(e)->z);
			f_cloudMortar->addPoint(p);

			const ccColor::Rgb col = cloud0->getPointColor(e);
			f_cloudMortar->addColor(col);

		}
		
	}

	stamp = dateStamp();
	auto_seg_log << stamp << " Mortar OK. " << allMortar.size() << " pts" << endl;
	


	//ADDING CLOUDS TO DB

	ccPointCloud* mortarMaps = nullptr;
	if (checkMortar == true) {
		stamp = dateStamp();
		auto_seg_log << " Starting mortar maps generation... " << endl;

		if (allMortar.size() > 0)
			mortarMaps = getMortarMaps(f_cloudStones, f_cloudMortar);

		stamp = dateStamp();
		auto_seg_log << stamp << " Mortar maps OK " << endl;

		mortarMaps->showSF(true);
	}



	// Move all the clouds back
		cloud0->translate(minBox);
		cloud0->applyRigidTransformation(R3.inverse());
		cloud0->applyRigidTransformation(R2.inverse());

		if (allMortar.size() > 0) {
			f_cloudMortar->translate(minBox);
			f_cloudMortar->applyRigidTransformation(R3.inverse());
			f_cloudMortar->applyRigidTransformation(R2.inverse());
		}

		if (allStones.size() > 0) {
			f_cloudStones->translate(minBox);
			f_cloudStones->applyRigidTransformation(R3.inverse());
			f_cloudStones->applyRigidTransformation(R2.inverse());
		}


		if (checkMortar == true) {
			mortarMaps->translate(minBox);
			mortarMaps->applyRigidTransformation(R3.inverse());
			mortarMaps->applyRigidTransformation(R2.inverse());
		}
		
		//Generating contour polylines

		stamp = dateStamp();
		auto_seg_log << stamp << " Start of contour generation " << endl;

		vector<ccPolyline*> contours;

		ccHObject* contoursContainer = new ccHObject("Stone contours");

		int cntSt = 0;


		// contours in an alternative way using stone indexes and original cloud
		vector<double> stonesSize;
		if (vecStones.size() > 0)
		{
			for (auto ve : vecStones)
			{
				string s_name = "stone " + to_string(cntSt);
				QString qs_name = QString::fromUtf8(s_name.c_str());

				ccPolyline* contour = nullptr;

				if (ve.size() > 0)
				{
					contour = contourPoly2(cloud0, ve, qs_name);
					contoursContainer->addChild(contour);
					cntSt++;
					//Get BB per stone from contour.
					CCVector3 bbMinS;
					bbMinS = CCVector3(0, 0, 0);
					CCVector3 bbMaxS;
					bbMaxS = CCVector3(0, 0, 0);
					contour->getBoundingBox(bbMinS, bbMaxS);
					double sizeS = (bbMaxS.x - bbMinS.x)*(bbMaxS.z - bbMinS.z);
					stonesSize.push_back(sizeS);

				}
			}

			//Mean and median size of stones

			double meanSizeStones = accumulate(stonesSize.begin(), stonesSize.end(), 0.0) / stonesSize.size();
			double standDev = stddev(stonesSize);

			stamp = dateStamp();
			auto_seg_log << stamp << " Average size of stones (BB) " << meanSizeStones << " m2 +-" << standDev << " (" << meanSizeStones * 10000 << " cm2)" << endl;


			sort(stonesSize.begin(), stonesSize.end());
			bool evenAmount = (stonesSize.size() % 2 == 0);
			int oddAmountIdx = ceil(stonesSize.size() / 2);
			double evenAmountMedian = (stonesSize[oddAmountIdx - 1] + stonesSize[oddAmountIdx]) / 2;
			double medianSizeStones = evenAmount ? evenAmountMedian : stonesSize[oddAmountIdx];

			stamp = dateStamp();
			auto_seg_log << stamp << " Median size of stones (BB) " << medianSizeStones << " m2 (" << medianSizeStones * 10000 << " cm2)" << endl;
	
		}

		stamp = dateStamp();
		auto_seg_log << stamp << " Contour generation done " << endl;



		f_cloudStones->showColors(true);

		if (allMortar.size() > 0)
		{
			f_cloudMortar->showColors(true);
		}
	
	m_app->addToDB(f_cloudStones, true, true, false, true);


	if (allMortar.size() > 0)
		m_app->addToDB(f_cloudMortar, true, true, false, true);

	stamp = dateStamp();
	auto_seg_log << stamp << " Mortar cloud OK " << endl;


	m_app->addToDB(contoursContainer, true, true, false, true);

	
	if (checkMortar == true && allMortar.size() > 0)
		m_app->addToDB(mortarMaps, true, true, false, true);





	ccGLWindow* win = m_app->getActiveGLWindow();
	win->setView(CC_FRONT_VIEW);





}
