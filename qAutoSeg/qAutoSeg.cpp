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
//#								 v1.3                                      #
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
#include <ccGLWindowInterface.h>

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

//Calculate std
static double stddev(const std::vector<double>& func)
{
	if (func.size() < 2)
	{
		assert(false);
		return 0.0;
	}

	double mean = std::accumulate(func.begin(), func.end(), 0.0) / func.size();
	double sq_sum = std::inner_product(func.begin(), func.end(), func.begin(), 0.0,
		[](double const & x, double const & y) { return x + y; },
		[mean](double const & x, double const & y) { return (x - mean)*(y - mean); });
	return std::sqrt(sq_sum / (func.size() - 1));
}

static std::string dateStamp()
{
	return QDateTime::currentDateTime().toString("hh:mm:ss.zzz").toStdString();
}

static double optRotY(const ccPointCloud& cloud)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl_cloud->width = cloud.size();
	pcl_cloud->height = 1;
	pcl_cloud->points.resize(pcl_cloud->width * pcl_cloud->height);

	std::vector<int> v(cloud.size());
	std::iota(v.begin(), v.end(), 0);
	std::random_shuffle(v.begin(), v.end());
	v.erase(v.begin(), v.begin() + cloud.size());

	for (unsigned i = 0; i < cloud.size(); i++)
	{
		const CCVector3* P = cloud.getPoint(i);
		pcl_cloud->points[i].x = P->x;
		pcl_cloud->points[i].y = P->y;
		pcl_cloud->points[i].z = P->z;
	}

	pcl::PointXYZRGB bbMin0, bbMax0;
	pcl::getMinMax3D(*pcl_cloud, bbMin0, bbMax0);

	pcl::PointCloud<pcl::PointXYZRGB> ::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	transformed_cloud->width = cloud.size();
	transformed_cloud->height = 1;
	transformed_cloud->points.resize(transformed_cloud->width * transformed_cloud->height);

	double th0 = 0;
	double area0 = (bbMax0.x - bbMin0.x)*(bbMax0.z - bbMin0.z);
	for (int th = -45; th <= 45; th++)
	{
		//Apply this to a sparser cloud to make the process lighter
		double thD = th * (M_PI / 180.0);
		
		Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

		// Define a translation
		transform_2.translation() << 0, 0.0, 0.0;

		// The same rotation matrix as before; theta radians around Y axis
		transform_2.rotate(Eigen::AngleAxisf(thD, Eigen::Vector3f::UnitY()));

		pcl::transformPointCloud(*pcl_cloud, *transformed_cloud, transform_2);

		pcl::PointXYZRGB bbMin, bbMax;
		pcl::getMinMax3D(*transformed_cloud, bbMin, bbMax);
		double area = (bbMax.x - bbMin.x)*(bbMax.z - bbMin.z);
		if (area < area0)
		{
			area0 = area;
			th0 = thD;
		}

		Eigen::Affine3f transform_inv = transform_2.inverse();
		pcl::transformPointCloud(*transformed_cloud, *pcl_cloud, transform_inv);
	}

	return th0;
}

#if 0 // not used
static ccPointCloud* rotY(const ccPointCloud& cloud, double th0)
{
	bool hasColors = cloud.hasColors();

	pcl::PointCloud<pcl::PointXYZRGB> ::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl_cloud->width = cloud.size();
	pcl_cloud->height = 1;
	pcl_cloud->points.resize(pcl_cloud->width * pcl_cloud->height);

	std::vector<int> v(cloud.size());
	std::iota(v.begin(), v.end(), 0);
	std::random_shuffle(v.begin(), v.end());
	v.erase(v.begin(), v.begin() + cloud.size());

	for (size_t i = 0; i < cloud.size(); i++)
	{
		pcl_cloud->points[i].x = cloud.getPoint(i)->x;
		pcl_cloud->points[i].y = cloud.getPoint(i)->y;
		pcl_cloud->points[i].z = cloud.getPoint(i)->z;

		if (hasColors)
		{
			pcl_cloud->points[i].r = cloud.getPointColor(i).r;
			pcl_cloud->points[i].g = cloud.getPointColor(i).g;
			pcl_cloud->points[i].b = cloud.getPointColor(i).b;
		}
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

	//From PCL to CC object
	ccPointCloud* cloudAligned = new ccPointCloud("Aligned");
	if (!cloudAligned->reserve(static_cast<unsigned>(transformed0->size())))
	{
		//not enough memory
		delete cloudAligned;
		return nullptr;
	}
	if (hasColors && !cloudAligned->reserveTheRGBTable())
	{
		//not enough memory to preserve colors
		hasColors = false;
	}

	for (size_t i = 0; i < transformed0->size(); i++)
	{
		CCVector3 p(transformed0->points[i].x, transformed0->points[i].y, transformed0->points[i].z);
		cloudAligned->addPoint(p);

		if (hasColors)
		{
			const auto& rgb = transformed0->points[i];
			const ccColor::Rgb col{ rgb.r, rgb.g, rgb.b };
			cloudAligned->addColor(col);
		}
	}

	return cloudAligned;
}

#endif

static void cloud2binary(	const ccPointCloud& cloud,
							cv::Mat& corrMatS,
							std::vector<cv::Point>& idxPxS,
							std::vector<int>& idxPx1DS,
							cv::Mat& imageBWS )
{
	//Correspondence 2D->1D. Each px has an index instead of two coords
	int cntC = 0;
	for (int y = 0; y < corrMatS.rows; y++)
	{
		for (int x = 0; x < corrMatS.cols; x++)
		{
			corrMatS.at<int>(y, x) = cntC;
			cntC++;
		}
	}

	//Binary
	unsigned s = cloud.size();
	idxPxS.reserve(s);
	idxPx1DS.reserve(s);
	for (unsigned i = 0; i < s; i++)
	{
		//depth
		const CCVector3* pointC0 = cloud.getPoint(i);
		int z = std::floor(pointC0->z);
		int x = std::floor(pointC0->x);
		imageBWS.at<uchar>(z, x) = 255;

		//idxPx per 3D point
		idxPxS.push_back(cv::Point(z, x));
		idxPx1DS.push_back(corrMatS.at<int>(z, x));
	}
}

//Extract Skeleton from Binary (CV_8U)
static cv::Mat skeleton(const cv::Mat& I, bool flagInv)
{
	cv::Mat toSkel;
	if (flagInv)
	{
		//If black-white inversion is required
		toSkel = 255 * cv::Mat::ones(I.rows, I.cols, CV_8U) - I;
	}
	else
	{
		toSkel = I;
	}

	cv::Mat skel(toSkel.size(), CV_8U, cv::Scalar(0));
	cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));

	int iterations = 0;
	bool done = false;
	do
	{
		cv::Mat eroded;
		cv::erode(toSkel, eroded, element);
		cv::Mat temp;
		cv::dilate(eroded, temp, element);
		cv::subtract(toSkel, temp, temp);
		cv::bitwise_or(skel, temp, skel);
		eroded.copyTo(toSkel);

		done = (cv::countNonZero(toSkel) == 0);
		iterations++;
	}
	while (!done && (iterations < 100));

	return skel;
}

#if 0 // not used
//Extract elements of a vector given the indices
static std::vector<cv::Point> extractVectorFromIdx(const std::vector<cv::Point>& v, const std::vector<int>& idx)
{
	std::vector<cv::Point> subset;
	subset.reserve(idx.size());
	for (size_t i = 0; i < idx.size(); i++)
	{
		subset.push_back(v[idx[i]]);
	}
	return std::move(subset);
}

//Populate a matrix with a given value
static void populateMat(cv::Mat& populated, const std::vector<cv::Point>& pixList, int value)
{
	for (size_t i = 0; i < pixList.size(); i++)
	{
		populated.at<uchar>(pixList[i].x, pixList[i].y) = value;
	}
}

//Calculate difference set in two pixel lists (Matlab setdiff equivalent). Returns pixels in listOld that are not in listNew
static std::vector<cv::Point> setdiffPixs(const std::vector<cv::Point>& listNew, const std::vector<cv::Point>& listOld)
{
	std::vector<cv::Point> setdiffList;

	for (size_t i = 0; i < listOld.size(); i++)
	{
		bool same = false;
		for (size_t j = 0; j < listNew.size(); j++)
		{
			if (listOld[i].x == listNew[j].x && listOld[i].y == listNew[j].y)
			{
				same = true;
				break; // no need to process any further
			}
		}
		if (!same)
		{
			setdiffList.push_back(listOld[i]);
		}

	}
	return std::move(setdiffList);
}

//Calculate intersection set in two pixel lists. Returns idx pixels in listNew that are in listOld
static std::vector<int> setIntersectIdxPixs(const std::vector<cv::Point>& listNew, const std::vector<cv::Point>& listOld)
{
	std::vector<int> commonList;

	//Common
	for (size_t i = 0; i < listOld.size(); i++)
	{
		for (size_t j = 0; j < listNew.size(); j++)
		{
			if (listOld[i].x == listNew[j].x && listOld[i].y == listNew[j].y)
			{
				commonList.push_back(i);
			}
		}
	}

	return std::move(commonList);
}

//Calculate exclusive OR in two pixel lists (Matlab setxor equivalent)
static std::vector<cv::Point> setxorPixs(std::vector<cv::Point> listNew, std::vector<cv::Point> listOld)
{
	//Common
	std::vector<int> commonList;
	for (size_t i = 0; i < listNew.size(); i++)
	{
		for (size_t j = 0; j < listNew.size(); j++)
		{
			if (listOld[i].x == listNew[j].x && listOld[i].y == listNew[j].y)
			{
				commonList.push_back(i);
			}
		}
	}

	//Setxor
	size_t nLabels = listNew.size();
	std::vector<int> v(nLabels);
	std::iota(std::begin(v), std::end(v), 0);
	std::vector<int> setxorIdx;
	std::set_difference(std::begin(v), std::end(v), std::begin(commonList), std::end(commonList), std::back_inserter(setxorIdx));

	std::vector<cv::Point> setxorList;
	setxorList.reserve(setxorIdx.size());
	for (size_t j = 0; j < setxorIdx.size(); j++)
	{
		setxorList.push_back(listNew[setxorIdx[j]]);
	}

	return std::move(setxorList);
}

static std::vector<int> findIdx(const std::vector<int>& inputV, int thres)
{
	std::vector<int> indices;

	for (int i = 0; i < inputV.size(); i++)
	{
		if (inputV[i] == thres)
			indices.push_back(i);
	}

	return indices;
}

//Calculates the conjugate of a complex matrix
static cv::Mat conjugate(cv::Mat_<std::complex<double>> M1, bool* flag)
{
	using CP = std::complex<double>;
	cv::Mat_<CP> M3(M1.rows, M1.cols);
	//When there is no complex part, half of the values of the real part are considered as complex
	try
	{
		double a = M1.at<CP>(M1.rows - 1, M1.cols - 1).real();
		for (int i = 0; i < M1.rows; ++i)
		{
			for (int j = 0; j < M1.cols; ++j)
			{

				M3.at<CP>(i, j) = CP(M1.at<CP>(i, j).real(), -M1.at<CP>(i, j).imag());
			}
		}

		*flag = true;
		return M3;
	}
	catch (const std::exception)
	{
		//If there is no complex part
		M3 = M1;
		*flag = false;
		return M3;
	}
}
#endif

std::vector<int> setIntersectIdxPixs1D(const std::vector<int>& listNew, const std::vector<int>& listOld)
{
	std::vector<int> commonList;

	//Common
	for (size_t i = 0; i < listNew.size(); ++i)
	{
		int e = listNew[i];

		for (auto ee : listOld)
		{
			if (e == ee)
			{
				commonList.push_back(i);
				break;
			}
		}
	}

	return std::move(commonList);
}

//Extract values of pixels per labelled segment in a binary image
static std::vector<int> pixelValues(const cv::Mat& inputM, const std::vector<cv::Point>& pixelList)
{
	std::vector<int> pixelV;
	pixelV.reserve(pixelList.size());
	for (size_t i = 0; i < pixelList.size(); i++)
	{
		int val = inputM.at<uchar>(pixelList[i].x, pixelList[i].y);
		pixelV.push_back(val);
	}

	return std::move(pixelV);
}

//Extract pixels per labelled segment in a binary image
std::vector<std::vector<cv::Point>> pixelListMat(const cv::Mat& inputM, int nLabels)
{
	std::vector<std::vector<cv::Point>> pixelSegments(nLabels);
	for (int i = 0; i < inputM.rows; i++)
	{
		for (int j = 0; j < inputM.cols; j++)
		{
			int pixelValue = inputM.at<int>(i, j);
			if (pixelValue > 0)
			{
				cv::Point x(i, j);
				pixelSegments[pixelValue - 1].push_back(x); //-1 to correct value 0 for background
			}

		}
	}

	return std::move(pixelSegments);
}

//Find values over/under a value in an array. op = 1: lt; op = 2: leq; op = 3: gt; op = 4: geq; op = 5: eq
std::vector<int> findMatlab(const std::vector<double>& inputV, int op, double thres)
{
	std::vector<int> indices;

	switch (op)
	{
	case 1:
		for (int i = 0; i < inputV.size(); i++)
		{
			if (inputV[i] < thres)
				indices.push_back(i);
		}
		break;

	case 2:
		for (int i = 0; i < inputV.size(); i++)
		{
			if (inputV[i] <= thres)
				indices.push_back(i);
		}
		break;

	case 3:
		for (int i = 0; i < inputV.size(); i++)
		{
			if (inputV[i] > thres)
				indices.push_back(i);
		}
		break;

	case 4:
		for (int i = 0; i < inputV.size(); i++)
		{
			if (inputV[i] >= thres)
				indices.push_back(i);
		}
		break;

	case 5:
		for (int i = 0; i < inputV.size(); i++)
		{
			if (inputV[i] == thres)
				indices.push_back(i);
		}
		break;

	default:
		assert(false);
		break;
	}

	return std::move(indices);
}

//Dilation
static void dilationCC(int dilation_elem, int dilation_size, cv::Mat &src, cv::Mat &dilation_dst)
{
	int dilation_type = cv::MORPH_RECT;
	if (dilation_elem == 0)
	{
		//dilation_type = cv::MORPH_RECT;
	}
	else if (dilation_elem == 1)
	{
		dilation_type = cv::MORPH_CROSS;
	}
	else if (dilation_elem == 2)
	{
		dilation_type = cv::MORPH_ELLIPSE;
	}
	else
	{
		assert(false);
	}

	cv::Mat element = cv::getStructuringElement(dilation_type,
												cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1),
												cv::Point(dilation_size, dilation_size));

	dilate(src, dilation_dst, element);
}

// Erosion
static void erosionCC(int erosion_elem, int erosion_size, cv::Mat& src, cv::Mat& erosion_dst)
{
	int erosion_type = cv::MORPH_RECT;
	if (erosion_elem == 0)
	{
		//erosion_type = cv::MORPH_RECT;
	}
	else if (erosion_elem == 1)
	{
		erosion_type = cv::MORPH_CROSS;
	}
	else if (erosion_elem == 2)
	{
		erosion_type = cv::MORPH_ELLIPSE;
	}
	else
	{
		assert(false);
	}

	cv::Mat element = cv::getStructuringElement(erosion_type,
												cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
												cv::Point(erosion_size, erosion_size));

	erode(src, erosion_dst, element);
}


//Stones division in smaller segments (when joint by narrow areas)
static cv::Mat stonesDivision(const std::vector<std::vector<cv::Point>> pixsLabel, const cv::Mat& labels, const std::vector<int>& bigStones)
{
	//smallIndices
	size_t nLabels = pixsLabel.size();
	std::vector<int> v(nLabels);
	std::iota(std::begin(v), std::end(v), 0);

	std::vector<int> noBigStones;
	std::set_difference(std::begin(v), std::end(v),std::begin(bigStones), std::end(bigStones),std::back_inserter(noBigStones));
	
	//smallMatrix
	cv::Mat smallStones = cv::Mat::zeros(labels.size(), CV_8U);
	for (size_t i = 0; i < noBigStones.size(); i++)
	{
		const auto& pixsLabelI = pixsLabel[noBigStones[i]];
		for (size_t j = 0; j < pixsLabelI.size(); j++)
		{
			smallStones.at<uchar>(pixsLabelI[j].x, pixsLabelI[j].y) = 255;
		}
	}

	//bigMatrix
	cv::Mat input = cv::Mat::zeros(labels.size(), CV_8U);
	for (size_t i = 0; i < bigStones.size(); i++)
	{
		const auto& pixsLabelI = pixsLabel[bigStones[i]];
		for (size_t j = 0; j < pixsLabelI.size(); j++)
		{
			input.at<uchar>(pixsLabelI[j].x, pixsLabelI[j].y) = 255;
		}
	}

	//Erosion of big stones (in matlab was open: erosion + dilation, but here we dilated before) 
	int erosion_elem = 2;
	int erosion_size = 1;
	cv::Mat output;
	erosionCC(erosion_elem, erosion_size, input, output);

	//Combine small + eroded big stones
	cv::Mat combined = output+smallStones;

	return combined; //new segments
}

// Gets only the biggest segments
static cv::Mat threshSegments(cv::Mat &src, double threshSize)
{
	// FindContours:
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::Mat srcBuffer, output;
	src.copyTo(srcBuffer);
	findContours(srcBuffer, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_TC89_KCOS);

	std::vector<std::vector<cv::Point>> allSegments;

	// For each segment:
	for (size_t i = 0; i < contours.size(); ++i)
	{
		cv::drawContours(srcBuffer, contours, i, cv::Scalar(200, 0, 0), 1, 8, hierarchy, 0, cv::Point());
		cv::Rect brect = cv::boundingRect(contours[i]);
		cv::rectangle(srcBuffer, brect, cv::Scalar(255, 0, 0));

		std::vector<cv::Point> segment;
		for (int row = brect.y; row < brect.y + brect.height; ++row)
		{
			for (int col = brect.x; col < brect.x + brect.width; ++col)
			{
				int result = pointPolygonTest(contours[i], cv::Point(col, row), false);
				if (result == 1 || result == 0)
				{
					segment.push_back(cv::Point(col, row));
				}
			}
		}

		allSegments.push_back(segment);
	}

	output = cv::Mat::zeros(src.size(), CV_8U);
	int totalSize = output.rows*output.cols;
	for (int segmentCount = 0; segmentCount < allSegments.size(); ++segmentCount)
	{
		const std::vector<cv::Point>& segment = allSegments[segmentCount];
		if (segment.size() > threshSize)
		{
			for (size_t idx = 0; idx < segment.size(); ++idx)
			{
				output.at<uchar>(segment[idx].y, segment[idx].x) = 255;
			}
		}
	}

	return output;
}

// Returns the type of a matrix as a string
static std::string type2str(int type)
{
	std::string r;

	uchar depth = type & CV_MAT_DEPTH_MASK;
	uchar chans = 1 + (type >> CV_CN_SHIFT);

	switch (depth)
	{
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
static std::vector<std::vector<cv::Point>> extractInfo(cv::Mat &input, cv::Mat& labels, cv::Mat& stats, std::vector<std::vector<cv::Point>>* contours = nullptr)
{
	//Get connected components and stats
	const int connectivity_8 = 8;
	cv::Mat centroids;

	int nLabels = connectedComponentsWithStats(input, labels, stats, centroids, connectivity_8, CV_32S); //Note that component 0 is the background (i.e. nLabels = nsegments+1)

	//Extract the contours 
	std::vector<cv::Vec4i> hierarchy;
	std::vector<std::vector<cv::Point>> contours0;
	cv::findContours(input, contours ? *contours : contours0, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

	std::string ty = type2str(labels.type());

	return std::move(pixelListMat(labels, nLabels - 1));
}

//Fill in the holes inside segments
static void fillEdgeImage(const cv::Mat& edgesIn, cv::Mat& filledEdgesOut, int z)
{
	cv::Mat edgesNeg = edgesIn.clone();
	std::string hola = type2str(edgesNeg.type());

	////////Check that seed is in mortar joints and not inside a stone
	cv::Mat labels, stats;
	const int connectivity_8 = 8;
	cv::Mat centroids;
	//invert 1s and 0s to label black areas from CWT instead of whites. In this way, we know that label 1 (the bigger segment) corresponds to mortar joints
	cv::bitwise_not(edgesIn, edgesIn);

	int nLabels = connectedComponentsWithStats(edgesIn, labels, stats, centroids, connectivity_8, CV_32S); //Note that component 0 is the background (i.e. nLabels = nsegments+1)
	
	cv::bitwise_not(edgesIn, edgesIn);

	int stSize = stats.at<int>(1, 4); //0 is stone area, largest (not considering that one) is continuous mortar
	int	cntBiggestSize = 1;
	for (int i = 2; i < stats.rows; i++)
	{
		int loopSize = stats.at<int>(i, 4);

		if (loopSize > stSize)
		{		
			stSize = loopSize;
			cntBiggestSize = i;
		}
	}

	int f = 0, c = 0;
	for (int i = 0; i < edgesNeg.rows; i++)
	{
		for (int j = 0; j < edgesNeg.cols; j++)
		{
			uchar h = edgesNeg.at<uchar>(i, j);
			int l = labels.at<int>(i, j);
			if (h == 0 && l == cntBiggestSize)
			{
				// seed is a 0 in CWT and belongs to mortar joints
				f = i;
				c = j;
				goto stop;
			}
		}
	}

stop:

	cv::floodFill(edgesNeg, cv::Point(c, f), CV_RGB(255, 255, 255)); //The seed must be a black pixel

	cv::bitwise_not(edgesNeg, edgesNeg);

	filledEdgesOut = (edgesNeg | edgesIn);

	return;
}

// Calculates the fft2
static void fft2(const cv::Mat& in, cv::Mat& complexI)
{
	cv::Mat padded;
	int m = cv::getOptimalDFTSize(in.rows);
	int n = cv::getOptimalDFTSize(in.cols);
	copyMakeBorder(in, padded, 0, m - in.rows, 0, n - in.cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));

	cv::Mat planes[] { cv::Mat_<float>(padded), cv::Mat::zeros(padded.size(), CV_32F) };
	cv::merge(planes, 2, complexI);
	cv::dft(complexI, complexI);
}

// Multiplies two complex matrices
static cv::Mat multiplicationCC(const cv::Mat_<std::complex<double>>& M1, const cv::Mat_<std::complex<double>>& M2)
{
	using CP = std::complex<double>;
	cv::Mat_<CP> M3(M1.rows, M1.cols);
	for (int i = 0; i < M3.rows; ++i)
	{
		for (int j = 0; j < M3.cols; ++j)
		{
			M3.at<CP>(i, j) = CP(M1.at<CP>(i, j).real()*M2.at<CP>(i, j).real() - M1.at<CP>(i, j).imag()*M2.at<CP>(i, j).imag(), M1.at<CP>(i, j).real()*M2.at<CP>(i, j).imag() + M1.at<CP>(i, j).imag()*M2.at<CP>(i, j).real());
		}
	}
	return M3;
}

//Calculates the Continuous Wavelet Transform
static void cwt2d(double sca, cv::Mat& image, cv::Mat& cwtcfs, cv::Mat& bincfs)
{
	cv::Mat I = cv::imread("depthMap.bmp", cv::IMREAD_GRAYSCALE);
	cv::Mat fImage = image; //(if there is 'image' matrix)

	// FFT
	//std::cout << "Direct transform...\n";
	cv::Mat fourierTransform;
	fft2(fImage, fourierTransform);
	
	//Create frequency plane
	cv::Size S = fourierTransform.size();
	int H = S.height;
	int W = S.width;

	int W2 = (W - 1) / 2;
	int H2 = (H - 1) / 2;

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

	cv::Mat matW(W_puls);
	cv::Mat matH(H_puls);

	cv::Mat1d xx, yy;
	cv::repeat(matW.reshape(1, 1), matH.total(), 1, xx); //Matlab's Meshgrid equivalent
	cv::repeat(matH.reshape(1, 1).t(), 1, matW.total(), yy);

	double ang = 0;
	cv::Mat nxx = sca * (cos(ang)*xx - sin(ang)*yy);
	cv::Mat nyy = sca * (sin(ang)*xx + cos(ang)*yy);

	int sigmax = 1;
	int sigmay = 1;
	int order = 2;

	cv::Mat nxx2;
	cv::pow(nxx, 2, nxx2);
	cv::Mat nyy2;
	cv::pow(nyy, 2, nyy2);

	cv::Mat sum = nxx2 + nyy2;
	cv::Mat sumPow;
	cv::pow(sum, order / 2, sumPow);

	cv::Mat prod1 = sigmax * nxx;
	cv::Mat prod12;
	cv::pow(prod1, 2, prod12);
	cv::Mat prod2 = sigmay * nyy;
	cv::Mat prod22;
	cv::pow(prod2, 2, prod22);

	cv::Mat ex = -(prod12 + prod22) / 2;
	cv::Mat ex2;
	cv::exp(ex, ex2);

	cv::Mat waveft2 = -(2 * M_PI) * sumPow.mul(ex2);

	//Complex mask
	using CP = std::complex<double>;
	cv::Mat_<CP> mask(cv::Size(waveft2.rows, waveft2.cols));
	mask = sca * waveft2;

	bool flag = false;
	cv::Mat mask_conj = mask;
	mask.convertTo(mask_conj, CV_32F);

	cv::Mat g, mask2;
	std::vector<cv::Mat> channels;
	if (flag)
	{
		//Mask is complex. TBC if there is the case
	}
	else
	{
		g = cv::Mat::zeros(cv::Size(mask_conj.cols, mask_conj.rows), CV_32FC1);
		mask2 = cv::Mat::zeros(cv::Size(mask_conj.cols, mask_conj.rows), CV_32FC2);

		channels.push_back(mask_conj);
		channels.push_back(g);

		merge(channels, mask2);
	}

	//Matrices product
	cv::Mat M3 = multiplicationCC(fourierTransform, mask2);

	// IFFT
	//std::cout << "Inverse transform...\n";
	cv::dft(M3, cwtcfs, cv::DFT_INVERSE | cv::DFT_REAL_OUTPUT);

	// Back to 8-bits. angle values in Matlab code
	cwtcfs.convertTo(bincfs, CV_8U);
}

#if 0 // not used

// Returns the contour of a stone as a polyline 
static ccPolyline* contourPoly(const ccPointCloud& stone)
{
	std::vector<CCCoreLib::PointProjectionTools::IndexedCCVector2> points2D;
	points2D.reserve(stone.size());

	for (unsigned i = 0; i < stone.size(); ++i)
	{
		const CCVector3* _P = stone.getPoint(i);
		CCCoreLib::PointProjectionTools::IndexedCCVector2 P(_P->x, _P->z, i);
		points2D.push_back(P);
	}

	std::list<CCCoreLib::PointProjectionTools::IndexedCCVector2*> hullPoint2;
	CCCoreLib::PointProjectionTools::extractConcaveHull2D(points2D, hullPoint2, 0.0001);

	std::vector<unsigned> Vec;
	Vec.reserve(hullPoint2.size());
	for (auto it = hullPoint2.begin(); it != hullPoint2.end(); ++it)
	{
		Vec.push_back((*it)->index);
	}

	unsigned cnt = static_cast<unsigned>(Vec.size());

	ccPointCloud* contour = new ccPointCloud(stone.getName() + " - Contour Cloud");
	if (!contour->reserve(cnt))
	{
		// not enough memory
		delete contour;
		return nullptr;
	}
	bool hasColors = (stone.hasColors() && contour->reserveTheRGBTable());

	for (auto e : Vec)
	{
		contour->addPoint(*stone.getPoint(e));
		if (hasColors)
		{
			contour->addColor(stone.getPointColor(e));
		}

	}

	ccPolyline* pContour = new ccPolyline(contour);
	pContour->addChild(contour);

	if (pContour->reserve(cnt))
	{
		pContour->addPointIndex(0, cnt);
		pContour->setClosed(true);
		pContour->setVisible(true);
		pContour->setName(stone.getName() +" - Contour");
		pContour->setColor(ccColor::cyan);
		pContour->showColors(true);
		pContour->showVertices(true);
	}
	else
	{
		// not enough memory
		delete pContour;
		pContour = nullptr;
	}

	return pContour;
}

#endif

static ccPolyline* contourPoly2(const ccPointCloud& cloud0, const std::vector<int>& V, const QString& name)
{
	std::vector<CCCoreLib::PointProjectionTools::IndexedCCVector2> points2D;
	points2D.reserve(V.size());

	for (size_t i = 0; i < V.size(); ++i)
	{
		const CCVector3* _P = cloud0.getPoint(V[i]);
		CCCoreLib::PointProjectionTools::IndexedCCVector2 P(_P->x, _P->z, i);
		points2D.push_back(P);
	}

	std::list<CCCoreLib::PointProjectionTools::IndexedCCVector2*> hullPoint2;
	CCCoreLib::PointProjectionTools::extractConcaveHull2D(points2D, hullPoint2, 0.0001);

	std::vector<unsigned> Vec;
	Vec.reserve(hullPoint2.size());
	for (auto it = hullPoint2.begin(); it != hullPoint2.end(); ++it)
	{
		Vec.push_back((*it)->index);
	}

	unsigned cnt = static_cast<unsigned>(Vec.size());

	ccPointCloud* contour = new ccPointCloud(name + " - Contour Cloud");
	if (!contour->reserve(cnt))
	{
		// not enough memory
		delete contour;
		return nullptr;
	}
	bool hasColors = (cloud0.hasColors() && contour->reserveTheRGBTable());

	for (auto e : Vec)
	{
		contour->addPoint(*cloud0.getPoint(V[e]));
		if (hasColors)
		{
			contour->addColor(cloud0.getPointColor(V[e]));
		}
	}

	ccPolyline* pContour = new ccPolyline(contour);
	pContour->addChild(contour);

	if (pContour->reserve(cnt))
	{
		pContour->addPointIndex(0, cnt);
		pContour->setClosed(true);
		pContour->setVisible(true);
		pContour->setName(name + " - Contour");
		pContour->setColor(ccColor::cyan);
		pContour->showColors(true);
		pContour->showVertices(true);
	}
	else
	{
		// not enough memory
		delete pContour;
		pContour = nullptr;
	}

	return pContour;
}

// Returns mortar maps (Depth and width)
static ccPointCloud* getMortarMaps(ccPointCloud& f_cloudStones, ccPointCloud& f_cloudMortar)
{
	//Binary Mortar
	f_cloudMortar.scale(100, 1000, 100); //To cm mm cm
	CCVector3 minBox(0, 0, 0);
	CCVector3 maxBox(0, 0, 0);
	f_cloudMortar.getBoundingBox(minBox, maxBox);
	int rowsM = ceil(maxBox.z);
	int colsM = ceil(maxBox.x);

	cv::Mat corrMatM = cv::Mat::zeros(rowsM, colsM, CV_32F);
	cv::Mat imageBWS = cv::Mat::zeros(rowsM, colsM, CV_8U);

	std::vector<cv::Point> idxPxM;
	std::vector<int> idxPxM1D;
	cloud2binary(f_cloudMortar, corrMatM, idxPxM, idxPxM1D, imageBWS);

	//Skeleton
	cv::Mat skel = skeleton(imageBWS, false);

	f_cloudMortar.scale(0.01, 0.001, 0.01); //Scale back

	//Skeleton 3D
	std::vector<int> pixelsSkel;
	for (int i = 0; i < skel.rows; i++)
	{
		for (int j = 0; j < skel.cols; j++)
		{
			unsigned char a = skel.at<char>(i, j);
			if (a == 255)
			{
				int idx = corrMatM.at<int>(i, j);
				pixelsSkel.push_back(idx);
			}
		}
	}

	std::vector<int> idxCloudMortarSkel = setIntersectIdxPixs1D(idxPxM1D, pixelsSkel);
	ccPointCloud* skelM = new ccPointCloud("Skeleton Mortar");
	if (!skelM->reserve(static_cast<unsigned>(idxCloudMortarSkel.size())))
	{
		// not enough memory
		delete skelM;
		return nullptr;
	}
	bool hasColors = (f_cloudMortar.hasColors() && skelM->reserveTheRGBTable());

	for (auto e : idxCloudMortarSkel)
	{
		skelM->addPoint(*f_cloudMortar.getPoint(e));
		if (hasColors)
		{
			skelM->addColor(f_cloudMortar.getPointColor(e));
		}
	}

	CCCoreLib::CloudSamplingTools::SFModulationParams modParams(false); 	//Subsample result
	CCCoreLib::ReferenceCloud* refCloud = CCCoreLib::CloudSamplingTools::resampleCloudSpatially(skelM, 0.01, modParams, 0, 0); //1 point per cm^2
	if (!refCloud)
	{
		delete skelM;
		return nullptr;
	}

	ccPointCloud* f_skelMortar = skelM->partialClone(refCloud); 	//save output

	delete refCloud;
	refCloud = nullptr;
	delete skelM;
	skelM = nullptr;

	if (!f_skelMortar)
	{
		return nullptr;
	}

	// Maps using PCL
	//Mortar depth map
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloudStones(new pcl::PointCloud<pcl::PointXYZ>);
	pcl_cloudStones->width = f_cloudStones.size();
	pcl_cloudStones->height = 1;
	pcl_cloudStones->points.resize(pcl_cloudStones->width * pcl_cloudStones->height);

	for (size_t i = 0; i < f_cloudStones.size(); ++i)
	{
		const CCVector3* P = f_cloudStones.getPoint(i);
		pcl_cloudStones->points[i].x = P->x;
		pcl_cloudStones->points[i].y = P->y;
		pcl_cloudStones->points[i].z = P->z;
	}

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(pcl_cloudStones);

	std::vector<double> distStD, dist3St, distStD2;
	std::vector<double> distStW;
	for (unsigned i = 0; i < f_skelMortar->size(); i++)
	{
		pcl::PointXYZ searchPoint;
		searchPoint.x = f_skelMortar->getPoint(i)->x;
		searchPoint.y = f_skelMortar->getPoint(i)->y;
		searchPoint.z = f_skelMortar->getPoint(i)->z;

		std::vector<int> pointIdxRadiusSearch;
		std::vector<float> pointRadiusSquaredDistance;

		float radius = 0.1f;	//10cm // TODO FIXME:the search radius is hardcoded?!
		pcl::PointXYZ nnPoint;

		if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
		{
			nnPoint.x = pcl_cloudStones->points[pointIdxRadiusSearch[0]].x;
			nnPoint.y = pcl_cloudStones->points[pointIdxRadiusSearch[0]].y;
			nnPoint.z = pcl_cloudStones->points[pointIdxRadiusSearch[0]].z;

			double disty = std::abs(searchPoint.y - nnPoint.y); //Depth
			double distx = std::abs(searchPoint.x - nnPoint.x);
			double distz = std::abs(searchPoint.z - nnPoint.z);
			double dist3 = std::sqrt(distx*distx + distz * distz);//2D distance (XZ) will be used as a reference for the creation of mortar width map		

			distStD.push_back(disty * 1000); //depth (y axis) in mm
			dist3St.push_back(dist3);

			double dist1 = 1000.0; double dist2 = 1000.0; double dist1y = 1000.0; double dist2y = 1000.0;
			int idx1 = -1; int idx2 = -1;
			for (size_t j = 0; j < pointIdxRadiusSearch.size(); ++j)
			{
				double distx = std::abs(searchPoint.x - pcl_cloudStones->points[pointIdxRadiusSearch[j]].x); //2D distance
				double distz = std::abs(searchPoint.z - pcl_cloudStones->points[pointIdxRadiusSearch[j]].z);
				double disty = std::abs(searchPoint.y - pcl_cloudStones->points[pointIdxRadiusSearch[j]].y);
				double dist3 = std::sqrt(abs(distx)*abs(distx) + abs(distz)*abs(distz));

				int idxSt = f_cloudStones.getPointScalarValue(pointIdxRadiusSearch[j]);

				if (j == 0)
				{
					//Initialise NN stone 1
					dist1 = dist3;
					dist1y = disty;
					idx1 = idxSt;
				}
				else
				{
					if (idxSt == idx1)
					{
						//If point belongs to stone 1
						if (dist3 < dist1)
							dist1 = dist3;
						dist1y = disty;
					}
					else if (idx2 == -1)
					{
						//Initialise stone 2
						idx2 = idxSt;
						dist2 = dist3;
						dist2y = disty;
					}
					if (idxSt == idx2)
					{
						//If point belongs to stone 2
						if (dist3 < dist2)
							dist2 = dist3;
						dist2y = disty;
					}
				}
			} //Points are ordered by radius. Idx 0 is the nearest neighbour

			//For each point, there is dist1 or dist1 and dist2
			if (idx1 > -1 && idx2 == -1 && dist1 < 1000)
			{
				//Only one stone (boundaries of wall)
				distStW.push_back(dist1 * 1000);
				distStD2.push_back(dist1y * 1000);
			}
			else if (idx1 > -1 && idx2 > -1)
			{
				distStW.push_back((dist1 + dist2) * 1000);
				distStD2.push_back((dist1y + dist2y) / 2 * 1000);
			}
		}
		else
		{
			distStD.push_back(100); //Add high values for non-calculated values (no neighbours available). These values can be modified according to the needs
			distStD2.push_back(100);
			dist3St.push_back(200);
			distStW.push_back(200);
		}
	}

	//Add depth as a scalarField to f_skelMortar
	int sfIdxD = f_skelMortar->getScalarFieldIndexByName("Mortar relative depth (mm)");
	if (sfIdxD < 0)
	{
		sfIdxD = f_skelMortar->addScalarField("Mortar relative depth (mm)");
	}
	if (sfIdxD >= 0)
	{
		CCCoreLib::ScalarField* depthSF = f_skelMortar->getScalarField(sfIdxD);

		for (unsigned i = 0; i < static_cast<unsigned>(distStD2.size()); i++)
		{
			ScalarType index = static_cast<ScalarType>(distStD2[i]);
			depthSF->setValue(i, index);
		}
		depthSF->computeMinAndMax();

		f_skelMortar->setCurrentDisplayedScalarField(sfIdxD);
		f_skelMortar->showSF(true);
	}
	else
	{
		// not enough memory?
	}

	//Add width as a scalarField to f_skelMortar
	int sfIdxW = f_skelMortar->getScalarFieldIndexByName("Mortar relative width (mm)");
	if (sfIdxW < 0)
	{
		sfIdxW = f_skelMortar->addScalarField("Mortar relative width (mm)");
	}
	if (sfIdxW >= 0)
	{
		CCCoreLib::ScalarField* widthSF = f_skelMortar->getScalarField(sfIdxW);

		for (unsigned i = 0; i < static_cast<unsigned>(distStW.size()); i++)
		{
			ScalarType index = static_cast<ScalarType>(distStW[i]);
			widthSF->setValue(i, index);
		}
		widthSF->computeMinAndMax();

		f_skelMortar->setCurrentDisplayedScalarField(sfIdxW);
		f_skelMortar->showSF(true);
	}
	else
	{
		// not enough memory?
	}

	f_skelMortar->setName("Mortar Maps");
	f_skelMortar->setPointSize(7);

	return f_skelMortar;
}

#if 0 // not used
static float getDensity(const ccPointCloud& cloud, const std::vector<int>& idx)
{
	ccPointCloud* stone = new ccPointCloud("stone");
	if (!stone->reserve(static_cast<unsigned>(idx.size())))
	{
		// not enough memory
		return 0.0;
	}

	for (auto e : idx)
	{
		stone->addPoint(*cloud.getPoint(e));
	}

	CCVector3 minBox(0, 0, 0);
	CCVector3 maxBox(0, 0, 0);
	stone->getBoundingBox(minBox, maxBox);

	float area = (maxBox.x - minBox.x)*(maxBox.z - minBox.z);
	float density = (area > 0 ? idx.size() / area : 0.0f);
	return density;
}
#endif

// This method should enable or disable your plugin actions
// depending on the currently selected entities ('selectedEntities').
void ccAutoSeg::onNewSelection(const ccHObject::Container& selectedEntities)
{
	if (m_action)
	{
		m_action->setEnabled(selectedEntities.size() == 1 && selectedEntities.front()->isA(CC_TYPES::POINT_CLOUD));
	}
}

// This method returns all the 'actions' your plugin can perform.
// getActions() will be called only once, when plugin is loaded.
QList<QAction*> ccAutoSeg::getActions()
{
	if ( !m_action )
	{
		m_action = new QAction( getName(), this );
		m_action->setToolTip( getDescription() );
		m_action->setIcon(QIcon(QString::fromUtf8(":/CC/plugin/qAutoSeg/cyberbuildIcon.png")));
		
		// Connect appropriate signal
		connect( m_action, &QAction::triggered, this, &ccAutoSeg::doAction );
	}

	return { m_action };
}

void ccAutoSeg::doAction()
{
	if (m_app == nullptr)
	{
		// m_app should have already been initialized by CC when plugin is loaded
		Q_ASSERT(false);

		return;
	}

	//Select point cloud (manually, from CC UI)
	ccPointCloud* cloud0 = nullptr;
	{
		const ccHObject::Container& selectedEntities = m_app->getSelectedEntities();

		if (!m_app->haveOneSelection() || !selectedEntities.front()->isA(CC_TYPES::POINT_CLOUD))
		{
			m_app->dispToConsole("Select one cloud!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return;
		}

		cloud0 = static_cast<ccPointCloud*>(selectedEntities.front());
	}

	//Create pop-up window
	ProfileImportDlg piDlg(m_app->getMainWindow());
	if (!piDlg.exec())
	{
		return;
	}

	double joints = piDlg.jointsSpinBox->value(); //Mortar joint for CWT
	double segH = piDlg.segmentHSpinBox->value(); //Segmentation window
	double segV = piDlg.segmentVSpinBox->value();
	
	bool checkAlignment = piDlg.alignmentCheckBox->isChecked();
	bool checkSegment = piDlg.segmentCheckBox->isChecked();
	bool checkMortar = piDlg.mortarCheckBox->isChecked();

	//If mortar is selected but not segmentation, show an error message and ask for checking both
	if (checkMortar == true && checkSegment == false)
	{
		m_app->dispToConsole("Segmentation is required for the creation of mortar maps. Please, check both \"Automatic segmentation\" and \"Mortar maps\".", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	//Create log file
	std::ofstream auto_seg_log;
	{
		std::string filename("APS_log_");
		filename += QDateTime::currentDateTime().toString("yyyy_MM_dd_hh_mm_ss").toStdString();
		filename += ".txt";
		auto_seg_log.open(filename);

		auto_seg_log << dateStamp() << " Importing point cloud" << endl;

		std::string parentName = (cloud0->getParent() != nullptr ? cloud0->getParent()->getName().toStdString() : "");

		std::string stamp = dateStamp();
		auto_seg_log << stamp << " File \"" << parentName << "\" loaded" << endl;
		auto_seg_log << stamp << " Cloud \"" << cloud0->getName().toStdString() << "\" imported" << endl;

		//Add input parameters and tasks to be done to the log
		stamp = dateStamp();
		auto_seg_log << stamp << " Estimated mortar joints width: " << joints << "cm" << endl;
		auto_seg_log << stamp << " Segmentation window. X: " << segH << "m and Y: " << segV << "m" << endl;
		auto_seg_log << stamp << " Alignment: " << checkAlignment << endl;
		auto_seg_log << stamp << " Segmentation: " << checkSegment << endl;
		auto_seg_log << stamp << " Mortar maps: " << checkMortar << endl;
		auto_seg_log << " Starting process..." << endl;
	}

	try
	{
		//Check plane orientation
		cv::Mat_<double> cldm(cloud0->size(), 3);
		for (unsigned i = 0; i < cloud0->size(); i++)
		{
			const CCVector3* P = cloud0->getPoint(i);
			cldm.row(i)(0) = P->x;
			cldm.row(i)(1) = P->y;
			cldm.row(i)(2) = P->z;
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
		std::vector<int> v2(cloud0->size());
		std::iota(v2.begin(), v2.end(), 0);
		std::random_shuffle(v2.begin(), v2.end());

		size_t nbSub = std::min(v2.size(), static_cast<size_t>(1000000));

		ccPointCloud* cloud2Rot = new ccPointCloud("cloud2Rot"); //Kike
		if (!cloud2Rot->reserve(static_cast<unsigned>(nbSub)) || !cloud2Rot->reserveTheRGBTable())
		{
			// not enough memory
			ccLog::Error("Not enough memory");
			delete cloud2Rot;
			return;
		}

		bool hasColors = cloud0->hasColors();
		if (hasColors && !cloud2Rot->reserveTheRGBTable())
		{
			// not enough memory to process colors
			hasColors = false;
		}

		for (size_t i = 0; i < nbSub; i++)
		{
			cloud2Rot->addPoint(*cloud0->getPoint(v2[i]));
			if (hasColors)
			{
				cloud2Rot->addColor(cloud0->getPointColor(v2[i]));
			}
		}

		double th0 = optRotY(*cloud2Rot);

		CCVector3 c1, c2, c3, c4;
		c1[0] = cos(th0);		c1[1] = 0; c1[2] = -sin(th0);	c1[3] = 0;
		c2[0] = 0;				c2[1] = 1; c2[2] = 0;			c2[3] = 0;
		c3[0] = sin(th0);		c3[1] = 0; c3[2] = cos(th0);	c3[3] = 0;
		c4[0] = 0;				c4[1] = 0; c4[2] = 0;			c4[3] = 1;
		ccGLMatrix R3(c1, c2, c3, c4);

		cloud0->applyRigidTransformation(R3);

		//If only alignment is required
		if (checkAlignment == true && checkSegment == false && checkMortar == false)
		{
			ccGLWindowInterface* win = m_app->getActiveGLWindow();
			win->setView(CC_FRONT_VIEW);
			return;
		}

		auto_seg_log << dateStamp() << "Cloud aligned to XZ." << endl;

		//Bounding box
		CCVector3 minBox(0, 0, 0);
		CCVector3 maxBox(0, 0, 0);
		cloud0->getBoundingBox(minBox, maxBox);

		//Move point cloud to origin
		cloud0->translate(-minBox);

		double axX = maxBox.x - minBox.x;
		double axY = maxBox.z - minBox.z;

		// targeted window dimensions
		float winX = segH;
		float winY = segV;

		// Number of windows
		unsigned int nwinX = std::ceil(axX / winX);
		unsigned int nwinY = std::ceil(axY / winY);

		//real window dimensions without overlaping
		double rwinX = axX / nwinX;
		double rwinY = axY / nwinY;

		//boundaries of windows [(xmin1,xmax1,ymin1,ymax1),(xmin2,xmax2....)...]
		std::vector<std::vector<double>> boundaries;

		double X = 0;
		std::vector<int> global_idx;

		for (unsigned int i = 0; i < nwinX; i++)
		{
			double Y = 0;
			for (unsigned int j = 0; j < nwinY; j++)
			{
				boundaries.push_back({ X, X + rwinX + 0.3, Y, Y + rwinY + 0.3 });
				Y += rwinY;
			}

			X += rwinX;
		}

		//spliting points by index
		std::vector<std::vector<int>> idxClouds(boundaries.size());

		auto_seg_log << dateStamp() << " Splitting cloud in patches (" << nwinX * nwinY << ")" << endl;

		for (unsigned i = 0; i < cloud0->size(); i++)
		{
			for (size_t j = 0; j < boundaries.size(); j++)
			{
				const CCVector3* p = cloud0->getPoint(i);
				const auto& boundary = boundaries[j];
				if (p->x >= boundary[0] && p->x <= boundary[1] && p->z >= boundary[2] && p->z <= boundary[3])
				{
					idxClouds[j].push_back(i);
				}

			}
			global_idx.push_back(i);
		}

		// removing empty ones
		{
			std::vector<std::vector<int>> idxCloudsC;
			for (auto& e : idxClouds)
				if (e.size() != 0)
					idxCloudsC.push_back(std::move(e));

			idxClouds.swap(idxCloudsC);
		}

		auto_seg_log << dateStamp() << " Splitting done" << endl;

		// adding points to the corresponding window
		auto_seg_log << dateStamp() << " Creating subclouds" << endl;
		std::vector<ccPointCloud*> sub_clouds;
		for (size_t i = 0; i < idxClouds.size(); i++)
		{
			QString str = "part_" + QString::number(i);
			if (idxClouds[i].size() != 0)
			{
				ccPointCloud* win_cloud = new ccPointCloud(str);
				if (!win_cloud->reserve(idxClouds[i].size()))
				{
					// not enough memory
					ccLog::Warning(QString("Failed to create sub cloud #%1 (not enough memory)").arg(i + 1));
					delete win_cloud;
					continue;
				}
				bool withColors = (cloud0->hasColors() && win_cloud->reserveTheRGBTable());

				for (auto e : idxClouds[i])
				{
					win_cloud->addPoint(*cloud0->getPoint(e));
					if (withColors)
					{
						win_cloud->addColor(cloud0->getPointColor(e));
					}
				}

				sub_clouds.push_back(win_cloud);
			}
		}

		// Going through the sub_clouds vector
		std::vector<std::vector<std::pair<int, int>>> stones;
		int stoneIdx = 0;

		for (size_t z = 0; z < sub_clouds.size(); ++z)
		{
			ccPointCloud* subCloud = sub_clouds[z];

			auto_seg_log << dateStamp() << " Cloud #" << z+1 << ": " << subCloud->size() << " pts" << endl;

			if (subCloud->size() == 0)
			{
				assert(false);
				continue;
			}

			m_app->dispToConsole(QString("part_%1 has %2 points").arg(z).arg(subCloud->size()), ccMainAppInterface::STD_CONSOLE_MESSAGE);

			//Bounding box
			CCVector3 minBox_0(0, 0, 0);
			CCVector3 maxBox_0(0, 0, 0);
			subCloud->getBoundingBox(minBox_0, maxBox_0);

			//Move point cloud to origin
			subCloud->translate(-minBox_0);

			//Create and populate maps

			//Bounding box
			CCVector3 minBox2(0, 0, 0);
			CCVector3 maxBox2(0, 0, 0);
			subCloud->getBoundingBox(minBox2, maxBox2);

			subCloud->scale(100, 1000, 100); //To cm mm cm

			int rows = std::ceil(maxBox2.z);
			int cols = std::ceil(maxBox2.x);

			//Correspondence 2D->1D. Each px has an index instead of two coords
			cv::Mat corrMat = cv::Mat::zeros(rows, cols, CV_32F);
			int cntC = 0;
			for (int y = 0; y < corrMat.rows; y++)
			{
				for (int x = 0; x < corrMat.cols; x++)
				{
					corrMat.at<int>(y, x) = cntC++;
				}
			}

			//Depth
			cv::Mat image = cv::Mat::zeros(rows, cols, CV_32F);

			std::vector<cv::Point> idxPx;
			std::vector<int> idxPx1D;
			for (unsigned i = 0; i < subCloud->size(); i++)
			{
				//depth
				const CCVector3* pointC = subCloud->getPoint(i);
				int z = std::floor(pointC->z);
				int x = std::floor(pointC->x);
				image.at<float>(z, x) = static_cast<float>(pointC->y);

				//idxPx per 3D point
				idxPx.push_back(cv::Point(z, x));
				idxPx1D.push_back(corrMat.at<int>(z, x));
			}

			//Median filter to fill holes
			cv::Mat imageFilt = cv::Mat::zeros(rows, cols, CV_32F);
			cv::medianBlur(image, imageFilt, 3);

			//Check for black areas after filling holes. These are structural holes (windows, doors)
			std::vector<int> nullCols, nullRows;

			for (int y = 0; y < image.rows; y++)
			{
				for (int x = 0; x < image.cols; x++)
				{
					if (image.at<float>(y, x) == 0)
					{
						image.at<float>(y, x) = imageFilt.at<float>(y, x);

						if (image.at<float>(y, x) == 0)
						{
							nullCols.push_back(x);
							nullRows.push_back(y);
						}
					}
				}
			}

			imageFilt.release();

			//cv::Mat imagePlot;
			//image.convertTo(imagePlot, CV_8U);
			//imagePlot.release();

			subCloud->scale(0.01, 0.001, 0.01); //Scale back
			subCloud->translate(minBox_0); //Translate back

			auto_seg_log << dateStamp() << " Depth map created" << endl;

			//CWT
			double scaleCWT = joints * 0.254;//1.27 for 5cm
			cv::Mat cwtcfs;
			cv::Mat bincfs = cv::Mat::zeros(cwtcfs.size(), CV_8UC1);
			cwt2d(scaleCWT, image, cwtcfs, bincfs);
			cwtcfs.release();

			//Cropping borders added in cwt

			cv::Rect myROI(0, 0, image.cols, image.rows);
			bincfs = bincfs(myROI);
			image.release();

			auto_seg_log << dateStamp() << " CWT calculated" << endl;

			//Clean areas from holes
			for (size_t i = 0; i < nullRows.size(); i++)
			{
				bincfs.at<uchar>(nullRows[i], nullCols[i]) = 0;
			}

			//Fill in the holes
			cv::Mat hola;
			fillEdgeImage(bincfs, hola, z);
			bincfs.release();

			//Remove small segments
			cv::Mat output = threshSegments(hola, 20);
			hola.release();

			//Erosion + Dilation
			int erosion_elem = 2; //0 = morph_rect, 1 = morph_cross, 2 = morph_ellipse (I think Matlab uses 0)
			int erosion_size = 1;
			cv::Mat erodedM;
			erosionCC(erosion_elem, erosion_size, output, erodedM);
			output.release();


			///This has been added here (not in matlab)
			int dilation_elem = 2; //0 = morph_rect, 1 = morph_cross, 2 = morph_ellipse
			int dilation_size = 1;
			cv::Mat dilatedM;
			dilationCC(dilation_elem, dilation_size, erodedM, dilatedM);
			erodedM.release();

			auto_seg_log << dateStamp() << " CWT post-processing done" << endl;

			//EXTRACT INFO FROM BINARY SEGMENTATION
			cv::Mat labels, stats;
			std::vector<std::vector<cv::Point>> pixsLabel = extractInfo(dilatedM, labels, stats);
			dilatedM.release();

			if (stats.rows < 2)
			{
				assert(false);
				continue;
			}

			//OPTIMISE CWT
			//areas
			std::vector<double> area(stats.rows - 1, 0); //nLabels-1 as i = 0 corresponds to background
			for (int i = 1; i < stats.rows; i++)
			{
				area.at(i - 1) = stats.at<int>(i, cv::CC_STAT_AREA);
			}
			//mean
			double meanArea = std::accumulate(area.begin(), area.end(), 0.0) / (stats.rows - 1);
			stats.release();

			//std
			double stdevArea = 0.0;
			{
				std::vector<double> diff(area.size());
				std::transform(area.begin(), area.end(), diff.begin(), [meanArea](double x) { return x - meanArea; });
				double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
				stdevArea = std::sqrt(sq_sum / area.size());
				diff.clear();
			}

			int op = 3; //op = 1: lt; op = 2: leq; op = 3: gt; op = 4: geq;
			double thres = meanArea + stdevArea; //meanArea + 5*stdevArea
			std::vector<int> bigStones = findMatlab(area, op, thres);
			area.clear();

			//Divide big stones. Erode big > new segments. New matrix (from dilateM) without big ones but adding new segments 
			cv::Mat firstSegmentation = stonesDivision(pixsLabel, labels, bigStones);
			bigStones.clear();
			pixsLabel.clear();
			labels.release();

			std::vector<std::vector<cv::Point>> contours2;
			{
				cv::Mat labels2, stats2;
				std::vector<std::vector<cv::Point>> pixsLabel2 = extractInfo(firstSegmentation, labels2, stats2, &contours2); //Extract info after big stone division
			}

			auto_seg_log << dateStamp() << " Big stones divided" << endl;

			//cv::Mat skel = skeleton(firstSegmentation, 1);
			//skel.release();

			// Find the convex hull object for each contour
			std::vector<std::vector<cv::Point>> hull(contours2.size());
			for (size_t i = 0; i < contours2.size(); i++)
			{
				convexHull(cv::Mat(contours2[i]), hull[i], false);
			}
			contours2.clear();

			// Draw contours + hull results
			cv::RNG rng(12345);
			std::vector<std::vector<cv::Point>> pixsHullSt(hull.size());
			cv::Mat drawingAll = cv::Mat::zeros(firstSegmentation.size(), CV_8UC3);

			// Segments growth
			std::vector<int> pixsLabels(firstSegmentation.cols*firstSegmentation.rows, -1); //Store idxs in a vector instead of vector of vectors //FK2310
			std::vector<int> pixsLabels2(firstSegmentation.cols*firstSegmentation.rows, -1); //Store idxs in a vector instead of vector of vectors. Consider overlapping //FK2310
			for (size_t i = 0; i < hull.size(); i++) //
			{
				cv::Mat drawing = cv::Mat::zeros(firstSegmentation.size(), CV_8UC1);

				cv::Scalar color(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
				cv::Scalar color2(255, 255, 255);
				drawContours(drawing, hull, i, color2, -1, 8, {}, 0, {}); //5th input param to -1 to colour segments
				drawContours(drawingAll, hull, i, color, -1, 8, {}, 0, {}); //To plot

				cv::Mat labelsHull, statsHull;
				std::vector<std::vector<cv::Point>> pixsHull = extractInfo(drawing, labelsHull, statsHull); //Extract info from individual convex hulls

				pixsHullSt[i] = pixsHull[0];

				for (size_t j = 0; j < pixsHull[0].size(); j++)
				{
					int idx = corrMat.at<int>(pixsHull[0][j].x, pixsHull[0][j].y);
					if (pixsLabels.at(idx) == -1)
					{
						// Stones inside each other not considered as duplicated
						pixsLabels.at(idx) = i;
					}
					else
					{
						// Additional vector to consider pixels labelled as part of two stones (due to convex hull operation, for example)
						pixsLabels2.at(idx) = i;
					}
				}
			}
			firstSegmentation.release();
			hull.clear();
			drawingAll.release();

			auto_seg_log << dateStamp() << " Convex hulls defined" << endl;

			// EXTRACT 3D POINTS PER SEGMENT
			for (size_t i = 0; i < pixsHullSt.size(); i++)
			{
				std::vector<cv::Point>& pixels = pixsHullSt[i];

				////////// New 1D
				std::vector<int> pixels1d;
				for (const auto& e : pixels)
				{
					int idx = corrMat.at<int>(e.x, e.y);
					pixels1d.push_back(idx);
				}

				std::vector<int> idxCloud;
				for (size_t j = 0; j < idxPx.size(); j++)
				{
					if (pixsLabels[idxPx1D[j]] == i)
						idxCloud.push_back(j);
					else if (pixsLabels2[idxPx1D[j]] == i) //Check potential second label (this will be cleaned afterwards)
						idxCloud.push_back(j);
				}

				std::vector<std::pair<int, int>> global_SglStoneIdx;
				unsigned max_size = subCloud->size() / 1; // maximum size per single stone 
				//float density = getDensity(*subCloud, idxCloud);
				//float max_density = 40000.0f;
				if (idxCloud.size() < max_size / 10)
				{
					for (auto e : idxCloud)
					{
						if (e < idxClouds[z].size())
						{
							global_SglStoneIdx.push_back(std::make_pair(idxClouds[z][e], stoneIdx));
						}
						else
						{
							auto_seg_log << "====> Error :" << z << "	" << e << endl;
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

			auto_seg_log << dateStamp() << " 3D points per segment extracted" << endl;

			/*** HERE ENDS THE ACTION ***/

			//Deleting releasing memory ?
			auto_seg_log << dateStamp() << " Point cloud processed. Releasing memory ... " << endl;

			corrMat.release();
			pixsHullSt.clear();

			subCloud->removeMetaData("cloud");
			delete subCloud;
			sub_clouds[z] = subCloud = nullptr;
		}

		auto_seg_log << dateStamp() << " Stitching stones" << endl;

		//concatenating all stone's cloudidx and stoneidx pairs into one vector instead of a vector of vectors with separate stone in each vector
		std::vector<std::pair<int, int>> allStones;
		if (stones.size() != 0)
		{
			allStones = stones[0];
		}

		for (size_t i = 1; i < stones.size(); i++)
		{
			allStones.insert(allStones.end(), stones[i].begin(), stones[i].end());
		}
		auto_seg_log << dateStamp() << " Total number of unstitched stones: " << stones.size() << endl;

		//Start of stiching

		//indentifying and merging indentical stones after the stitching
		//This can be defined as a function

		// sorting a vector of pairs 
		std::sort(allStones.begin(), allStones.end(), [](auto &left, auto &right) {return left.first < right.first; });
		auto_seg_log << dateStamp() << " Start looking for whole stones " << endl;

		size_t i = 0;
		while (i < allStones.size())
		{
			size_t j = i + 1;

			while (j < allStones.size() && allStones[i].first == allStones[j].first && allStones[i].second != allStones[j].second)
			{
				int oldIdx = allStones[j].second;
				int newIdx = allStones[i].second;

				for (size_t k = 0; k < allStones.size(); k++)
				{
					if (allStones[k].second == oldIdx)
					{
						allStones[k].second = newIdx;
					}
				}

				j++;
			}
			i = j;
		}

		//End of stitching
		auto_seg_log << dateStamp() << " Indices corrected " << endl;

		//eliminating repeated pairs ( originaly common points in stones)
		allStones.erase(unique(allStones.begin(), allStones.end()), allStones.end());
		auto_seg_log << dateStamp() << " Repeated pairs removed " << endl;

		std::vector<int> allStones_Idx; // storing only cloud index of points that's within stones

		auto_seg_log << dateStamp() << " Getting all stones idx " << endl;
		allStones_Idx.reserve(allStones.size());
		for (const auto& e : allStones)
		{
			allStones_Idx.push_back(e.first);
		}
		std::sort(allStones_Idx.begin(), allStones_Idx.end());

		// Mortar idx
		std::vector<int> allMortar;
		std::set_difference(global_idx.begin(), global_idx.end(), allStones_Idx.begin(), allStones_Idx.end(), std::inserter(allMortar, allMortar.end()));

		auto_seg_log << dateStamp() << " Mortar idx identified " << endl;

		// correcting stone indices (make them 1,2,3 .... in order)

		std::vector<int> stoneId; // saving all the existing and used indices for stones 
		stoneId.reserve(allStones.size());
		for (const auto& e : allStones)
		{
			stoneId.push_back(e.second);
		}

		std::sort(stoneId.begin(), stoneId.end());

		stoneId.erase(unique(stoneId.begin(), stoneId.end()), stoneId.end());

		std::vector<std::vector<int>> vecStones(stoneId.size());
		std::sort(allStones.begin(), allStones.end(), [](auto &left, auto &right) {return left.second < right.second; });

		int old_idx = allStones[0].second;
		int new_idx = 0;

		for (size_t i = 0; i < allStones.size(); i++)
		{
			if (allStones[i].second == old_idx)
			{
				allStones[i].second = new_idx;
				vecStones[new_idx].push_back(allStones[i].first);
			}
			else
			{
				++new_idx;
				old_idx = allStones[i].second;
				allStones[i].second = new_idx;
				vecStones[new_idx].push_back(allStones[i].first);
			}
		}

		stoneId.clear();
		allStones_Idx.clear(); // values were sorted

		for (const auto& e : allStones)
		{
			allStones_Idx.push_back(e.first);
			stoneId.push_back(e.second);
		}

		// suffling stone IDs for better visuals
		std::vector<int> stIdShuffle = stoneId;
		std::sort(stIdShuffle.begin(), stIdShuffle.end());
		stIdShuffle.erase(unique(stIdShuffle.begin(), stIdShuffle.end()), stIdShuffle.end());

		auto rng = std::default_random_engine{};
		std::shuffle(stIdShuffle.begin(), stIdShuffle.end(), rng);

		// generating final clouds
		auto_seg_log << dateStamp() << " Writing final clouds" << endl;

		//stones
		ccPointCloud* f_cloudStones = nullptr;
		if (allStones_Idx.size() != 0)
		{
			f_cloudStones = new ccPointCloud("f_cloudStones");
			if (f_cloudStones->reserve(static_cast<unsigned>(allStones_Idx.size())) && f_cloudStones->reserveTheRGBTable())
			{
				for (auto e : allStones_Idx)
				{
					f_cloudStones->addPoint(*cloud0->getPoint(e));
					f_cloudStones->addColor(cloud0->getPointColor(e));
				}

				auto_seg_log << dateStamp() << " Stone cloud OK. " << allStones_Idx.size() << " pts. " << vecStones.size() << " stones." << endl;

				// adding scalar field of stoneId to stone point cloud
				int sfIdx = f_cloudStones->getScalarFieldIndexByName("Stone Index");
				if (sfIdx < 0)
				{
					sfIdx = f_cloudStones->addScalarField("Stone Index");
				}
				if (sfIdx >= 0)
				{
					CCCoreLib::ScalarField* stIdSF = f_cloudStones->getScalarField(sfIdx);
					for (unsigned int i = 0; i < stoneId.size(); i++)
					{
						int index = static_cast<int>(stIdShuffle[stoneId[i]]);
						stIdSF->setValue(i, index);
					}
					stIdSF->computeMinAndMax();
					f_cloudStones->setCurrentDisplayedScalarField(sfIdx);
					auto_seg_log << dateStamp() << " Stone cloud SF OK " << endl;
				}
				else
				{
					// not enough memory
					ccLog::Warning("Not enough memory");
				}
			}
			else
			{
				// not enough memory
				delete f_cloudStones;
				f_cloudStones = nullptr;
				ccLog::Warning("Not enough memory");
			}
		}

		// Mortar
		ccPointCloud* f_cloudMortar = nullptr;
		if (allMortar.size() != 0)
		{
			f_cloudMortar = new ccPointCloud("f_cloudMortar");
			if (f_cloudMortar->reserve(static_cast<unsigned>(allMortar.size())) && f_cloudMortar->reserveTheRGBTable())
			{
				for (auto e : allMortar)
				{
					f_cloudMortar->addPoint(*cloud0->getPoint(e));
					f_cloudMortar->addColor(cloud0->getPointColor(e));
				}
				auto_seg_log << dateStamp() << " Mortar OK. " << allMortar.size() << " pts" << endl;
			}
			else
			{
				// not enough memory
				delete f_cloudMortar;
				f_cloudMortar = nullptr;
				ccLog::Warning("Not enough memory");
			}
		}

		//ADDING CLOUDS TO DB
		ccPointCloud* mortarMaps = nullptr;
		if (checkMortar == true && f_cloudStones && f_cloudMortar)
		{
			if (allMortar.size() > 0)
			{
				auto_seg_log << dateStamp() << " Starting mortar maps generation... " << endl;
				mortarMaps = getMortarMaps(*f_cloudStones, *f_cloudMortar);
				if (mortarMaps)
				{
					auto_seg_log << dateStamp() << " Mortar maps OK " << endl;
					mortarMaps->showSF(true);
				}
			}
		}

		// Move all the clouds back
		cloud0->translate(minBox);
		cloud0->applyRigidTransformation(R3.inverse());
		cloud0->applyRigidTransformation(R2.inverse());

		if (f_cloudMortar)
		{
			f_cloudMortar->translate(minBox);
			f_cloudMortar->applyRigidTransformation(R3.inverse());
			f_cloudMortar->applyRigidTransformation(R2.inverse());
			f_cloudMortar->showColors(true);
			m_app->addToDB(f_cloudMortar, true, true, false, true);
		}

		if (f_cloudStones)
		{
			f_cloudStones->translate(minBox);
			f_cloudStones->applyRigidTransformation(R3.inverse());
			f_cloudStones->applyRigidTransformation(R2.inverse());
			f_cloudStones->showColors(true);
			m_app->addToDB(f_cloudStones, true, true, false, true);
		}

		if (mortarMaps)
		{
			mortarMaps->translate(minBox);
			mortarMaps->applyRigidTransformation(R3.inverse());
			mortarMaps->applyRigidTransformation(R2.inverse());
			m_app->addToDB(mortarMaps, true, true, false, true);
		}

		// contours in an alternative way using stone indexes and original cloud
		if (vecStones.size() != 0)
		{
			//Generating contour polylines
			auto_seg_log << dateStamp() << " Start of contour generation" << endl;

			std::vector<double> stonesSize;
			ccHObject* contoursContainer = nullptr;
			for (const auto& ve : vecStones)
			{
				QString qs_name = "stone " + QString::number(stonesSize.size());

				if (ve.size() != 0)
				{
					ccPolyline* contour = contourPoly2(*cloud0, std::move(ve), qs_name);
					if (contour)
					{
						if (!contoursContainer)
						{
							contoursContainer = new ccHObject("Stone contours");
						}
						contoursContainer->addChild(contour);
						//Get BB per stone from contour.
						CCVector3 bbMinS(0, 0, 0);
						CCVector3 bbMaxS(0, 0, 0);
						contour->getBoundingBox(bbMinS, bbMaxS);
						double sizeS = (bbMaxS.x - bbMinS.x)*(bbMaxS.z - bbMinS.z);
						stonesSize.push_back(sizeS);
					}
				}
			}

			if (contoursContainer)
			{
				m_app->addToDB(contoursContainer, true, true, false, true);
			}

			//Mean and median size of stones
			if (stonesSize.size() != 0)
			{
				double meanSizeStones = accumulate(stonesSize.begin(), stonesSize.end(), 0.0) / stonesSize.size();
				double standDev = stddev(stonesSize);

				auto_seg_log << dateStamp() << " Average size of stones (BB) " << meanSizeStones << " m2 +-" << standDev << " (" << meanSizeStones * 10000 << " cm2)" << endl;

				std::sort(stonesSize.begin(), stonesSize.end());
				bool evenAmount = (stonesSize.size() % 2 == 0);
				int oddAmountIdx = ceil(stonesSize.size() / 2);
				double evenAmountMedian = (stonesSize[oddAmountIdx - 1] + stonesSize[oddAmountIdx]) / 2;
				double medianSizeStones = evenAmount ? evenAmountMedian : stonesSize[oddAmountIdx];

				auto_seg_log << dateStamp() << " Median size of stones (BB) " << medianSizeStones << " m2 (" << medianSizeStones * 10000 << " cm2)" << endl;
			}

			auto_seg_log << dateStamp() << " Contour generation done" << endl;
		}

		ccGLWindowInterface* win = m_app->getActiveGLWindow();
		if (win)
		{
			win->setView(CC_FRONT_VIEW);
		}
	}
	catch (const std::bad_alloc)
	{
		ccLog::Error("Not enough memory");
	}
}
