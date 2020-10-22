//##########################################################################
//#                                                                        #
//#                CLOUDCOMPARE PLUGIN: qManualSeg                         #
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


#include "qManualSeg.h"

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

//Local
#include "profileImportDlg.h"

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
#include <ccGenericPointCloud.h>

//CCCoreLib
#include <CloudSamplingTools.h>
#include <ManualSegmentationTools.h>
#include <SquareMatrix.h>
#include <CCConst.h>



#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <fstream>
#include <random>


#include<algorithm>
#include <math.h>  


//#include <kdtree.h>
using namespace std;
using namespace cv;


ccManualSeg::ccManualSeg( QObject *parent )
	: QObject( parent )
	, ccStdPluginInterface( ":/CC/plugin/qManualSeg/info.json" )
	, m_action(0)
	, m_segmentationPoly(0)
	, m_polyVertices(0)
{
	m_polyVertices = new ccPointCloud("vertices");
	m_segmentationPoly = new ccPolyline(m_polyVertices);
	m_segmentationPoly->setForeground(true);
	m_segmentationPoly->setColor(ccColor::green);
	m_segmentationPoly->showColors(true);
	m_segmentationPoly->set2DMode(true);

}

void ccManualSeg::onNewSelection( const ccHObject::Container &selectedEntities )
{
	if (m_action)
	{
	}

}

QList<QAction *> ccManualSeg::getActions()
{


	if ( !m_action )
	{
		m_action = new QAction( getName(), this );
		m_action->setToolTip( getDescription() );
		m_action->setIcon(QIcon(QString::fromUtf8(":/CC/plugin/qManualSeg/cyberbuildIcon.png")));
		
		// Connect appropriate signal
		connect( m_action, &QAction::triggered, this, &ccManualSeg::doAction );
	}

	return QList<QAction *>{
		m_action,
	};
}


// Returns indexes of points inside a polyline from a point cloud
vector<int> ccManualSeg::pointIdx(ccPointCloud* cloud, ccPolyline* poly)
{

	//Create pop-up window
	ProfileImportDlg piDlg(m_app->getMainWindow());
	ccGLWindow* win = m_app->getActiveGLWindow();
	vector<int> ptsIdx;
	int cnt = 0;

	ccGenericPointCloud* gCloud = ccHObjectCaster::ToGenericPointCloud(cloud);
	assert(gCloud);
	gCloud->resetVisibilityArray();

	if (!poly)
	{
		ccLog::Error("No polyline defined!");
		
	}

	if (!poly->isClosed())
	{
		ccLog::Error("Define and/or close the segmentation polygon first! (right click to close)");
		
	}

	//viewing parameters
	ccGLCameraParameters camera;
	win->getGLCameraParameters(camera);
	const double half_w = camera.viewport[2] / 2.0;
	const double half_h = camera.viewport[3] / 2.0;


	m_segmentationPoly->clear();
	m_polyVertices->clear();
	

	CCCoreLib::GenericIndexedCloudPersist* vertices = poly->getAssociatedCloud();
	bool mode3D = !poly->is2DMode();


	//duplicate polyline 'a minima' (only points and indexes + closed state)
	if (m_polyVertices->reserve(vertices->size() + (poly->isClosed() ? 0 : 1))
		&& m_segmentationPoly->reserve(poly->size() + (poly->isClosed() ? 0 : 1)))
	{
		for (unsigned i = 0; i < vertices->size(); ++i)
		{
			CCVector3 P = *vertices->getPoint(i);
			if (mode3D)
			{
				CCVector3d Q2D;
				camera.project(P, Q2D);

				P.x = static_cast<PointCoordinateType>(Q2D.x - half_w);
				P.y = static_cast<PointCoordinateType>(Q2D.y - half_h);
				P.z = 0;
			}
			m_polyVertices->addPoint(P);
		}

		for (unsigned j = 0; j < poly->size(); ++j)
		{
			m_segmentationPoly->addPointIndex(poly->getPointGlobalIndex(j));
		}
	}

	m_segmentationPoly->setClosed(poly->isClosed());


	ccGenericPointCloud::VisibilityTableType& visibilityArrayBase2 = gCloud->getTheVisibilityArray();
	
	for (int j = 0; j < static_cast<int>(gCloud->size()); ++j)
	{
		if (visibilityArrayBase2[j] == CCCoreLib::POINT_VISIBLE)
		{
			const CCVector3* P3D = gCloud->getPoint(j);

			CCVector3d Q2D;
			camera.project(*P3D, Q2D);

			CCVector2 P2D(static_cast<PointCoordinateType>(Q2D.x - half_w),
				static_cast<PointCoordinateType>(Q2D.y - half_h));

			bool pointInside = CCCoreLib::ManualSegmentationTools::isPointInsidePoly(P2D, m_segmentationPoly);


			visibilityArrayBase2[j] = (false != pointInside ? CCCoreLib::POINT_HIDDEN : CCCoreLib::POINT_VISIBLE); //visibility values of 1 are points inside (with false in this line)
			cnt = cnt + visibilityArrayBase2[j];
			if (visibilityArrayBase2[j] == 1)
				ptsIdx.push_back(j);

		}
	}

	return ptsIdx;



}

// Returns a the contour as a polyline
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
		CCVector3 p(stone->getPoint(e)->x, 0, stone->getPoint(e)->z);
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
		pContour->setName(stone->getName() + " - Contour");
		pContour->addChild(contour);
		pContour->setColor(ccColor::cyan);
		pContour->showColors(true);
		pContour->showVertices(true);

	}

	return pContour;

}

// Returns stone indexes given points idexes 
vector<int> ccManualSeg::stIdFromPtId(std::vector<int> pts, std::vector<pair<int, int>> pairs)
{
	vector<int> v;

	for (auto e : pts)
			if (!(find(v.begin(), v.end(), e) != v.end()))
				v.push_back(pairs[e].second);

	sort(v.begin(), v.end());

	v.erase(unique(v.begin(), v.end()), v.end());

	return v;
}

// Returns returns points indexes given stone indexes
vector<int> ccManualSeg::ptIdFromStId(std::vector<int> stIdx, std::vector<pair<int, int>> pairs)
{
	vector<int> v;

	for (auto e : stIdx)
		for (unsigned int i = 0; i < pairs.size();i++)
			if (pairs[i].second == e)
				v.push_back(pairs[i].first);

	return v;
}

// Returns the centroid of a 2D point data set 
pair <double, double> getCentroid(vector<pair <double, double>> V)
{
	double Sx = 0;
	double Sz = 0;
	int n = V.size();

	for (auto e : V )
	{ 
		Sx += e.first;
		Sz += e.second;
	}

	return(make_pair(Sx / n, Sz / n));
}


// Returns indexes of k (3 by default) nearest neighbors 
vector <int> getKNN(pair<double, double> P, vector<pair<double, double>> V, int k =3)
{

	if (k > V.size())
		k = V.size();

	vector < pair<int, double>> distances;
	double d;
	vector <int> t (k);


	for (unsigned i = 0; i < V.size();++i)
	{
		d = sqrt(pow((P.first - V[i].first), 2) + pow((P.second - V[i].second), 2));
		distances.push_back(make_pair(i, d));
	}


	sort(distances.begin(), distances.end(), [](auto &left, auto &right) {return left.second < right.second;});

	for (unsigned i = 0; i < k;++i)
	{
		t[i] = distances[i].first;
	}

	return t;
}

// Returns the number of common points within two data sets
int getNCommonPts(vector<pair<double, double>> tar, vector<pair<double, double>> ref)
{
	int n = 0;
	int n0 = 0;

	vector<pair<double, double>> allpts = tar;

	sort(allpts.begin(), allpts.end());
	allpts.erase(unique(allpts.begin(), allpts.end()), allpts.end());

	allpts.insert(allpts.end(), ref.begin(), ref.end());
	sort(allpts.begin(), allpts.end());


	n0 = allpts.size();

	allpts.erase(unique(allpts.begin(), allpts.end()), allpts.end());

	n = n0 - allpts.size();

	return n;
}


// Returns the indexes of common points in two Point Clouds 
vector<vector<int>> getCommonPtsIdx(ccPointCloud* Cl1, ccPointCloud* Cl2)
{
	vector<vector<int>> CommonPtsIdx(2);
	vector<pair<float,float>> Pts1(Cl1->size());
	vector<pair<float, float>> Pts2(Cl2->size());
	vector<pair<float, float>> Pts1C;
	vector<pair<float, float>> Pts2C;
	vector<pair<float, float>> allPts;
	vector<pair<float, float>> uniquePts;
	vector<pair<float, float>> commonPts;


	for (unsigned i = 0; i < Cl1->size(); i++)
	{
		Pts1[i] = make_pair(Cl1->getPoint(i)->x,Cl1->getPoint(i)->z);
	}
	for (unsigned i = 0; i < Cl2->size(); i++)
	{
		Pts2[i] = make_pair(Cl2->getPoint(i)->x, Cl2->getPoint(i)->z);
	}

	Pts1C = Pts1;
	Pts2C = Pts2;

	sort(Pts1C.begin(), Pts1C.end());
	sort(Pts2C.begin(), Pts2C.end());

	Pts1C.erase(unique(Pts1C.begin(), Pts1C.end()), Pts1C.end());
	Pts2C.erase(unique(Pts2C.begin(), Pts2C.end()), Pts2C.end());

	allPts = Pts1C;
	allPts.insert(allPts.end(), Pts2C.begin(), Pts2C.end());
	sort(allPts.begin(), allPts.end());
	uniquePts = allPts;

	uniquePts.erase(unique(uniquePts.begin(), uniquePts.end()), uniquePts.end());
	std::set_difference(allPts.begin(), allPts.end(), uniquePts.begin(), uniquePts.end(), std::inserter(commonPts, commonPts.end()));
	sort(commonPts.begin(), commonPts.end());


	for (unsigned i = 0; i < Cl1->size(); i++)
	{
		if (std::binary_search(commonPts.begin(), commonPts.end(), Pts1[i]))
			CommonPtsIdx[0].push_back(i);
	}

	for (unsigned i = 0; i < Cl2->size(); i++)
	{
		if (std::binary_search(commonPts.begin(), commonPts.end(), Pts2[i]))
			CommonPtsIdx[1].push_back(i);
	}

	return CommonPtsIdx;

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

vector<int> setIntersectIdxPixs1D(vector<int> listNew, vector<int> listOld) {
	vector<int> commonList;

	//Common
	int cnt = 0;
	for (auto &e : listNew) {
		for (auto &ee : listOld) {
			if (e == ee) {
				commonList.push_back(cnt);
				break;
			}
		}
		cnt++;
	}

	return commonList;
}

ccPolyline* contourPoly2(ccPointCloud* cloud0, vector<int> V, QString name)
{
	std::vector<CCCoreLib::PointProjectionTools::IndexedCCVector2> point;
	std::list<CCCoreLib::PointProjectionTools::IndexedCCVector2*> hullPoint2;


	for (unsigned i = 0; i < V.size();++i)

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
		CCVector3 p(cloud0->getPoint(V[e])->x, cloud0->getPoint(V[e])->y, cloud0->getPoint(V[e])->z); //kike added "y" value (it was 0)
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


// Returns mortar maps (Depth and width)
ccPointCloud* getMortarMaps(ccPointCloud* f_cloudStones, ccPointCloud* f_cloudMortar)
{
	//Binary Mortar

	//Bringing clouds to the origin
	CCVector3 minBox0;
	minBox0 = CCVector3(0, 0, 0);
	CCVector3 maxBox0;
	maxBox0 = CCVector3(0, 0, 0);
	f_cloudMortar->getBoundingBox(minBox0, maxBox0);
	f_cloudMortar->translate(-minBox0);
	f_cloudStones->translate(-minBox0);

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

	f_cloudMortar->translate(minBox0);
	f_cloudStones->translate(minBox0);
	f_skelMortar->translate(minBox0);
	widthSF->computeMinAndMax();
	f_skelMortar->setPointSize(10);
	f_skelMortar->showSF(true);
	f_skelMortar->setName("Mortar Maps");
	return f_skelMortar;
}


void ccManualSeg::doAction()
{	

	//Create log file

	time_t rawtime;
	struct tm * timeinfo;
	char buffer[80];

	time(&rawtime);
	timeinfo = localtime(&rawtime);

	strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", timeinfo);
	std::string str(buffer);

	string filename("MSP_log_");
	filename += str;
	filename += ".txt";

	ofstream auto_seg_log;
	auto_seg_log.open(filename);

	string stamp;

	time_t my_time = time(NULL);



	if ( m_app == nullptr )
	{
		// m_app should have already been initialized by CC when plugin is loaded
		Q_ASSERT( false );
		
		return;
	}

	ccGLWindow* win = m_app->getActiveGLWindow();
	win->setView(CC_FRONT_VIEW);

	int cnt = 0;
	int cnt2 = 0;

	//Dialog showing message. 31/10/2018
	ProfileImportDlg piDlg(m_app->getMainWindow());

	//Check loaded point clouds (there should be 2: base and segmented)
	ccHObject* root = m_app->dbRootObject();

	std::vector<int> idxPoly;
	int idxStone = -1;
	int idxMortar = -1;
	int refStone = -1;
	int refMortar = -1;

	ccHObject::Container pcs;
	ccHObject::Container polys;
	ccHObject::Container contourfolder;
	ccHObject::Container oldMortarMaps;

	bool bench = false;

	//removing old contours and mortar maps from DB
	if (root)
	{
		root->filterChildren(contourfolder, true, CC_TYPES::HIERARCHY_OBJECT);
		root->filterChildren(oldMortarMaps, true, CC_TYPES::POINT_CLOUD);
		
		if (contourfolder.size() >= 0)
		{

			for (int i = 0; i < contourfolder.size(); ++i)
			{
				ccHObject* ContoursContainer = contourfolder[i];

				QString s = ContoursContainer->getName();
				string  s0 = s.toStdString();
				string s1 = "tone contours";
				
				if (s0.find(s1) != std::string::npos)
				{
					m_app->removeFromDB(ContoursContainer,false);
				}
			}
		}


		if (oldMortarMaps.size() >= 0)
		{

			for (int i = 0; i < contourfolder.size(); ++i)
			{
				ccHObject* Maps = contourfolder[i];

				QString s = Maps->getName();
				string  s0 = s.toStdString();
				string s1 = "Mortar Maps";

				if (s0.find(s1) != std::string::npos)
				{
					m_app->removeFromDB(Maps, false);
				}
			}
		}



	}


	ccPointCloud* cloudStone = new ccPointCloud("cloudStone");
	ccPointCloud* cloudMortar = new ccPointCloud("cloudMortar");
	ccPointCloud* refStoneCloud = new ccPointCloud("refStoneCloud");
	ccPointCloud* refMortarCloud = new ccPointCloud("refMortarCloud");

	CCVector3 minBox0;
	minBox0 = CCVector3(0, 0, 0);
	CCVector3 maxBox0;
	maxBox0 = CCVector3(0, 0, 0);

	if (root)
	{

		int sizeClouds = 1000000000;

			root->filterChildren(pcs, true, CC_TYPES::POINT_CLOUD);
			
			root->filterChildren(polys, true, CC_TYPES::POLY_LINE);

			

			int t = pcs.size();
			int n = polys.size();

			if (t >= 2 )
			{ 
				for (int i = 0; i < pcs.size(); ++i)
				{
					ccPointCloud* cl = static_cast<ccPointCloud*>(pcs.at(i));
					QString s = cl->getName();
					string  s0 = s.toStdString();
					string s1 = "tone";
					string s2 = "ortar";
					string s3 = "ref";


					if ((s0.find(s3) != std::string::npos)&& (s0.find(s1) != std::string::npos))
					{
						refStone = i;
						string cComment = "reference stone cloud found!";
						QString qComment = QString::fromUtf8(cComment.c_str());
						m_app->dispToConsole(qComment, ccMainAppInterface::STD_CONSOLE_MESSAGE);

						stamp = dateStamp();
						auto_seg_log << stamp << " Reference stone cloud found" << endl;
					}

					if ((s0.find(s3) != std::string::npos) && (s0.find(s2) != std::string::npos))
					{
						refMortar = i;
						string cComment = "reference mortar cloud found!";
						QString qComment = QString::fromUtf8(cComment.c_str());
						m_app->dispToConsole(qComment, ccMainAppInterface::STD_CONSOLE_MESSAGE);
						
						stamp = dateStamp();
						auto_seg_log << stamp << " Reference mortar cloud found" << endl;
					}



					if (!(s0.find(s3) != std::string::npos) && (s0.find(s1) != std::string::npos))
					{
						idxStone = i;
						string cComment = "Stone cloud found!";
						QString qComment = QString::fromUtf8(cComment.c_str());
						m_app->dispToConsole(qComment, ccMainAppInterface::STD_CONSOLE_MESSAGE);
						
						stamp = dateStamp();
						auto_seg_log << stamp << " Stone cloud found" << endl;
					}

					if (!(s0.find(s3) != std::string::npos) && (s0.find(s2) != std::string::npos))
					{
						idxMortar = i;
						string cComment = "Mortar cloud found!";
						QString qComment = QString::fromUtf8(cComment.c_str());
						m_app->dispToConsole(qComment, ccMainAppInterface::STD_CONSOLE_MESSAGE);

						stamp = dateStamp();
						auto_seg_log << stamp << " Mortar cloud found" << endl;
					}

					// For Bench

					if (n == 0 && refStone != -1 && idxStone != -1 && refMortar != -1 && idxMortar!= -1 )
					{
						bench = true;
						
						refStoneCloud = static_cast<ccPointCloud*>(pcs.at(refStone));
						refMortarCloud = static_cast<ccPointCloud*>(pcs.at(refMortar));
						cloudStone = static_cast<ccPointCloud*>(pcs.at(idxStone));
						cloudMortar = static_cast<ccPointCloud*>(pcs.at(idxMortar));

					}

					//
				}

			}

			else {

					m_app->dispToConsole("At least one cloud and a polyline are required", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
					return;
				}

			

			
			if (!bench)
			{

				if (idxMortar != -1 && idxStone != -1 )
				{
					cloudStone = static_cast<ccPointCloud*>(pcs.at(idxStone));
					cloudMortar = static_cast<ccPointCloud*>(pcs.at(idxMortar));
				}

				else
				{

					if (idxMortar == -1 && idxStone == -1)
					{

						int cnt = 0;

						for (int i = 0; i < pcs.size(); ++i)
						{
							ccPointCloud* cl = static_cast<ccPointCloud*>(pcs.at(i));

							if (cl->size() > 100)
							{
								idxMortar = i;
								cnt++;
							}


						}

						if (cnt > 1)
						{
							m_app->dispToConsole("For manual segmentation leave only one cloud in DB", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
							return;
						}


						cloudMortar = static_cast<ccPointCloud*>(pcs.at(idxMortar));

						
						cloudMortar->getBoundingBox(minBox0, maxBox0);

					}

					else
					{
						m_app->dispToConsole("Only one cloud has been foud", ccMainAppInterface::ERR_CONSOLE_MESSAGE);

						m_app->dispToConsole("for manual segmentation please change it's name so that it doesn't contain the sequence 'stone' or 'mortar'", ccMainAppInterface::ERR_CONSOLE_MESSAGE);

						return;

					}


				}
			}
		
	}

	else
	{
		return;
	}


	if (!bench)
	{
		// saving point size

		unsigned pointSizeMortar = cloudMortar->getPointSize();


		std::vector<int> recIdx;
		std::vector<int> polyIdx;

		// Make the distinction between rectangles and non rectangular polylines

		for (unsigned int i = 0; i < polys.size(); i++)
		{
			ccPolyline* poly = static_cast<ccPolyline*>(polys.at(i));
			//poly->point
			int j = poly->size();

			const CCVector3* Pt0 = poly->getPoint(0);
			float x0 = poly->getPoint(0)->x;
			float x1 = poly->getPoint(1)->x;
			float x2 = poly->getPoint(2)->x;
			float x3 = poly->getPoint(3)->x;
			float dx1 = x1 - x0;
			float dx2 = x3 - x2;
			bool a = false;

			if (poly->size() == 4 && dx1 == dx2)
			{
				recIdx.push_back(i);
			}
			else { polyIdx.push_back(i); }

		}

		// Creating vectors 

		vector <vector<pair<int, int>>> stonePairs(2);
		vector <vector<int>> mortarPoints(2);
		vector<int> allStIdx;


		int sfindex = cloudStone->getScalarFieldIndexByName("Stone Index");
		CCCoreLib::ScalarField* stIdSF = cloudStone->getScalarField(sfindex);

		for (unsigned int i = 0; i < cloudMortar->size();i++)
			mortarPoints[0].push_back(i);


		int maxId = 0;

		if (cloudStone->size() > 0)
		{
			for (unsigned int i = 0; i < cloudStone->size();i++)
			{
				stonePairs[0].push_back(make_pair(i, stIdSF->getValue(i)));
				allStIdx.push_back(stIdSF->getValue(i));
			}


			sort(allStIdx.begin(), allStIdx.end());

			int n = allStIdx.size();
			maxId = allStIdx[n - 1];
		}

		stamp = dateStamp();
		auto_seg_log << " Evaluating polylines (" << polyIdx.size() + recIdx.size() << ")..." << endl;

		// rectangular polyline 
		if (recIdx.size() != 0)
		{
			stamp = dateStamp();
			auto_seg_log << " Deleting incorrectly identified stones (" << recIdx.size() << ")..." << endl;
			int cntRect = 0;
			for (auto i : recIdx)
			{
				cntRect++;
				stamp = dateStamp();
				auto_seg_log << stamp << " Polyline " << cntRect << "/" << recIdx.size() << endl;

				vector<int> v;
				vector<int> stId;

				ccPolyline* poly = static_cast<ccPolyline*>(polys.at(i));

				v = pointIdx(cloudStone, poly);
				stId = stIdFromPtId(v, stonePairs[0]);

				for (auto e : stId)
					for (unsigned int i = 0; i < stonePairs[0].size(); ++i)
					{
						if (stonePairs[0][i].second == e)
						{
							stonePairs[0][i].second = -1;
							mortarPoints[1].push_back(stonePairs[0][i].first);

						}
					}


				m_app->removeFromDB(poly, false);
			}

		}


		// Polylines

		if (polyIdx.size() != 0)
		{
		
			stamp = dateStamp();
			auto_seg_log << " Correcting boundaries of stones (" << polyIdx.size() << ")..." << endl;
			int cntPoly = 0;
			for (auto i : polyIdx)
			{
				cntPoly++;
				stamp = dateStamp();
				auto_seg_log << stamp << " Polyline " << cntPoly << "/" << polyIdx.size() << endl;
				ccPolyline* poly = static_cast<ccPolyline*>(polys.at(i));

				vector<int> vs;
				vector<int> vm;
				vector<int> stId;

				vs = pointIdx(cloudStone, poly);
				vm = pointIdx(cloudMortar, poly);

				int k;
				string kLabel;

				if (vm.size() == 0 && vs.size() != 0) { // values can be test for higher than zero just in case noise
					k = 1;
					kLabel = " (Removing stone)";
				}
				if (vs.size() == 0 && vm.size() != 0) {
					k = 2;
					kLabel = " (Adding new stone)";
				}
				if (vs.size() != 0 && vm.size() != 0) {
					k = 3;
					kLabel = " (Correcting previously segmented stone)";
				}

				stamp = dateStamp();
				auto_seg_log << stamp << " Case " << k << kLabel << endl;

			
				switch (k)
				{

				case 1:
				{
					vector<int> vr;
					vector<int> vall;

					stId = stIdFromPtId(vs, stonePairs[0]);


					if (stId.size() == 1 && stId[0] == -1)
					{
						maxId++;
						for (auto e : vs)
						{
							stonePairs[0][e].second = maxId;
						}

						vector <int> tempMortar = mortarPoints[1];
						vector<int> resMortar;
						sort(tempMortar.begin(), tempMortar.end());
						std::set_difference(tempMortar.begin(), tempMortar.end(), vs.begin(), vs.end(), std::inserter(resMortar, resMortar.end()));

						mortarPoints[1] = resMortar;

					}

					else
					{
						vall = ptIdFromStId(stId, stonePairs[0]);

						std::set_difference(vall.begin(), vall.end(), vs.begin(), vs.end(), std::inserter(vr, vr.end()));

						for (auto e : vr)
						{
							mortarPoints[1].push_back(e);
							stonePairs[0][e].second = -1;

						}
					}

					break;


				}

				case 2:
				{

					maxId++;

					for (auto e : vm)
					{
						stonePairs[1].push_back(make_pair(e, maxId));
						(mortarPoints[0])[e] = -1;
					}

					

					break;

				}


				case 3:
				{
			
					int minId;
					vector<int> vr;
					vector<int> vall;


					stId = stIdFromPtId(vs, stonePairs[0]); //Extract stone indices from selection

	

						minId = stId[0]; //Select minId from indices

						for (auto e : stId)
						{
							if (e < minId)
								minId = e;
						}

						if (minId == -1) // If there is stone labelled as -1 inside (not belonging to any stone)
						{
							maxId++;
							minId = maxId; //Create a new label (?). This should be done only if there is no other stone in the selection
						}

						for (auto e : vm) //for each point from mortar cloud in the selection
						{
							stonePairs[1].push_back(make_pair(e, minId)); //Add to stone vector. Note that stonePairs[1] contains points from the stone cloud to be added as mortar
							mortarPoints[0][e] = -1; //Remove from mortar (idx to -1)
						}

						vall = ptIdFromStId(stId, stonePairs[0]); //All the points with idx similar to the ones in the selection

						for (auto e : vall) { 
							stonePairs[0][e].second = minId; //Relabel all those points with minId 
						}

						sort(vs.begin(), vs.end());
						sort(vall.begin(), vall.end());
						std::set_difference(vall.begin(), vall.end(), vs.begin(), vs.end(), std::inserter(vr, vr.end())); //Difference between vall and vs > Extract to vr
						


						if (vr.size() > 0){ 
							for (auto e : vr) //Label these as mortar. Some points can be duplicates (e.g. stone divided in several chunks and outliers assigned to mortar several times), so this is checked below
							{
								mortarPoints[1].push_back(e); 
								stonePairs[0][e].second = -1; //remove from stone (label -1)

							}

						}

						//Remove duplicates
						sort(mortarPoints[1].begin(), mortarPoints[1].end());
						mortarPoints[1].erase(unique(mortarPoints[1].begin(), mortarPoints[1].end()), mortarPoints[1].end());


						if (stId[0] == -1) 
						{
						
							vector <int> tempMortar = mortarPoints[1];
							vector<int> resMortar;
							sort(tempMortar.begin(), tempMortar.end());
							std::set_difference(tempMortar.begin(), tempMortar.end(), vs.begin(), vs.end(), std::inserter(resMortar, resMortar.end()));

							mortarPoints[1] = resMortar;

						}

					break;

				}


				}

				m_app->removeFromDB(poly, false);


			}
		}


		win->redraw(true, true);

		int m = 5;

		ccColor::Rgb col;
		const CCVector3* pt;

		////Create globalCloud //Kike 06/11/2019
		ccPointCloud* new_CloudGlobal = new ccPointCloud("global");
		for (unsigned int i = 0; i < cloudStone->size(); ++i)
		{
			new_CloudGlobal->reserveThePointsTable(1);
			new_CloudGlobal->reserveTheRGBTable();

			col = cloudStone->getPointColor(i);
			pt = cloudStone->getPoint(i);


			new_CloudGlobal->addPoint(*pt);
			new_CloudGlobal->addColor(col);

		}

		for (unsigned int i = 0; i < cloudMortar->size(); ++i)
		{
			new_CloudGlobal->reserveThePointsTable(1);
			new_CloudGlobal->reserveTheRGBTable();

			col = cloudMortar->getPointColor(i);
			pt = cloudMortar->getPoint(i);


			new_CloudGlobal->addPoint(*pt);
			new_CloudGlobal->addColor(col);

		}

		///////


		// adding points to each point cloud

		ccPointCloud* new_CloudStone = new ccPointCloud("Stone - cloud");
		ccPointCloud* new_CloudMortar = new ccPointCloud("Mortar - cloud");


		// Adding points to the final Stone cloud


		vector<pair<int, int>> rStPairs;

		// removing points that are going to the mortar cloud (with index -1)
		for (auto e : stonePairs[0])
		{
			if (e.second != -1)
				rStPairs.push_back(e);
		}

		for (unsigned int i = 0; i < rStPairs.size(); ++i)
		{
			new_CloudStone->reserveThePointsTable(1);
			new_CloudStone->reserveTheRGBTable();

			col = cloudStone->getPointColor(rStPairs[i].first);
			pt = cloudStone->getPoint(rStPairs[i].first);

			
			new_CloudStone->addPoint(*pt);
			new_CloudStone->addColor(col);

		}


		// adding points coming from mortar


		if (stonePairs[1].size() != 0)
		{
			for (auto i = 0; i < stonePairs[1].size(); ++i)
			{
				new_CloudStone->reserveThePointsTable(1);
				new_CloudStone->reserveTheRGBTable();

				col = cloudMortar->getPointColor(stonePairs[1][i].first);
				pt = cloudMortar->getPoint(stonePairs[1][i].first);

				new_CloudStone->addPoint(*pt);
				new_CloudStone->addColor(col);

			}
		}

		//Adding scalrfield to the stone cloud


		CCCoreLib::ScalarField* new_stIdSF = nullptr;


		int nsfIdx = new_CloudStone->getScalarFieldIndexByName("Stone Index");
		if (nsfIdx < 0)
		{
			nsfIdx = new_CloudStone->addScalarField("Stone Index");
		}
		if (nsfIdx < 0)
		{
			return;
		}

		new_stIdSF = new_CloudStone->getScalarField(nsfIdx);


		for (unsigned int i = 0; i < rStPairs.size(); ++i)
		{
			int index = static_cast<int>(rStPairs[i].second);
			new_stIdSF->setValue(i, index);

		}




		// adding points coming from mortar

		if (stonePairs[1].size() != 0)
		{

			for (auto i = 0; i < stonePairs[1].size(); ++i)
			{

				int index = static_cast<int>(stonePairs[1][i].second);
				new_stIdSF->setValue(rStPairs.size() + i, index);

			}
		}



		new_CloudStone->setCurrentDisplayedScalarField(nsfIdx);

		new_stIdSF->computeMinAndMax();
		new_CloudStone->showColors(true);
		new_CloudMortar->showColors(true);



		for (auto e : mortarPoints[0])
		{
			if (e != -1)
			{
				new_CloudMortar->reserveThePointsTable(1);
				new_CloudMortar->reserveTheRGBTable();

				col = cloudMortar->getPointColor(e);
				pt = cloudMortar->getPoint(e);


				new_CloudMortar->addPoint(*pt);
				new_CloudMortar->addColor(col);

			}


		}

		// points coming from stone cloud

		if (mortarPoints[1].size() != 0)
		{

			for (auto e : mortarPoints[1])
			{
				if (e != -1)
				{
					new_CloudMortar->reserveThePointsTable(1);
					new_CloudMortar->reserveTheRGBTable();

					col = cloudStone->getPointColor(e);
					pt = cloudStone->getPoint(e);


					new_CloudMortar->addPoint(*pt);
					new_CloudMortar->addColor(col);
				}

			}
		}

		// moving the clouds to center in case they were not 

		if (idxStone == -1)
		{
			new_CloudStone->translate(-minBox0);
			new_CloudMortar->translate(-minBox0);
		}

		// Creating new vectors 

		int nbpts = new_CloudStone->size();
		vector<pair<int, int>> nstonePairs;
		vector<int> nallStIdx;


		int nsfindex = new_CloudStone->getScalarFieldIndexByName("Stone Index");
		CCCoreLib::ScalarField* nstIdSF = new_CloudStone->getScalarField(nsfindex);

		if (nbpts > 0)
		{
			for (unsigned int i = 0; i < nbpts;i++)
			{
				nstonePairs.push_back(make_pair(i, nstIdSF->getValue(i)));
				nallStIdx.push_back(nstIdSF->getValue(i));
			}

		}

		sort(nallStIdx.begin(), nallStIdx.end());
		nallStIdx.erase(unique(nallStIdx.begin(), nallStIdx.end()), nallStIdx.end());

		//correcting indexes

		vector<vector<int>> vecStones(nallStIdx.size());

		sort(nstonePairs.begin(), nstonePairs.end(), [](auto &left, auto &right) {return left.second < right.second;});

		int old_idx = nstonePairs[0].second;
		int new_idx = 0;

		for (unsigned i = 0; i < nstonePairs.size(); i++)
		{
			if (nstonePairs[i].second == old_idx)
			{
				nstonePairs[i].second = new_idx;
				vecStones[new_idx].push_back(nstonePairs[i].first);

			}


			else
			{
				new_idx++;
				old_idx = nstonePairs[i].second;
				nstonePairs[i].second = new_idx;
				vecStones[new_idx].push_back(nstonePairs[i].first);

			}

		}

		sort(nstonePairs.begin(), nstonePairs.end(), [](auto &left, auto &right) {return left.first < right.first;});


		//shuffling scalarfield for better visuals

		vector <int> stIdShuffle;

		for (auto e : nstonePairs)
		{
			stIdShuffle.push_back(e.second);
		}

		sort(stIdShuffle.begin(), stIdShuffle.end());

		stIdShuffle.erase(unique(stIdShuffle.begin(), stIdShuffle.end()), stIdShuffle.end());

		auto rng = std::default_random_engine{};
		std::shuffle(stIdShuffle.begin(), stIdShuffle.end(), rng);

		if (nbpts > 0)
		{
			for (unsigned int i = 0; i < nbpts;i++)
			{
				int index = static_cast<int>(stIdShuffle[nstonePairs[i].second]);
				nstIdSF->setValue(i, index);
			}
		}

		//Mortar maps
		auto_seg_log << " Starting mortar maps generation..." << endl;
		ccPointCloud* mortarMaps = getMortarMaps(new_CloudStone, new_CloudMortar);

		stamp = dateStamp();
		auto_seg_log << stamp << " Mortar maps OK" << endl;

		// Move all the clouds back
		new_CloudStone->translate(minBox0);
		new_CloudMortar->translate(minBox0);
		mortarMaps->translate(minBox0);


		//generating contour polylines

		ccHObject* nContoursContainer = new ccHObject("Stone contours");
		
		int cntSt = 0;
		stamp = dateStamp();
		auto_seg_log << " Starting contours regeneration"<< endl;

// contours in an alternative way using stone indexes and original cloud

		if (vecStones.size() > 0)
		{
			for (auto ve : vecStones)
			{
				string s_name = "stone " + to_string(cntSt);
				QString qs_name = QString::fromUtf8(s_name.c_str());
				ccPolyline* contour = nullptr;

				stamp = dateStamp();
				auto_seg_log << stamp << " Polyline " << cntSt << "/" << vecStones.size()-1 << endl;

				if (ve.size() > 0)
				{
					contour = contourPoly2(new_CloudStone, ve, qs_name);
					nContoursContainer->addChild(contour);
					cntSt++;
				}

				
			}
		}
		stamp = dateStamp();
		auto_seg_log << stamp << " Contours regeneration done" << endl;

		new_CloudMortar->setPointSize(pointSizeMortar);
		new_CloudStone->setPointSize(pointSizeMortar);

			
		m_app->addToDB(new_CloudStone, true, true, false, true);
		m_app->addToDB(new_CloudMortar, true, true, false, true);
		m_app->addToDB(nContoursContainer, true, true, false, true);
		m_app->addToDB(mortarMaps, true, true, false, true);


		ccGLWindow* win = m_app->getActiveGLWindow();
		win->setView(CC_FRONT_VIEW);


	}
}

