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

//PCL
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

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

//OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//System
#include <iostream>
#include <fstream>
#include <random>
#include <algorithm>
#include <math.h>  

using namespace std;
using namespace cv;

ccManualSeg::ccManualSeg( QObject *parent )
	: QObject( parent )
	, ccStdPluginInterface( ":/CC/plugin/qManualSeg/info.json" )
	, m_action(nullptr)
{
}

QList<QAction*> ccManualSeg::getActions()
{
	if ( !m_action )
	{
		m_action = new QAction( getName(), this );
		m_action->setToolTip( getDescription() );
		m_action->setIcon(QIcon(QString::fromUtf8(":/CC/plugin/qManualSeg/cyberbuildIcon.png")));
		
		// Connect appropriate signal
		connect( m_action, &QAction::triggered, this, &ccManualSeg::doAction );
	}

	return { m_action };
}

vector<unsigned> ccManualSeg::pointIdx(ccPointCloud* cloud, ccPolyline* poly) const
{
	if (!cloud)
	{
		assert(false);
		return {};
	}

	if (!poly)
	{
		ccLog::Error("No polyline defined!");
		return {};
	}

	if (!poly->isClosed())
	{
		ccLog::Error("Define and/or close the segmentation polygon first! (right click to close)");
		return {};
	}

	ccGLWindow* win = m_app->getActiveGLWindow();
	if (!win)
	{
		assert(false);
		return {};
	}

	//viewing parameters
	ccGLCameraParameters camera;
	win->getGLCameraParameters(camera);
	const double half_w = camera.viewport[2] / 2.0;
	const double half_h = camera.viewport[3] / 2.0;

	//duplicate polyline 'a minima' (only points and indexes + closed state)
	ccPointCloud* polyVertices = new ccPointCloud("vertices");
	ccPolyline segmentationPoly(polyVertices);
	{
		segmentationPoly.set2DMode(true);
		segmentationPoly.addChild(polyVertices);

		CCCoreLib::GenericIndexedCloudPersist* vertices = poly->getAssociatedCloud();
		assert(vertices);

		if (	!polyVertices->reserve(vertices->size() + (poly->isClosed() ? 0 : 1))
			||	!segmentationPoly.reserve(poly->size() + (poly->isClosed() ? 0 : 1)))
		{
			ccLog::Error("Not enough memory");
			return  {};
		}

		bool mode3D = !poly->is2DMode();

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
			polyVertices->addPoint(P);
		}
	
		for (unsigned j = 0; j < poly->size(); ++j)
		{
			segmentationPoly.addPointIndex(poly->getPointGlobalIndex(j));
		}
	
		segmentationPoly.setClosed(poly->isClosed());
	}
	
	if (!cloud->isVisibilityTableInstantiated() && !cloud->resetVisibilityArray())
	{
		ccLog::Error("Not enough memory");
		return {};
	}

	vector<unsigned> ptsIdx;

	try
	{
		ccGenericPointCloud::VisibilityTableType& gVisibilityArray = cloud->getTheVisibilityArray();
		for (unsigned j = 0; j < cloud->size(); ++j)
		{
			if (gVisibilityArray[j] == CCCoreLib::POINT_VISIBLE)
			{
				const CCVector3* P3D = cloud->getPoint(j);

				CCVector3d Q2D;
				camera.project(*P3D, Q2D);

				CCVector2 P2D(	static_cast<PointCoordinateType>(Q2D.x - half_w),
								static_cast<PointCoordinateType>(Q2D.y - half_h));

				bool pointInside = CCCoreLib::ManualSegmentationTools::isPointInsidePoly(P2D, &segmentationPoly);

				gVisibilityArray[j] = (pointInside ? CCCoreLib::POINT_HIDDEN : CCCoreLib::POINT_VISIBLE); //visibility values of 1 are points inside
				if (pointInside)
				{
					ptsIdx.push_back(j);
				}
			}
		}
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Error("Not enough memory");
		return  {};
	}

	ptsIdx.shrink_to_fit();

	return ptsIdx;
}

vector<int> ccManualSeg::StIdFromPtId(const std::vector<unsigned>& pts, const std::vector<pair<unsigned, int>>& pairs)
{
	vector<int> v;

	try
	{
		for (unsigned ptIndex : pts)
		{
			if (find(v.begin(), v.end(), ptIndex) == v.end())
			{
				v.push_back(pairs[ptIndex].second);
			}
		}

		sort(v.begin(), v.end());

		v.erase(unique(v.begin(), v.end()), v.end());
	}
	catch (const std::bad_alloc&)
	{
		// not enough memory
		return {};
	}

	return v;
}

vector<unsigned> ccManualSeg::PtIdFromStId(const std::vector<int>& stIdx, const std::vector<pair<unsigned, int>>& pairs)
{
	vector<unsigned> v;

	try
	{
		for (int ptIndex : stIdx)
		{
			for (size_t i = 0; i < pairs.size(); i++)
				if (pairs[i].second == ptIndex)
					v.push_back(pairs[i].first);
		}
	}
	catch (const std::bad_alloc&)
	{
		// not enough memory
		return {};
	}

	return v;
}

static void Cloud2Binary(ccPointCloud* cloud, Mat& corrMatS, vector<Point>& idxPxS, vector<int>& idxPx1DS, Mat& imageBWS)
{
	if (!cloud)
	{
		assert(false);
		return;
	}
	
	//Correspondence 2D->1D. Each px has an index instead of two coords
	int cntC = 0;
	for (int y = 0; y < corrMatS.rows; y++)
	{
		for (int x = 0; x < corrMatS.cols; x++)
		{
			corrMatS.at<int>(y, x) = cntC;
			++cntC;
		}
	}

	//Binary
	unsigned s = cloud->size();
	for (unsigned i = 0; i < s; i++)
	{
		//depth
		const CCVector3* pointC0 = cloud->getPoint(i);
		int z = floor(pointC0->z);
		int x = floor(pointC0->x);
		imageBWS.at<uchar>(z, x) = 255;

		//idxPx per 3D point
		idxPxS.push_back(Point(z, x));
		idxPx1DS.push_back(corrMatS.at<int>(z, x));
	}
}

//Extract Skeleton from Binary (CV_8U)
static Mat Skeleton(const Mat& I, bool flagInv)
{
	Mat toSkel;
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
	} while (!done && (iterations < 100));

	return skel;
}

vector<int> SetIntersectIdxPixs1D(const vector<int>& listNew, const vector<int>& listOld)
{
	vector<int> commonList;
	try
	{
		//Common
		int cnt = 0;
		for (auto &e : listNew)
		{
			for (auto &ee : listOld)
			{
				if (e == ee)
				{
					commonList.push_back(cnt);
					break;
				}
			}
			++cnt;
		}
	}
	catch (const std::bad_alloc&)
	{
		return  {};
	}

	return commonList;
}

ccPolyline* ContourPoly2(ccPointCloud* cloud0, const vector<int>& V, const QString& name)
{
	if (!cloud0)
	{
		assert(false);
		return nullptr;
	}

	vector<unsigned> Vec;
	try
	{
		std::vector<CCCoreLib::PointProjectionTools::IndexedCCVector2> points;
		for (size_t i = 0; i < V.size(); ++i)
		{
			PointCoordinateType x = cloud0->getPoint(V[i])->x;
			PointCoordinateType z = cloud0->getPoint(V[i])->z;
			CCCoreLib::PointProjectionTools::IndexedCCVector2 P(x, z, i);

			points.push_back(P);
		}

		std::list<CCCoreLib::PointProjectionTools::IndexedCCVector2*> hullPoint2;
		CCCoreLib::PointProjectionTools::extractConcaveHull2D(points, hullPoint2, 0.0001);

		for (size_t i = 0; i < hullPoint2.size(); ++i)
		{
			auto it = hullPoint2.begin();
			std::advance(it, i);
			CCCoreLib::PointProjectionTools::IndexedCCVector2* P2 = *it;
			Vec.push_back(P2->index);
		}
	}
	catch (const std::bad_alloc&)
	{
		return  {};
	}

	ccPointCloud* contour = new ccPointCloud(name + " - Contour Cloud");
	if (!contour->reserve(Vec.size()) || !contour->reserveTheRGBTable())
	{
		// not enough memory
		ccLog::Warning("Not enough memory");
		delete contour;
		return nullptr;
	}

	for (unsigned pointIndex : Vec)
	{
		const CCVector3* P0 = cloud0->getPoint(pointIndex);
		contour->addPoint(*P0);

		const ccColor::Rgba& col = cloud0->getPointColor(pointIndex);
		contour->addColor(col);
	}

	unsigned cnt = static_cast<unsigned>(Vec.size());

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
		ccLog::Warning("Not enough memory");
		delete pContour;
		pContour = nullptr;
	}

	return pContour;
}

static string DateStamp()
{
	time_t rawtime;
	time(&rawtime);

	struct tm* timeinfo = localtime(&rawtime);
	char buffer[80];
	strftime(buffer, sizeof(buffer), "%H:%M:%S", timeinfo);
	std::string str(buffer);

	return "[" + str + "]";
}

// Returns mortar maps (Depth and width)
static ccPointCloud* GetMortarMaps(ccPointCloud* f_cloudStones, ccPointCloud* f_cloudMortar)
{
	if (!f_cloudStones || !f_cloudMortar)
	{
		assert(false);
		return nullptr;
	}

	//Binary Mortar

	//Bringing clouds to the origin
	CCVector3 minBox0, maxBox0;
	f_cloudMortar->getBoundingBox(minBox0, maxBox0);
	f_cloudMortar->translate(-minBox0);
	f_cloudStones->translate(-minBox0);

	f_cloudMortar->scale(100, 1000, 100); //To cm mm cm
	CCVector3 minBox, maxBox;
	f_cloudMortar->getBoundingBox(minBox, maxBox);
	
	int rowsM = static_cast<int>(ceil(maxBox.z));
	int colsM = static_cast<int>(ceil(maxBox.x));

	cv::Mat corrMatM = Mat::zeros(rowsM, colsM, CV_32F);
	cv::Mat imageBWS = Mat::zeros(rowsM, colsM, CV_8U);
	vector<Point> idxPxM;
	vector<int> idxPxM1D;
	Cloud2Binary(f_cloudMortar, corrMatM, idxPxM, idxPxM1D, imageBWS);

	//Skeleton
	Mat skel = Skeleton(imageBWS, false);

	f_cloudMortar->scale(0.01, 0.001, 0.01); //Scale back

	//Skeleton 3D
	vector<int> pixelsSkel;
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

	vector<int> idxCloudMortarSkel = SetIntersectIdxPixs1D(idxPxM1D, pixelsSkel);
	ccPointCloud* skelM = new ccPointCloud("Skeleton Mortar");
	{
		if (!skelM->reserve(idxCloudMortarSkel.size()) || !skelM->reserveTheRGBTable())
		{
			ccLog::Warning("Not enough memory");
			delete skelM;
			return nullptr;
		}
		for (auto e : idxCloudMortarSkel)
		{
			skelM->addPoint(*f_cloudMortar->getPoint(e));
			const ccColor::Rgba& col = f_cloudMortar->getPointColor(e);
			skelM->addColor(col);
		}
	}

	//Subsample result
	CCCoreLib::CloudSamplingTools::SFModulationParams modParams(false);
	CCCoreLib::ReferenceCloud* refCloud = CCCoreLib::CloudSamplingTools::resampleCloudSpatially(skelM, 0.01, modParams, 0, 0); //1 point per cm^2
	if (!refCloud)
	{
		ccLog::Warning("Not enough memory");
		delete skelM;
		return nullptr;
	}

	//Save output
	ccPointCloud* f_skelMortar = skelM->partialClone(refCloud);

	delete refCloud;
	refCloud = nullptr;
	delete skelM;
	skelM = nullptr;

	if (!f_skelMortar)
	{
		ccLog::Warning("Not enough memory");
		return nullptr;
	}
	f_skelMortar->setName("Mortar Maps");

	//Maps using PCL
	//Mortar depth map
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloudStones(new pcl::PointCloud<pcl::PointXYZ>);
	{
		pcl_cloudStones->width = f_cloudStones->size();
		pcl_cloudStones->height = 1;
		pcl_cloudStones->points.resize(pcl_cloudStones->width * pcl_cloudStones->height);

		for (size_t i = 0; i < f_cloudStones->size(); ++i)
		{
			pcl_cloudStones->points[i].x = f_cloudStones->getPoint(i)->x;
			pcl_cloudStones->points[i].y = f_cloudStones->getPoint(i)->y;
			pcl_cloudStones->points[i].z = f_cloudStones->getPoint(i)->z;
		}
	}

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(pcl_cloudStones);

	vector<double> distStD, dist3St, distStD2, distStWidth, distStW;
	for (unsigned i = 0; i < f_skelMortar->size(); i++)
	{
		pcl::PointXYZ searchPoint(	f_skelMortar->getPoint(i)->x,
									f_skelMortar->getPoint(i)->y,
									f_skelMortar->getPoint(i)->z);

		std::vector<int> pointIdxRadiusSearch;
		std::vector<float> pointRadiusSquaredDistance;

		float radius = 0.1f; //10cm

		if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
		{
			pcl::PointXYZ nnPoint = pcl_cloudStones->points[pointIdxRadiusSearch[0]];

			double disty = abs(searchPoint.y - nnPoint.y); //Depth
			double distx = abs(searchPoint.x - nnPoint.x);
			double distz = abs(searchPoint.z - nnPoint.z);
			double dist3 = sqrt(distx*distx + distz*distz);//2D distance (XZ) will be used as a reference for the creation of mortar width map		

			distStD.push_back(disty * 1000); //depth (y axis) in mm
			dist3St.push_back(dist3);

			double dist1 = 1000;
			double dist2 = 1000;
			double dist1y = 1000;
			double dist2y = 1000;
			int idx1 = -1;
			int idx2 = -1;
			for (size_t j = 0; j < pointIdxRadiusSearch.size(); ++j)
			{
				double distx = abs(searchPoint.x - pcl_cloudStones->points[pointIdxRadiusSearch[j]].x); //2D distance
				double distz = abs(searchPoint.z - pcl_cloudStones->points[pointIdxRadiusSearch[j]].z);
				double disty = abs(searchPoint.y - pcl_cloudStones->points[pointIdxRadiusSearch[j]].y);
				double dist3 = sqrt(distx*distx + distz*distz);

				int idxSt = f_cloudStones->getPointScalarValue(pointIdxRadiusSearch[j]);

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
					if (idxSt != idx1 && idx2 == -1)
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
	{
		static constexpr const char MortarRelativeDepthSFName[] = "Mortar relative depth (mm)";
		int sfIdxD = f_skelMortar->getScalarFieldIndexByName(MortarRelativeDepthSFName);
		if (sfIdxD < 0)
		{
			sfIdxD = f_skelMortar->addScalarField(MortarRelativeDepthSFName);
		}

		CCCoreLib::ScalarField* depthSF = f_skelMortar->getScalarField(sfIdxD);
		if (depthSF)
		{
			for (unsigned int i = 0; i < distStD2.size(); i++)
			{
				ScalarType val = static_cast<ScalarType>(distStD2.at(i));
				depthSF->setValue(i, val);
			}
			depthSF->computeMinAndMax();

			f_skelMortar->setCurrentDisplayedScalarField(sfIdxD);
			f_skelMortar->showSF(true);
		}
		else
		{
			ccLog::Warning(QString("Failed to create SF:") + MortarRelativeDepthSFName);
		}
	}

	//Add width as a scalarField to f_skelMortar
	{
		static constexpr const char MortarRelativeWidthSFName[] = "Mortar relative width (mm)";
		int sfIdxW = f_skelMortar->getScalarFieldIndexByName(MortarRelativeWidthSFName);
		if (sfIdxW < 0)
		{
			sfIdxW = f_skelMortar->addScalarField(MortarRelativeWidthSFName);
		}

		CCCoreLib::ScalarField* widthSF = f_skelMortar->getScalarField(sfIdxW);
		if (widthSF)
		{
			for (unsigned int i = 0; i < distStW.size(); i++)
			{
				ScalarType val = static_cast<ScalarType>(distStW.at(i));
				widthSF->setValue(i, val);
			}
			widthSF->computeMinAndMax();

			f_skelMortar->setCurrentDisplayedScalarField(sfIdxW);
			f_skelMortar->showSF(true);
		}
		else
		{
			ccLog::Warning(QString("Failed to create SF:") + MortarRelativeWidthSFName);
		}
	}

	f_cloudMortar->translate(minBox0);
	f_cloudStones->translate(minBox0);
	f_skelMortar->translate(minBox0);
	f_skelMortar->setPointSize(10);
	f_skelMortar->setName("Mortar Maps");
	
	return f_skelMortar;
}

void ccManualSeg::doAction()
{
	if (m_app == nullptr)
	{
		// m_app should have already been initialized by CC when plugin is loaded
		assert(false);
		return;
	}

	ccHObject* root = m_app->dbRootObject();
	if (!root)
	{
		ccLog::Error("No DB root object");
		return;
	}

	ccHObject::Container pcs;
	root->filterChildren(pcs, true, CC_TYPES::POINT_CLOUD);

	if (pcs.size() < 2)
	{
		m_app->dispToConsole("At least one cloud and a polyline are required", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	// Create log file
	time_t rawtime;
	time(&rawtime);

	struct tm* timeinfo = localtime(&rawtime);
	char buffer[80];
	strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", timeinfo);
	std::string str(buffer);

	string filename = "MSP_log_" + str + ".txt";
	ofstream auto_seg_log;
	auto_seg_log.open(filename);

	ccGLWindow* win = m_app->getActiveGLWindow();
	if (win)
	{
		win->setView(CC_FRONT_VIEW);
	}

	// Removing old contours and mortar maps from DB
	{
		ccHObject::Container contourfolder;
		root->filterChildren(contourfolder, true, CC_TYPES::HIERARCHY_OBJECT);
		for (ccHObject* folder : contourfolder)
		{
			if (folder && folder->getName().contains("Stone contours"))
			{
				m_app->removeFromDB(folder, false);
				break;
			}
		}

		ccHObject::Container oldMortarMaps;
		root->filterChildren(oldMortarMaps, true, CC_TYPES::POINT_CLOUD);
		for (ccHObject* folder : contourfolder)
		{
			if (folder && folder->getName().contains("Mortar Maps"))
			{
				m_app->removeFromDB(folder, false);
				break;
			}
		}
	}

	ccHObject::Container polys;
	root->filterChildren(polys, true, CC_TYPES::POLY_LINE);

	// Check loaded point clouds (there should be 2: base and segmented)
	ccPointCloud* cloudStone = nullptr;
	ccPointCloud* cloudMortar = nullptr;
	{
		int idxStone = -1;
		int idxMortar = -1;
		for (size_t i = 0; i < pcs.size(); ++i)
		{
			ccPointCloud* cl = static_cast<ccPointCloud*>(pcs.at(i));
			QString cloudName = cl->getName();

			if (!cloudName.contains("ref"))
			{
				if (idxStone < 0 && cloudName.contains("Stone"))
				{
					if (cl->getScalarFieldIndexByName("Stone Index") >= 0)
					{
						idxStone = i;
						m_app->dispToConsole("Stone cloud found!", ccMainAppInterface::STD_CONSOLE_MESSAGE);
						auto_seg_log << DateStamp() << " Stone cloud found" << endl;
					}
				}

				if (idxMortar < 0 && cloudName.contains("Mortar"))
				{
					idxMortar = i;
					m_app->dispToConsole("Mortar cloud found!", ccMainAppInterface::STD_CONSOLE_MESSAGE);
					auto_seg_log << DateStamp() << " Mortar cloud found" << endl;
				}
			}
		}

		if (idxMortar != -1 && idxStone != -1)
		{
			// Both clouds were found
			cloudStone = static_cast<ccPointCloud*>(pcs.at(idxStone));
			cloudMortar = static_cast<ccPointCloud*>(pcs.at(idxMortar));
		}
		else
		{
			if (idxMortar == -1 && idxStone == -1)
			{
				int cnt = 0;
				for (size_t i = 0; i < pcs.size(); ++i)
				{
					ccPointCloud* cl = static_cast<ccPointCloud*>(pcs.at(i));
					if (cl->size() > 100)
					{
						idxMortar = static_cast<int>(i);
						++cnt;
					}
				}

				if (cnt > 1)
				{
					m_app->dispToConsole("For manual segmentation leave only one cloud in DB", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
					return;
				}
				else if (cnt == 0)
				{
					m_app->dispToConsole("Clouds in DB are too small. Ignoring them", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
				}

				cloudMortar = static_cast<ccPointCloud*>(pcs.at(idxMortar));
			}
			else
			{
				m_app->dispToConsole("Only one cloud has been foud\nFor manual segmentation please change its name\nso that it doesn't contain the sequence 'stone' or 'mortar'", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
				return;
			}
		}
	}

	// Make the distinction between rectangles and non rectangular polylines
	std::vector<size_t> recIdx;
	std::vector<size_t> polyIdx;
	for (size_t i = 0; i < polys.size(); i++)
	{
		ccPolyline* poly = static_cast<ccPolyline*>(polys.at(i));
		if (poly->size() != 4)
		{
			polyIdx.push_back(i);
		}
		else
		{
			PointCoordinateType dx1 = poly->getPoint(1)->x - poly->getPoint(0)->x;
			PointCoordinateType dx2 = poly->getPoint(3)->x - poly->getPoint(2)->x;

			if (dx1 == dx2)
			{
				recIdx.push_back(i);
			}
			else
			{
				polyIdx.push_back(i);
			}
		}
	}

	// Creating vectors 
	vector< vector< pair<unsigned, int> > > stonePairs(2);
	vector< vector<unsigned> > mortarPoints(2);
	vector<unsigned> allStIdx;

	if (cloudMortar && cloudMortar->size() != 0)
	{
		mortarPoints[0].reserve(cloudMortar->size());
		for (unsigned i = 0; i < cloudMortar->size(); i++)
		{
			mortarPoints[0].push_back(i);
		}
	}

	int maxStoneId = 0;
	if (cloudStone)
	{
		int sfindex = cloudStone->getScalarFieldIndexByName("Stone Index");
		assert(sfindex != 0);
		CCCoreLib::ScalarField* stIdSF = cloudStone->getScalarField(sfindex);

		for (unsigned i = 0; i < cloudStone->size(); i++)
		{
			int index = static_cast<int>(stIdSF->getValue(i));
			stonePairs[0].push_back(make_pair(i, index));
			allStIdx.push_back(index);
		}

		sort(allStIdx.begin(), allStIdx.end());
		maxStoneId = static_cast<int>(allStIdx.back());

		auto_seg_log << DateStamp() << " Evaluating " << polyIdx.size() + recIdx.size() << " polylines..." << endl;

		// Rectangular polyline 
		if (recIdx.size() != 0)
		{
			auto_seg_log << DateStamp() << " Deleting incorrectly identified stones: " << recIdx.size() << "..." << endl;
			for (size_t polyIndex = 0; polyIndex < recIdx.size(); ++polyIndex)
			{
				auto_seg_log << DateStamp() << " Polyline " << polyIndex + 1 << "/" << recIdx.size() << endl;

				ccPolyline* poly = static_cast<ccPolyline*>(polys[polyIndex]);

				vector<unsigned> v = pointIdx(cloudStone, poly);
				vector<int> stIds = StIdFromPtId(v, stonePairs[0]);

				for (int stoneIndex : stIds)
				{
					for (size_t i = 0; i < stonePairs[0].size(); ++i)
					{
						if (stonePairs[0][i].second == stoneIndex)
						{
							stonePairs[0][i].second = -1;
							mortarPoints[1].push_back(stonePairs[0][i].first);
						}
					}
				}

				m_app->removeFromDB(poly, false);
			}
		}

		// Standard polylines
		if (polyIdx.size() != 0)
		{
			auto_seg_log << DateStamp() << " Correcting boundaries of stones (" << polyIdx.size() << ")..." << endl;
			for (size_t polyIndex = 0; polyIndex < polyIdx.size(); ++polyIndex)
			{
				auto_seg_log << DateStamp() << " Polyline " << polyIndex + 1 << "/" << polyIdx.size() << endl;

				ccPolyline* poly = static_cast<ccPolyline*>(polys[polyIndex]);
				vector<unsigned> vs = pointIdx(cloudStone, poly);
				vector<unsigned> vm;
				if (cloudMortar)
				{
					vm = pointIdx(cloudMortar, poly);
				}

				int k = 0;
				string kLabel;

				if (vm.empty() && !vs.empty())
				{
					// Values can be test for higher than zero just in case noise
					k = 1;
					kLabel = " (Removing stone)";
				}
				else if (vs.empty() && !vm.empty())
				{
					k = 2;
					kLabel = " (Adding new stone)";
				}
				else if (!vs.empty() && !vm.empty())
				{
					k = 3;
					kLabel = " (Correcting previously segmented stone)";
				}
				auto_seg_log << DateStamp() << " Case " << k << kLabel << endl;

				switch (k)
				{
				case 1:
				{
					vector<int> stId = StIdFromPtId(vs, stonePairs[0]);

					if (stId.size() == 1 && stId[0] == -1)
					{
						++maxStoneId;
						for (unsigned e : vs)
						{
							stonePairs[0][e].second = maxStoneId;
						}

						vector<unsigned> tempMortar = mortarPoints[1];
						vector<unsigned> resMortar;
						sort(tempMortar.begin(), tempMortar.end());
						std::set_difference(tempMortar.begin(), tempMortar.end(), vs.begin(), vs.end(), std::inserter(resMortar, resMortar.end()));
						mortarPoints[1] = resMortar;
					}
					else
					{
						vector<unsigned> vall = PtIdFromStId(stId, stonePairs[0]);

						vector<unsigned> vr;
						std::set_difference(vall.begin(), vall.end(), vs.begin(), vs.end(), std::inserter(vr, vr.end()));

						for (unsigned index : vr)
						{
							mortarPoints[1].push_back(static_cast<int>(index));
							stonePairs[0][index].second = -1;
						}
					}

					break;
				}

				case 2:
				{
					++maxStoneId;
					for (unsigned index : vm)
					{
						stonePairs[1].push_back(make_pair(index, maxStoneId));
						(mortarPoints[0])[index] = -1;
					}
					break;
				}

				case 3:
				{
					vector<int> stId = StIdFromPtId(vs, stonePairs[0]); //Extract stone indices from selection

					int minStoneId = -1;
					if (!stId.empty())
					{
						// Select minStoneId from indices
						minStoneId = *std::min_element(stId.begin(), stId.end());
					}

					if (minStoneId > 0) // If there is stone labelled as -1 inside (not belonging to any stone)
					{
						++maxStoneId;
						minStoneId = maxStoneId; // Create a new label (?). This should be done only if there is no other stone in the selection
					}

					for (unsigned e : vm) // For each point from mortar cloud in the selection
					{
						stonePairs[1].push_back(make_pair(e, minStoneId)); // Add to stone vector. Note that stonePairs[1] contains points from the stone cloud to be added as mortar
						mortarPoints[0][e] = -1; // Remove from mortar (idx to -1)
					}

					vector<unsigned> vall = PtIdFromStId(stId, stonePairs[0]); // All the points with idx similar to the ones in the selection
					for (unsigned e : vall)
					{
						stonePairs[0][e].second = minStoneId; // Relabel all those points with minId 
					}

					sort(vs.begin(), vs.end());
					sort(vall.begin(), vall.end());

					vector<unsigned> vr;
					std::set_difference(vall.begin(), vall.end(), vs.begin(), vs.end(), std::inserter(vr, vr.end())); //Difference between vall and vs > Extract to vr

					for (auto e : vr) //Label these as mortar. Some points can be duplicates (e.g. stone divided in several chunks and outliers assigned to mortar several times), so this is checked below
					{
						mortarPoints[1].push_back(e);
						stonePairs[0][e].second = -1; //remove from stone (label -1)
					}

					//Remove duplicates
					sort(mortarPoints[1].begin(), mortarPoints[1].end());
					mortarPoints[1].erase(unique(mortarPoints[1].begin(), mortarPoints[1].end()), mortarPoints[1].end());

					if (!stId.empty() && stId[0] == -1)
					{
						vector<unsigned> tempMortar = mortarPoints[1];
						sort(tempMortar.begin(), tempMortar.end());

						vector<unsigned> resMortar;
						std::set_difference(tempMortar.begin(), tempMortar.end(), vs.begin(), vs.end(), std::inserter(resMortar, resMortar.end()));

						mortarPoints[1] = resMortar;
					}
					break;
				}
				}

				m_app->removeFromDB(poly, false);
			}

			if (cloudStone)
				cloudStone->unallocateVisibilityArray();
			if (cloudMortar)
				cloudMortar->unallocateVisibilityArray();
		}
	}

	if (win)
	{
		win->redraw(true, true);
	}

	// Create Stone cloud 
	ccPointCloud* new_CloudStone = new ccPointCloud("Stone - cloud");
	CCCoreLib::ScalarField* new_stIdSF = nullptr;
	if (cloudStone)
	{
		// Adding points to the final Stone cloud
		vector<pair<unsigned, int>> rStPairs;

		// Removing points that are going to the mortar cloud (with index -1)
		rStPairs.reserve(stonePairs.size());
		for (auto e : stonePairs[0])
		{
			if (e.second != -1)
				rStPairs.push_back(e);
		}
		rStPairs.shrink_to_fit();

		if (!new_CloudStone->reserve(rStPairs.size()) || !new_CloudStone->reserveTheRGBTable())
		{
			delete new_CloudStone;
			ccLog::Error("Not enough memory");
			return;
		}

		for (size_t i = 0; i < rStPairs.size(); ++i)
		{
			unsigned stoneCloudIndex = rStPairs[i].first;
			new_CloudStone->addPoint(*cloudStone->getPoint(stoneCloudIndex));
			new_CloudStone->addColor(cloudStone->getPointColor(stoneCloudIndex));
		}

		// Adding points coming from mortar
		if (stonePairs[1].size() != 0 && cloudMortar)
		{
			if (!new_CloudStone->reserve(new_CloudStone->size() + static_cast<unsigned>(stonePairs[1].size())))
			{
				delete new_CloudStone;
				ccLog::Error("Not enough memory");
				return;
			}

			for (size_t i = 0; i < stonePairs[1].size(); ++i)
			{
				unsigned mortarPointIndex = stonePairs[1][i].first;
				new_CloudStone->addPoint(*cloudMortar->getPoint(mortarPointIndex));
				new_CloudStone->addColor(cloudMortar->getPointColor(mortarPointIndex));

			}
		}

		// Adding scalar field to the stone cloud
		{
			int nsfIdx = new_CloudStone->getScalarFieldIndexByName("Stone Index");
			if (nsfIdx < 0)
			{
				nsfIdx = new_CloudStone->addScalarField("Stone Index");
			}

			new_stIdSF = new_CloudStone->getScalarField(nsfIdx);
			if (new_stIdSF)
			{
				for (size_t i = 0; i < rStPairs.size(); ++i)
				{
					ScalarType value = static_cast<ScalarType>(rStPairs[i].second);
					new_stIdSF->setValue(static_cast<unsigned>(i), value);
				}

				// Adding points coming from mortar
				for (size_t i = 0; i < stonePairs[1].size(); ++i)
				{
					ScalarType value = static_cast<ScalarType>(stonePairs[1][i].second);
					new_stIdSF->setValue(static_cast<unsigned>(rStPairs.size() + i), value);
				}

				new_stIdSF->computeMinAndMax();
				new_CloudStone->setCurrentDisplayedScalarField(nsfIdx);
				new_CloudStone->showSF(true);
			}
			else
			{
				ccLog::Warning("Failed to create stone index SF");
			}
		}

		new_CloudStone->setPointSize(cloudStone->getPointSize());
		m_app->addToDB(new_CloudStone, true, true, false, true);
	}

	// Create Motrar cloud 
	ccPointCloud* new_CloudMortar = new ccPointCloud("Mortar - cloud");
	if (cloudMortar)
	{
		if (!new_CloudMortar->reserve(mortarPoints.size()) || !new_CloudMortar->reserveTheRGBTable())
		{
			delete new_CloudMortar;
			ccLog::Error("Not enough memory");
			return;
		}

		new_CloudMortar->showColors(true);

		for (auto index : mortarPoints[0])
		{
			if (index != -1)
			{
				new_CloudMortar->addPoint(*cloudMortar->getPoint(index));
				new_CloudMortar->addColor(cloudMortar->getPointColor(index));
			}
		}

		// Points coming from stone cloud
		if (mortarPoints[1].size() != 0)
		{
			if (!new_CloudMortar->reserve(new_CloudMortar->size() + static_cast<unsigned>(mortarPoints[1].size())))
			{
				delete new_CloudMortar;
				ccLog::Error("Not enough memory");
				return;
			}

			for (auto index : mortarPoints[1])
			{
				if (index != -1)
				{
					new_CloudMortar->addPoint(*cloudMortar->getPoint(index));
					new_CloudMortar->addColor(cloudMortar->getPointColor(index));
				}
			}
		}

		new_CloudMortar->setPointSize(cloudMortar->getPointSize());
		m_app->addToDB(new_CloudMortar, true, true, false, true);
	}

	// Mortar maps
	{
		auto_seg_log << DateStamp() << " Starting mortar maps generation..." << endl;

		// Moving the clouds to center in case they were not 
		CCVector3 minBox0, maxBox0;
		if (cloudMortar)
		{
			cloudMortar->getBoundingBox(minBox0, maxBox0);

			new_CloudStone->translate(-minBox0);
			new_CloudMortar->translate(-minBox0);
		}

		ccPointCloud* mortarMaps = GetMortarMaps(new_CloudStone, new_CloudMortar);

		// Move all the clouds back
		if (cloudMortar)
		{
			new_CloudStone->translate(minBox0);
			new_CloudMortar->translate(minBox0);
			if (mortarMaps)
			{
				mortarMaps->translate(minBox0);
			}
		}

		if (mortarMaps)
		{
			m_app->addToDB(mortarMaps, true, true, false, true);
			auto_seg_log << DateStamp() << " Mortar maps OK" << endl;
		}
	}

	// Creating new vectors 
	if (new_stIdSF && new_stIdSF->size() > 0)
	{
		vector<pair<unsigned, int>> nstonePairs;
		nstonePairs.reserve(new_stIdSF->size());

		vector<unsigned> nallStIdx;
		nallStIdx.reserve(new_stIdSF->size());
		for (unsigned i = 0; i < new_stIdSF->size(); i++)
		{
			nstonePairs.push_back(make_pair(i, new_stIdSF->getValue(i)));
			nallStIdx.push_back(static_cast<unsigned>(new_stIdSF->getValue(i)));
		}
		sort(nallStIdx.begin(), nallStIdx.end());
		nallStIdx.erase(unique(nallStIdx.begin(), nallStIdx.end()), nallStIdx.end());

		sort(nstonePairs.begin(), nstonePairs.end(), [](auto &left, auto &right) {return left.second < right.second; });

		// Correcting indexes
		int old_idx = nstonePairs[0].second;
		int new_idx = 0;

		vector<vector<int>> vecStones;
		vecStones.reserve(nstonePairs.size());
		for (size_t i = 0; i < nstonePairs.size(); i++)
		{
			if (nstonePairs[i].second == old_idx)
			{
				nstonePairs[i].second = new_idx;
				vecStones[new_idx].push_back(nstonePairs[i].first);
			}
			else
			{
				++new_idx;
				old_idx = nstonePairs[i].second;
				nstonePairs[i].second = new_idx;
				vecStones[new_idx].push_back(nstonePairs[i].first);
			}
		}

		sort(nstonePairs.begin(), nstonePairs.end(), [](auto &left, auto &right) {return left.first < right.first; });

		// Shuffling scalar field for better visuals
		vector<int> stIdShuffle;
		stIdShuffle.reserve(nstonePairs.size());
		for (const auto& p : nstonePairs)
		{
			stIdShuffle.push_back(p.second);
		}
		sort(stIdShuffle.begin(), stIdShuffle.end());

		stIdShuffle.erase(unique(stIdShuffle.begin(), stIdShuffle.end()), stIdShuffle.end());

		auto rng = std::default_random_engine{};
		std::shuffle(stIdShuffle.begin(), stIdShuffle.end(), rng);

		for (unsigned i = 0; i < new_stIdSF->size(); i++)
		{
			ScalarType value = static_cast<ScalarType>(stIdShuffle[nstonePairs[i].second]);
			new_stIdSF->setValue(i, value);
		}

		// contours in an alternative way using stone indexes and original cloud
		auto_seg_log << DateStamp() << " Starting contours regeneration" << endl;

		//generating contour polylines
		ccHObject* nContoursContainer = new ccHObject("Stone contours");

		size_t polylineIndex = 0;
		for (const auto& ve : vecStones)
		{
			if (ve.size() != 0)
			{
				auto_seg_log << DateStamp() << " Polyline " << polylineIndex + 1 << "/" << vecStones.size() << endl;
				ccPolyline* contour = ContourPoly2(new_CloudStone, ve, QString("stone %1").arg(polylineIndex));
				if (contour)
				{
					nContoursContainer->addChild(contour);
					++polylineIndex;
				}
			}
		}

		auto_seg_log << DateStamp() << " Contours regeneration done" << endl;

		if (nContoursContainer->getChildrenNumber() != 0)
		{
			m_app->addToDB(nContoursContainer, true, true, false, true);
		}
		else
		{
			delete nContoursContainer;
		}
	}

	if (win)
	{
		win->setView(CC_FRONT_VIEW);
	}
}
