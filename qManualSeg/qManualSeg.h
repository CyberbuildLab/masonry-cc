#ifndef Q_MANUALSEG_HEADER
#define Q_MANUALSEG_HEADER

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

#include "ccStdPluginInterface.h"

class ccManualSeg : public QObject, public ccStdPluginInterface
{
	Q_OBJECT
	Q_INTERFACES( ccPluginInterface ccStdPluginInterface )
	
	Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.qManualSeg" FILE "info.json")
	
public:
	explicit ccManualSeg( QObject *parent = nullptr );
	virtual ~ccManualSeg() = default;
	
	// inherited from ccStdPluginInterface
	void onNewSelection( const ccHObject::Container &selectedEntities ) override;
	QList<QAction *> getActions() override;


private:

	void doAction();
	void doAction2();

	std::vector<int> pointIdx(ccPointCloud* cloud, ccPolyline* poly);
	std::vector<int> stIdFromPtId(std::vector<int> pts, std::vector<std::pair<int, int>> pairs);
	std::vector<int> ptIdFromStId(std::vector<int> pts , std::vector<std::pair<int,int>> pairs);


	QAction* m_action;
	

protected:

	//! Segmentation polyline
	ccPolyline* m_segmentationPoly;
	//! Segmentation polyline vertices
	ccPointCloud* m_polyVertices;

};

#endif
