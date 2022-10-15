#pragma once

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
	QList<QAction *> getActions() override;

private:

	void doAction();

	//! Returns indexes of points inside a polyline from a point cloud
	std::vector<unsigned> pointIdx(ccPointCloud* cloud, ccPolyline* poly) const;
	//! Returns stone indexes given points idexes 
	static std::vector<int> StIdFromPtId(const std::vector<unsigned>& pts, const std::vector<std::pair<unsigned, int>>& pairs);
	//! Returns returns points indexes given stone indexes
	static std::vector<unsigned> PtIdFromStId(const std::vector<int>& pts, const std::vector<std::pair<unsigned, int>>& pairs);

	QAction* m_action;
};
