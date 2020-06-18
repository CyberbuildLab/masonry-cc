#ifndef Q_AUTOSEG_HEADER
#define Q_AUTOSEG_HEADER

//##########################################################################
//#                                                                        #
//#                CLOUDCOMPARE PLUGIN: qAutoSeg                          #
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

class ccAutoSeg : public QObject, public ccStdPluginInterface
{
	Q_OBJECT
	Q_INTERFACES( ccPluginInterface ccStdPluginInterface )
	
	Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.qAutoSeg" FILE "info.json")
	
public:
	explicit ccAutoSeg( QObject *parent = nullptr );
	//~ccAutoSeg() override = default;
	virtual ~ccAutoSeg() = default;
	
	// inherited from ccStdPluginInterface
	void onNewSelection( const ccHObject::Container &selectedEntities ) override;
	QList<QAction *> getActions() override;
	
private:

	void doAction();

	
	QAction* m_action;

};

#endif
