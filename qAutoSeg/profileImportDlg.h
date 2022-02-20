//##########################################################################
//#                                                                        #
//#                      CLOUDCOMPARE PLUGIN: qAutoSeg                     #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#                         Cyberbuild,2020                                #
//#								 v1.2                                      #                                                                        #
//##########################################################################

#ifndef QAUTOSEG_PROFILE_IMPORT_DLG_HEADER
#define QAUTOSEG_PROFILE_IMPORT_DLG_HEADER

#include "ui_profileImportDlg.h"

//! Dialog for setting the input parameters required for the automatic segmentation of masonry walls
class ProfileImportDlg : public QDialog, public Ui::ProfileImportDlg
{
	Q_OBJECT

public:

	//! Default constructor
    explicit ProfileImportDlg(QWidget* parent = nullptr);


protected slots:

	//! Save settings
	void saveSettings();
};

#endif 
