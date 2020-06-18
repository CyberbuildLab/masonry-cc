//##########################################################################
//#                                                                        #
//#                      CLOUDCOMPARE PLUGIN: qManualSeg                   #
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
//#								 v1.2                                      #
//##########################################################################

#ifndef QMANUAL_SEG_PROFILE_IMPORT_DLG_HEADER
#define QMANUAL_SEG_PROFILE_IMPORT_DLG_HEADER

#include "ui_profileImportDlg.h"

class ProfileImportDlg : public QDialog, public Ui::ProfileImportDlg
{
	Q_OBJECT

public:

	//! Default constructor
    explicit ProfileImportDlg(QWidget* parent = 0);

	int getAxisDimension() const;

	//! Sets default filename
	void setDefaultFilename(QString filename);

	//! Returns input filename (on completion)
	QString getFilename() const;

	//! Returns whether the profile heights are absolute or not (i.e. relative to the center)
	bool absoluteHeightValues() const;

protected slots:

	//! Called when the 'browse' tool button is pressed
	void browseFile();

};

#endif 
