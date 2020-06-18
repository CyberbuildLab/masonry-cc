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
//#								 v1.2                                      #
//#			                                                               #
//#                                                                        #
//##########################################################################

#include "profileImportDlg.h"

//Qt
#include <QFileDialog>

//System
#include <assert.h>

static double s_joints = 4; //Estimated mortar joints width in cm

static double s_horizontal = 3; //Segmentation window in metres (X axis)
static double s_vertical = 2; //Segmentation window in metres (Z axis)


ProfileImportDlg::ProfileImportDlg(QWidget* parent)
	: QDialog(parent, Qt::Tool)
	, Ui::ProfileImportDlg()
{
	setupUi(this);

	connect(buttonBox, SIGNAL(accepted()), this, SLOT(saveSettings));
	jointsSpinBox->setValue(s_joints);
	segmentHSpinBox->setValue(s_horizontal);
	segmentVSpinBox->setValue(s_vertical);
	alignmentCheckBox->setChecked(true);
	alignmentCheckBox->setDisabled(true);
	this->setWindowTitle("Automatic Segmentation plugin");

}

void ProfileImportDlg::saveSettings() {
	s_joints = jointsSpinBox->value();
	s_horizontal = segmentHSpinBox->value();
	s_vertical = segmentVSpinBox->value();
}

