/* ========================================================================
* PROJECT: ARToolKitPlus
* ========================================================================
* This work is based on the original ARToolKit developed by
*   Hirokazu Kato
*   Mark Billinghurst
*   HITLab, University of Washington, Seattle
* http://www.hitl.washington.edu/artoolkit/
*
* Copyright of the derived and new portions of this work
*     (C) 2006 Graz University of Technology
*
* This framework is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This framework is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this framework; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*
* For further information please contact 
*   Dieter Schmalstieg
*   <schmalstieg@icg.tu-graz.ac.at>
*   Graz University of Technology, 
*   Institut for Computer Graphics and Vision,
*   Inffeldgasse 16a, 8010 Graz, Austria.
* ========================================================================
** @author   Thomas Pintaric
*
* $Id$
* @file
* ======================================================================== */

#ifndef __ARTOOLKIT_CAMERAADVIMPL_HEADERFILE__
#define __ARTOOLKIT_CAMERAADVIMPL_HEADERFILE__

#include <kukadu/vision/arlocalizer/Camera.h>

namespace kukadu {

#define CAMERA_ADV_HEADER "ARToolKitPlus_CamCal_Rev02"
#define CAMERA_ADV_MAX_UNDIST_ITERATIONS 20

class CameraAdvImpl : public Camera
{
public:

	CameraAdvImpl();
	virtual ~CameraAdvImpl();

	virtual void observ2Ideal(ARFloat ox, ARFloat oy, ARFloat *ix, ARFloat *iy);
	virtual void ideal2Observ(ARFloat ix, ARFloat iy, ARFloat *ox, ARFloat *oy);
	virtual bool loadFromFile(const char* filename);
	virtual Camera* clone();
	virtual bool changeFrameSize(const int frameWidth, const int frameHeight);
	virtual void logSettings(Logger* p_log);

protected:

	ARFloat cc[2];
	ARFloat fc[2];
	ARFloat kc[6];
//  http://www.vision.caltech.edu/bouguetj/calib_doc/htmls/parameters.html

	int undist_iterations;
};

}

#endif // __ARTOOLKIT_CAMERAADVIMPL_HEADERFILE__
