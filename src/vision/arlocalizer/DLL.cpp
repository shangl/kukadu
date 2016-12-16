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
 ** @author   Daniel Wagner
 *
 * $Id: DLL.cpp 162 2006-04-19 21:28:10Z grabner $
 * @file
 * ======================================================================= */


#include <kukadu/vision/arlocalizer/TrackerSingleMarkerImpl.h>

namespace kukadu
{


#ifdef ARTOOLKITPLUS_DLL

ARTOOLKITPLUS_API TrackerSingleMarker*
createTrackerSingleMarker(int nWidth, int nHeight,
						  int nMarkerSizeX, int nMarkerSizeY, int nMarkerSampleNum,
						  PIXEL_FORMAT nPixelFormat)
{
	if(nMarkerSizeX==6 && nMarkerSizeY==6 && nMarkerSampleNum==6)
	{
		typedef TrackerSingleMarkerImpl<6,6,6, 3, 10> TrackerSingleMarker_6_6_6_3_10;

		if(memManager && !memManager->didInit())
			memManager->init(TrackerSingleMarker_6_6_6_3_10::getMemoryRequirements());

		TrackerSingleMarker* tracker = new TrackerSingleMarker_6_6_6_3_10(nWidth, nHeight);
		tracker->setPixelFormat(nPixelFormat);
		return tracker;
	}

	return NULL;
}


#endif

}


