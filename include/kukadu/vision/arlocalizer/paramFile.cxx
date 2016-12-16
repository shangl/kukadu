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
 * $Id: paramFile.cxx 162 2006-04-19 21:28:10Z grabner $
 * @file
 * ======================================================================== */

#include <stdio.h>
#include <stdarg.h>
#include <kukadu/vision/arlocalizer/param.h>
#include <kukadu/vision/arlocalizer/Tracker.h>
#include <kukadu/vision/arlocalizer/byteSwap.h>

namespace kukadu {

AR_TEMPL_FUNC int
AR_TEMPL_TRACKER::arParamSaveDouble(char *filename, int num, ARParamDouble *param, ...)
{
    FILE           *fp;
    va_list        ap;
    ARParamDouble  *param1;
    int            i;

    if( num < 1 ) return -1;

    fp = fopen( filename, "wb" );
    if( fp == NULL ) return -1;

#ifdef AR_LITTLE_ENDIAN
    byteswap( param );
#endif
    if( fwrite( param, sizeof(ARParamDouble), 1, fp ) != 1 ) {
        fclose(fp);
#ifdef AR_LITTLE_ENDIAN
        byteswap( param );
#endif
        return -1;
    }
#ifdef AR_LITTLE_ENDIAN
    byteswap( param );
#endif

    va_start(ap, param);
    for( i = 1; i < num; i++ ) {
        param1 = va_arg(ap, ARParamDouble*);
#ifdef AR_LITTLE_ENDIAN
        byteswap( param1 );
#endif
        if( fwrite( param1, sizeof(ARParamDouble), 1, fp ) != 1 ) {
            fclose(fp);
#ifdef AR_LITTLE_ENDIAN
            byteswap( param1 );
#endif
            return -1;
        }
#ifdef AR_LITTLE_ENDIAN
        byteswap( param1 );
#endif
    }

    fclose(fp);

    return 0;
}

AR_TEMPL_FUNC int
AR_TEMPL_TRACKER::arParamLoadDouble( char *filename, int num, ARParamDouble *param, ...)
{
    FILE           *fp;
    va_list        ap;
    ARParamDouble  *param1;
    int            i;

    if( num < 1 ) return -1;
	
    fp = fopen( filename, "rb" );
    if( fp == NULL ) return -1;

    if( fread( param, sizeof(ARParamDouble), 1, fp ) != 1 ) {
        fclose(fp);
        return -1;
    }
#ifdef AR_LITTLE_ENDIAN
    byteswap( param );
#endif

    va_start(ap, param);
    for( i = 1; i < num; i++ ) {
        param1 = va_arg(ap, ARParamDouble*);
        if( fread( param1, sizeof(ARParamDouble), 1, fp ) != 1 ) {
            fclose(fp);
            return -1;
        }
#ifdef AR_LITTLE_ENDIAN
        byteswap( param1 );
#endif
    }

    fclose(fp);

    return 0;
}

}
