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

#include <kukadu/vision/arlocalizer/rpp_types.h>

#ifndef __LINK_WITH_RPP__
#define __LINK_WITH_RPP__

namespace kukadu {

typedef double rpp_float;
typedef double rpp_vec[3];
typedef double rpp_mat[3][3];


void 
robustPlanarPose(rpp_float &err,
				 rpp_mat &R,
				 rpp_vec &t,
				 const rpp_float cc[2],
				 const rpp_float fc[2],
				 const rpp_vec *model,
				 const rpp_vec *iprts,
				 const unsigned int model_iprts_size,
				 const rpp_mat R_init,
				 const bool estimate_R_init,
				 const rpp_float epsilon,
				 const rpp_float tolerance,
				 const unsigned int max_iterations);


bool rppSupportAvailable();

void Quaternion_byAngleAndVector(quat_t &Q, const real_t &q_angle, const vec3_t &q_vector);
void GetRotationbyVector(mat33_t &R, const vec3_t &v1, const vec3_t &v2);
void xform(vec3_array &Q, const vec3_array &P, const mat33_t &R, const vec3_t &t);
void xformproj(vec3_array &Qp, const vec3_array &P, const mat33_t &R, const vec3_t &t);
void rpyMat(mat33_t &R, const vec3_t &rpy);
void rpyAng(vec3_t &angs, const mat33_t &R);
void rpyAng_X(vec3_t &ang_zyx, const mat33_t &R);
void decomposeR(mat33_t &Rz, const mat33_t &R);
void abskernel(mat33_t &R, vec3_t &t, vec3_array &Qout, real_t &err2,
               const vec3_array _P, const vec3_array _Q,
               const mat33_array F, const mat33_t G);
void objpose(mat33_t &R, vec3_t &t, unsigned int &it, real_t &obj_err, real_t &img_err,
             bool calc_img_err, const vec3_array _P, const vec3_array Qp, const options_t options);
void getRotationY_wrtT(scalar_array &al_ret, vec3_array &tnew, const vec3_array &v,
                       const vec3_array &p, const vec3_t &t, const real_t &DB,
                       const mat33_t &Rz);
void getRfor2ndPose_V_Exact(pose_vec &sol, const vec3_array &v, const vec3_array &P,
                            const mat33_t R, const vec3_t t, const real_t DB);
void get2ndPose_Exact(pose_vec &sol, const vec3_array &v, const vec3_array &P,
                      const mat33_t R, const vec3_t t, const real_t DB);

// ------------------------------------------------------------------------------------------
void robust_pose(real_t &err, mat33_t &R, vec3_t &t,
                 const vec3_array &_model, const vec3_array &_iprts,
                 const options_t _options);

}


#endif // __LINK_WITH_RPP__
