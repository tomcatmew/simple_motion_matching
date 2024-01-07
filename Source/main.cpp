/*
 * Copyright (c) 2019 Yifei Chen and Prof.Umetani (delfem2 library)
 * 
 */
#include <chrono>
#include <filesystem>
#include <iostream>
#include <vector>
#include <algorithm>
#define GL_SILENCE_DEPRECATION
#include <GLFW/glfw3.h>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl2.h"


#include "../traj.h"
#include "../model.h"
#include "../gendataset.h"
#include "../draw.h"
#include "delfem2/rig_geo3.h"
#include "delfem2/glfw/viewer3.h"
#include "delfem2/glfw/util.h"
#include "delfem2/opengl/old/funcs.h"
#include "delfem2/opengl/old/rigv3.h"
#include "delfem2/rig_bvh.h"

//#define DRAWDEBUG

//   bvh loader, parser and model loaders
namespace dfm2 = delfem2;


void joystick_callback(int jid, int event)
{
    if (event == GLFW_CONNECTED)
    {
        std::cout << "Controller Connected" << std::endl;
        // The joystick was connected
    }
    else if (event == GLFW_DISCONNECTED)
    {
        std::cout << "Controller Disconnected" << std::endl;
        std::cout << "Recommend connecting a controller for play" << std::endl;
       
        // The joystick was disconnected
    }
}


void DrawFloorShadow2(
    const std::vector< std::vector<delfem2::CRigBone> >& vec_vec_rig_bone,
    const Floor& floor,
    float yfloor) {
    namespace lcl = delfem2::opengl::old::funcs;
    GLboolean is_lighting = ::glIsEnabled(GL_LIGHTING);
    GLboolean is_blend = ::glIsEnabled(GL_BLEND);
    {  // draw floor
        ::glClearStencil(0);
        { // draw floor (stencil 1)
            glEnable(GL_STENCIL_TEST);
            glStencilFunc(GL_ALWAYS, 1, static_cast<GLuint>(~0));
            glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
            floor.draw_checkerboard();
        }
        for (const auto& vec_rig_bone : vec_vec_rig_bone) { // draw stensil
            glColorMask(0, 0, 0, 0);
            glDepthMask(0);
            glEnable(GL_STENCIL_TEST);
            glStencilFunc(GL_EQUAL, 1, static_cast<GLuint>(~0));
            glStencilOp(GL_KEEP, GL_KEEP, GL_INCR);
            glPushMatrix();
            {
                float plane[4] = { 0, 1, 0, -yfloor - 0.001f };
                float lpos[4] = { 0, 100, 0, 1 };
                float m_shadow[16];
                lcl::ShadowMatrix(m_shadow, plane, lpos);
                glMultMatrixf(m_shadow);
            }
            delfem2::opengl::DrawBone_Octahedron(
                vec_rig_bone,
                5, -1,
                0.1, 1.0);
            glPopMatrix();
            glColorMask(1, 1, 1, 1);
            glDepthMask(1);
        }
        { // draw shadow
            glStencilFunc(GL_EQUAL, 2, static_cast<GLuint>(~0));
            glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            ::glDisable(GL_DEPTH_TEST);
            ::glDisable(GL_LIGHTING);
            glColor4f(0.1f, 0.1f, 0.1f, 0.5f);
            floor.draw_geometry();
            glEnable(GL_DEPTH_TEST);
            glDisable(GL_BLEND);
            glDisable(GL_STENCIL_TEST);
        }
    }
    if (is_lighting) { ::glEnable(GL_LIGHTING); }
    if (is_blend) { ::glEnable(GL_BLEND); }
}



template<int ndimin_pose, int ndimout_pose, int nOctave>
bool Hoge_tomcat(
    std::vector<dfm2::CRigBone>& aBone,
    std::vector<double>& vec_input_pose,
    MLP_Fourier<ndimin_pose, nOctave>& model_pose,
    const dfm2::CVec2d& root_pos2,
    const std::vector<dfm2::CVec2d>& vec_pos2,
    const dfm2::CVec2d& vec_dirz,
    const double& vec_phase,
    const std::vector<unsigned int>& vec_id_bones_used) {
    //const unsigned int nframe = vec_phase.size();
    vec_input_pose.clear();
    {
        const dfm2::CVec2d p0 = root_pos2;
        const dfm2::CVec2d dz0 = vec_dirz;  // local z coord
        assert(fabs(dz0.norm() - 1) < 1.0e-5);
        const dfm2::CVec2d dx0(dz0.y, -dz0.x); // local x coord

        vec_input_pose.push_back(vec_phase);
        for (unsigned int ismpl = 0; ismpl < 10; ismpl++) {
            vec_input_pose.push_back(dx0.dot(vec_pos2[ismpl] - p0));
            vec_input_pose.push_back(dz0.dot(vec_pos2[ismpl] - p0));
        }
    }
    if (vec_phase < -0.5) { return false; }
    assert(vec_input_pose.size() == ndimin_pose);
    // ------------------------
    std::vector<double> vec_output_pose;
    {
        std::vector<float> aInF(vec_input_pose.begin(), vec_input_pose.end());
        model_pose.predict(vec_output_pose, aInF);
    }
    {
        const dfm2::CVec2d p0 = root_pos2;
        const dfm2::CVec2d dz0 = vec_dirz;  // local z coord
        const dfm2::CVec2d dx0(dz0.y, -dz0.x); // local x coord
        const dfm2::CMat3d R0(
            dx0.x, 0, dz0.x,
            0, 1, 0,
            dx0.y, 0, dz0.y);
        dfm2::CQuatd q0y;
        //R0.GetQuat_RotMatrix(q0y.p);
        q0y = R0.GetQuaternion();
        Encoder_Pose::decode(aBone, vec_output_pose, vec_id_bones_used, q0y, p0);
    }
    return true;
}

dfm2::CQuatd qfv(dfm2::CVec3d& u, dfm2::CVec3d& v)
{
    // a concise version of q from 2 vectors
    u.normalize();
    v.normalize();
    double k = 1.0 + u.dot(v);
    double s = 1 / sqrt(k + k);
    dfm2::CVec3d cross = u.cross(v);
    cross = s * cross;
    auto angle = 2 * acos(k * s);
    if (angle < 0.001f)
    {
        return dfm2::CQuatd::Identity();
    }
    dfm2::CQuatd out = dfm2::CQuatd(k * s, cross.x, cross.y, cross.z);
    out.normalize();
    out.SetSmallerRotation();
    return out;
}

void forwardKinematic(std::vector<dfm2::CRigBone>& aBone)
{
    for (std::size_t ibone = 0; ibone < aBone.size(); ++ibone) {
        const int ibone_p = aBone[ibone].ibone_parent;
        if (ibone_p < 0 || ibone_p >= (int)aBone.size()) { // root bone
            aBone[ibone].transRelative[0] = aBone[ibone].affmat3Global[3];
            aBone[ibone].transRelative[1] = aBone[ibone].affmat3Global[7];
            aBone[ibone].transRelative[2] = aBone[ibone].affmat3Global[11];
            dfm2::CMat3d matLocal3 = dfm2::CMat3d(aBone[ibone].affmat3Global[0], aBone[ibone].affmat3Global[1], aBone[ibone].affmat3Global[2],
                aBone[ibone].affmat3Global[4], aBone[ibone].affmat3Global[5], aBone[ibone].affmat3Global[6],
                aBone[ibone].affmat3Global[8], aBone[ibone].affmat3Global[9], aBone[ibone].affmat3Global[10]);
            dfm2::CQuatd bone_q = matLocal3.GetQuaternion();
            aBone[ibone].quatRelativeRot[0] = bone_q.x;
            aBone[ibone].quatRelativeRot[1] = bone_q.y;
            aBone[ibone].quatRelativeRot[2] = bone_q.z;
            aBone[ibone].quatRelativeRot[3] = bone_q.w;
            continue;
        }
        // B = A^(-1) * C
        // https://mathworld.wolfram.com/MatrixInverse.html
        assert(ibone_p < (int)ibone);
        double invAffmat[16];  dfm2::Inverse_Mat4(invAffmat, aBone[ibone_p].affmat3Global);
        double localAff[16];
        dfm2::MatMat4(localAff, invAffmat, aBone[ibone].affmat3Global);
        //aBone[ibone].transRelative[0] = localAff[3];
        //aBone[ibone].transRelative[1] = localAff[7];
        //aBone[ibone].transRelative[2] = localAff[11];
        dfm2::CMat3d matLocal3 = dfm2::CMat3d(localAff[0], localAff[1], localAff[2],
            localAff[4], localAff[5], localAff[6],
            localAff[8], localAff[9], localAff[10]);
        dfm2::CQuatd bone_q = matLocal3.GetQuaternion();
        aBone[ibone].quatRelativeRot[0] = bone_q.x;
        aBone[ibone].quatRelativeRot[1] = bone_q.y;
        aBone[ibone].quatRelativeRot[2] = bone_q.z;
        aBone[ibone].quatRelativeRot[3] = bone_q.w;
    }
}
void applyFK(std::vector<dfm2::CRigBone>& aBone)
{
    forwardKinematic(aBone);
    UpdateBoneRotTrans(aBone);
}

void backward(std::vector<dfm2::CVec3d>& joints, std::vector<double>& length, dfm2::CVec3d target)
{
    joints[joints.size() - 1].x = target.x;
    joints[joints.size() - 1].y = target.y;
    joints[joints.size() - 1].z = target.z;
    for (int i = joints.size() - 2; i >= 0; i--)
    {
        dfm2::CVec3d dir_back = joints[i] - joints[i + 1];
        dir_back.normalize();
        dfm2::CVec3d new_pos = joints[i + 1] + dir_back * length[i + 1];
        joints[i].x = new_pos.x;
        joints[i].y = new_pos.y;
        joints[i].z = new_pos.z;
    }
}


void forward(std::vector<dfm2::CVec3d>& joints, std::vector<double>& length, dfm2::CVec3d origin)
{
    joints[0].x = origin.x;
    joints[0].y = origin.y;
    joints[0].z = origin.z;
    for (unsigned int i = 1; i <= joints.size() - 1; i++)
    {
        dfm2::CVec3d dir_foward = joints[i] - joints[i - 1];
        dir_foward.normalize();
        dfm2::CVec3d new_pos = joints[i - 1] + dir_foward * length[i];
        joints[i].x = new_pos.x;
        joints[i].y = new_pos.y;
        joints[i].z = new_pos.z;
    }
}


void FABRIK(std::vector<dfm2::CRigBone>& aBone, dfm2::CVec3d target)
{
    std::vector<dfm2::CVec3d> start_pos;
    std::vector<dfm2::CVec3d> joints;
    std::vector<double> length;
    dfm2::CVec3d origin(aBone[2].affmat3Global[3], aBone[2].affmat3Global[7], aBone[2].affmat3Global[11]);
    for (int i = 2; i < 5; i++)
    {
        dfm2::CVec3d jointPost = dfm2::CVec3d(aBone[i].affmat3Global[3], aBone[i].affmat3Global[7], aBone[i].affmat3Global[11]);
        joints.push_back(jointPost);
        start_pos.push_back(jointPost);
        dfm2::CVec3d p0(aBone[i-1].invBindMat[3], aBone[i-1].invBindMat[7], aBone[i-1].invBindMat[11]);
        dfm2::CVec3d p1(aBone[i].invBindMat[3], aBone[i].invBindMat[7], aBone[i].invBindMat[11]);
        double boneLength = dfm2::Distance(p0,p1);
        length.push_back(boneLength);
    }

    for (unsigned int j = 0; j < 2; j++)
    {
        backward(joints,length, target);
        forward(joints,length, origin);
    }

    std::vector<dfm2::CVec3d> end_pos;
    for (unsigned int i = 0; i < joints.size(); i++)
    {
        dfm2::CVec3d pend = joints[i];
        end_pos.push_back(pend);
    }

    //reset 
    //for (unsigned int i = 0; i < start_pos.size(); i++)
    //{
    //    joints[i].move(start_pos[i].x, start_pos[i].y, start_pos[i].z);
    //}

    //Forward Kinematic
    for (unsigned int i = 1; i < joints.size(); i++)
    {
        dfm2::CQuatd r_q;
        r_q = qfv(start_pos[i] - end_pos[i - 1], end_pos[i] - end_pos[i - 1]);
        //r_q.SetSmallerRotation();
        r_q.normalize();
        int bonemap[3] = { 2,3,4 };
        dfm2::CQuatd bone_q(aBone[bonemap[i - 1]].quatRelativeRot[3], aBone[bonemap[i - 1]].quatRelativeRot[0], aBone[bonemap[i - 1]].quatRelativeRot[1], aBone[bonemap[i - 1]].quatRelativeRot[2]);
        dfm2::CQuatd q_current = r_q * bone_q;
        aBone[bonemap[i - 1]].quatRelativeRot[0] = q_current.x;
        aBone[bonemap[i - 1]].quatRelativeRot[1] = q_current.y;
        aBone[bonemap[i - 1]].quatRelativeRot[2] = q_current.z;
        aBone[bonemap[i - 1]].quatRelativeRot[3] = q_current.w;
    }
    UpdateBoneRotTrans(aBone);
}

void solveik(std::vector<dfm2::CRigBone>& aBone, dfm2::CVec3d target, int start_bone, int end_bone)
{
    UpdateBoneRotTrans(aBone);
    int num_iter = 3;
    for (unsigned int iter = 0; iter < num_iter; ++iter)
    {
        for (unsigned int i = start_bone; i >= end_bone; i--)
        {
            dfm2::CVec3d jointnow(aBone[i].RootPosition());
            dfm2::CVec3d end(aBone[start_bone + 1].RootPosition());
            dfm2::CVec3d endeff_dir = end - jointnow;
            dfm2::CVec3d target_dir = target - jointnow;
            dfm2::CQuatd r_q;
            r_q = qfv(endeff_dir, target_dir);
            //dfm2::CQuatd bone_parent(aBone[i-1].quatRelativeRot[3], aBone[i - 1].quatRelativeRot[0], aBone[i - 1].quatRelativeRot[1], aBone[i - 1].quatRelativeRot[2]);
            //dfm2::CQuatd bone_q(aBone[i].quatRelativeRot[3], aBone[i].quatRelativeRot[0], aBone[i].quatRelativeRot[1], aBone[i].quatRelativeRot[2]);
            
            //https://alogicalmind.com/res/ik_ccd/paper.pdf
            dfm2::CMat3d matLocal3 = dfm2::CMat3d(aBone[i-1].affmat3Global[0], aBone[i - 1].affmat3Global[1], aBone[i - 1].affmat3Global[2],
                aBone[i - 1].affmat3Global[4], aBone[i - 1].affmat3Global[5], aBone[i - 1].affmat3Global[6],
                aBone[i - 1].affmat3Global[8], aBone[i - 1].affmat3Global[9], aBone[i - 1].affmat3Global[10]);
            dfm2::CQuatd bone_parent = matLocal3.GetQuaternion();
            
            dfm2::CMat3d matLocalq = dfm2::CMat3d(aBone[i].affmat3Global[0], aBone[i].affmat3Global[1], aBone[i - 1].affmat3Global[2],
                aBone[i].affmat3Global[4], aBone[i].affmat3Global[5], aBone[i].affmat3Global[6],
                aBone[i].affmat3Global[8], aBone[i].affmat3Global[9], aBone[i].affmat3Global[10]);
            dfm2::CQuatd bone_q = matLocalq.GetQuaternion();
            
            dfm2::CQuatd parentinv = bone_parent.conjugate();
            dfm2::CQuatd qnew = parentinv * (r_q * bone_q);
            aBone[i].quatRelativeRot[0] = qnew.x;
            aBone[i].quatRelativeRot[1] = qnew.y;
            aBone[i].quatRelativeRot[2] = qnew.z;
            aBone[i].quatRelativeRot[3] = qnew.w;
            UpdateBoneRotTrans(aBone);
        }
    }
}

// Modify the RigBone here, postprocessing
void PostProcessRigBone(
    std::vector<dfm2::CRigBone>& aBone,
    std::vector<dfm2::CRigBone>& aBone_render,
    std::vector<dfm2::CVec3d>& feature_vectorL,
    std::vector<dfm2::CVec3d>& feature_vectorR,
    double* lockAffmat3GlobalL,
    double* lockAffmat3GlobalR
)
{
    applyFK(aBone);
    //aBone[4]
    //dfm2::CRigBone LeftFoot = aBone[4];
    //dfm2::CVec3d p0(aBone[4].RootPosition());
    //const dfm2::CVec3d p0(aBone[3].invBindMat[3], aBone[3].invBindMat[7], aBone[3].invBindMat[11]);
    
    //std::cout << "{x : " << aBone[2].affmat3Global[3] << "}{y : " << aBone[2].affmat3Global[7] << "}{z : " << aBone[2].affmat3Global[11] << "}" << std::endl;
    auto pos0L = aBone[4].RootPosition();
    auto pos0R = aBone[10].RootPosition();
    //auto pos1R = aBone_render[4].RootPosition();
    dfm2::CVec3d posL = dfm2::CVec3d(pos0L[0], pos0L[1], pos0L[2]);
    dfm2::CVec3d posR = dfm2::CVec3d(pos0R[0], pos0R[1], pos0R[2]);
    //dfm2::CVec3d pos1 = dfm2::CVec3d(pos1R[0], pos1R[1], pos1R[2]);
    feature_vectorL.erase(feature_vectorL.begin());
    feature_vectorL.insert(feature_vectorL.end(), posL);
    feature_vectorR.erase(feature_vectorR.begin());
    feature_vectorR.insert(feature_vectorR.end(), posR);
    if (!feature_vectorL.size() == 2)
        std::cout << "size wrong" << std::endl;
    double distanceLeft = dfm2::Distance(feature_vectorL[0], feature_vectorL[1]);
    double distanceRight = dfm2::Distance(feature_vectorR[0], feature_vectorR[1]);
    //std::cout << "Foot Displacement : "  << distance << std::endl;

    if (distanceLeft < 0.17f) {
#ifdef DRAWDEBUG
        float color[3] = { 0.9f,0.1f,0.1f };
        ::glColor3fv(color);
        ::delfem2::opengl::DrawSphereAt(32, 32, 0.5, aBone[4].affmat3Global[3], aBone[4].affmat3Global[7], aBone[4].affmat3Global[11]);
#endif
    }
    else
    {
        std::copy(aBone[4].affmat3Global, aBone[4].affmat3Global + 16, lockAffmat3GlobalL);
    }

    if (distanceRight < 0.17f) {
#ifdef DRAWDEBUG
        float color[3] = { 0.9f,0.1f,0.1f };
        ::glColor3fv(color);
        ::delfem2::opengl::DrawSphereAt(32, 32, 0.5, aBone[10].affmat3Global[3], aBone[10].affmat3Global[7], aBone[10].affmat3Global[11]);
#endif
    }
    else
    {
        std::copy(aBone[10].affmat3Global, aBone[10].affmat3Global + 16, lockAffmat3GlobalR);
    }

#ifdef DRAWDEBUG
    float color[3] = { 0.1f,0.9f,0.1f };
    ::glColor3fv(color);
    ::delfem2::opengl::DrawSphereAt(32, 32, 0.5, lockAffmat3GlobalL[3], lockAffmat3GlobalL[7], lockAffmat3GlobalL[11]);
    ::glColor3fv(color);
    ::delfem2::opengl::DrawSphereAt(32, 32, 0.5, lockAffmat3GlobalR[3], lockAffmat3GlobalR[7], lockAffmat3GlobalR[11]);
#endif
    for (size_t i = 0; i < aBone.size(); ++i) {
        // Copy individual member variables from source to destination
        aBone_render[i].name = aBone[i].name;
        std::copy(std::begin(aBone[i].invBindMat), std::end(aBone[i].invBindMat), std::begin(aBone_render[i].invBindMat));
        aBone_render[i].ibone_parent = aBone[i].ibone_parent;
        std::copy(std::begin(aBone[i].transRelative), std::end(aBone[i].transRelative), std::begin(aBone_render[i].transRelative));
        aBone_render[i].scale = aBone[i].scale;
        std::copy(std::begin(aBone[i].quatRelativeRot), std::end(aBone[i].quatRelativeRot), std::begin(aBone_render[i].quatRelativeRot));
        std::copy(std::begin(aBone[i].affmat3Global), std::end(aBone[i].affmat3Global), std::begin(aBone_render[i].affmat3Global));
    }
    UpdateBoneRotTrans(aBone_render);
    dfm2::CVec3d targetL = dfm2::CVec3d(lockAffmat3GlobalL[3], lockAffmat3GlobalL[7], lockAffmat3GlobalL[11]);
    dfm2::CVec3d targetR = dfm2::CVec3d(lockAffmat3GlobalR[3], lockAffmat3GlobalR[7], lockAffmat3GlobalR[11]);
    //if (distanceLeft < 0.17f) {
    //    solveIK(aBone_render, targetL, 3, 2);
    //}
    //if (distanceRight < 0.17f) {
    //    solveIK(aBone_render, targetR, 9, 8);
    //}
}


template<int ndimin_pose, int ndimout_pose, int nOctave>
void VisBvhPhase_tomcat(
    MLP_Fourier<ndimin_pose, nOctave>& model_pose,
    std::vector<dfm2::CRigBone>& aBone,
    std::vector<dfm2::CRigBone>& aBone_render,
    std::vector<dfm2::CVec3d>& feature_vectorL,
    std::vector<dfm2::CVec3d>& feature_vectorR,
    double* lockAffine3GlobalL,
    double* lockAffine3GlobalR,
    const dfm2::CVec2d& root_pos2,
    const std::vector<dfm2::CVec2d>& vec_pos2,
    const dfm2::CVec2d& vec_dirz,
    const double& vec_phase,
    const std::vector<dfm2::CChannel_BioVisionHierarchy>& vec_bvh_channel_info,
    delfem2::glfw::CViewer3& viewer2,
    Floor& floor) {
    //const size_t nframe = vec_pos2.size();
    const std::vector<unsigned int> vec_id_bones_used = GetIdBoneUsed(vec_bvh_channel_info);

        // ------------
        std::vector<double> vec_input_pose;

        bool res = Hoge_tomcat<ndimin_pose, ndimout_pose, nOctave>(
            aBone, vec_input_pose, model_pose, 
            root_pos2, vec_pos2, vec_dirz, vec_phase, vec_id_bones_used);

        if (!res) { std::cout << "Error Hugotomcat" << std::endl; }
        // ---------

        //viewer2.DrawBegin_oldGL();
        //::glEnable(GL_LINE_SMOOTH);
        //::glEnable(GL_BLEND);
        //::glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        DrawFloorShadow(aBone, floor, +0.1);
        DrawPhase(
            vec_phase,
            static_cast<double>(viewer2.width) / static_cast<double>(viewer2.height));
        //    ::glLineWidth(1);
        //    delfem2::opengl::DrawAxis(30);
        {
            const dfm2::CVec2d p0 = root_pos2;
            const dfm2::CVec2d dz0 = vec_dirz;  // local z coord
            const dfm2::CVec2d dx0(dz0.y, -dz0.x); // local x coord
            //std::cout << "sum of square :" << dz0.x * dz0.x + dz0.y * dz0.y << std::endl;

            const dfm2::CMat3d R0(
                dx0.x, 0, dz0.x,
                0, 1, 0,
                dx0.y, 0, dz0.y);
            dfm2::CQuatd q0y;
            //R0.GetQuat_RotMatrix(q0y.p);
            q0y = R0.GetQuaternion();
            Encoder_Pose::draw_in(vec_input_pose, q0y, p0);
            PostProcessRigBone(aBone, aBone_render, feature_vectorL, feature_vectorR, lockAffine3GlobalL, lockAffine3GlobalR);
        }
        dfm2::CVec3d target = dfm2::CVec3d(0.0f,0.0f,0.0f);

        ::glBegin(GL_LINES);
        ::glColor3d(0.0f, 0.0f, 1.0f);
        ::glVertex3d(0.0f, 0.0f, 0.0f);
        ::glVertex3d(0.0f, 0.1f, 10.0f);
        ::glEnd();
        ::glLineWidth(1);

        dfm2::opengl::DrawBone_Octahedron(
            aBone_render,
            12, -1,
            0.1, 1.0);
    
}

//==================

// print out error
static void error_callback(int error, const char* description) {
    fputs(description, stderr);
}


static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GL_TRUE);
}

float x_target_v = 0.f;
float y_target_v = 0.f;
float keyboard_speed_mag = 25.0f;


int main(int argc, char *argv[]) {
    double manual_phase = 0.0;

    constexpr int ndim_input_pose = 21;
    constexpr int ndim_output_pose = 183;
    //constexpr int noctave = 1;
    constexpr int noctave = 2;
    constexpr int size_hidden_pose = 512;
    const std::string mode = "ff";

    std::filesystem::path path = std::filesystem::current_path();
#if defined(WIN32) || defined(_WIN32)
    MLP_Fourier<ndim_input_pose, noctave> model_pose(
        std::string(PATH_SOURCE_DIR) +
        "/../data/pose_" + mode +
        "_no" + std::to_string(noctave) +
        "_nh" + std::to_string(size_hidden_pose) +
        ".pt");
#else
    MLP_Fourier<ndim_input_pose, noctave> model_pose(
        path.string() +
        "/../data/pose_" + mode +
        "_no" + std::to_string(noctave) +
        "_nh" + std::to_string(size_hidden_pose) +
        ".pt");
#endif

    constexpr int ndim_input_phase = 20;
    constexpr int num_layer_phase = 2;
    //constexpr int num_layer_phase = 1;
    constexpr int size_hidden_phase = 128;

    /*
    RNN_Phase<ndim_input_phase, num_layer_phase, size_hidden_phase> model_phase(
        std::string(PATH_SOURCE_DIR) +
        "/../data/phase_rnn_" +
        std::to_string(num_layer_phase) +
        "_" +
        std::to_string(size_hidden_phase) +
        ".pt");
    */
    std::vector<std::string> vec_path_bvh = {
        "/../data/LocomotionFlat03_000_mirror.bvh",
        "/../data/LocomotionFlat02_000_mirror.bvh",
        "/../data/LocomotionFlat01_000_mirror.bvh",
    };

    std::vector<dfm2::CRigBone> vec_rig_bone;
    std::vector<dfm2::CRigBone> vec_rig_bone_render; 
    std::vector<dfm2::CVec3d> feature_vectorL;
    std::vector<dfm2::CVec3d> feature_vectorR;
    double lockAffine3GlobalL[16];
    double lockAffine3GlobalR[16];
    std::vector<dfm2::CChannel_BioVisionHierarchy> vec_channel_bvh;
    std::vector<double> vec_phase;
    std::vector<double> real_phase;
    std::vector<dfm2::CVec2d> real_DirZ;
    std::vector<dfm2::CVec2d> aRootDirZ, aRootPos2;
    {
        std::string path_bvh = std::string(PATH_SOURCE_DIR) + vec_path_bvh[2];
        size_t nframe = 0;
        double frame_time;
        std::string header_bvh;
        std::vector<double> vec_bvh_time_series_data;
        Read_BioVisionHierarchy(
            vec_rig_bone,
            vec_channel_bvh,
            nframe,
            frame_time,
            vec_bvh_time_series_data,
            header_bvh,
            path_bvh);
        Read_BioVisionHierarchy(
            vec_rig_bone_render,
            vec_channel_bvh,
            nframe,
            frame_time,
            vec_bvh_time_series_data,
            header_bvh,
            path_bvh);

        dfm2::SetPose_BioVisionHierarchy(
            vec_rig_bone, vec_channel_bvh,
            vec_bvh_time_series_data.data() + 0 * vec_channel_bvh.size());
        dfm2::SetPose_BioVisionHierarchy(
            vec_rig_bone_render, vec_channel_bvh,
            vec_bvh_time_series_data.data() + 0 * vec_channel_bvh.size());
        assert(vec_rig_bone.size() == 38 && vec_channel_bvh.size() == 96);
        //
        std::vector<double> aRootHeight;
        std::vector<delfem2::CMat3d> aRootRot;
        TransRotRoot(aRootPos2, aRootHeight, aRootRot, aRootDirZ,
            vec_channel_bvh, vec_bvh_time_series_data);
    }
    auto pos0L = vec_rig_bone[4].RootPosition();
    dfm2::CVec3d pos0Lv = dfm2::CVec3d(pos0L[0], pos0L[1], pos0L[2]);
    std::copy(vec_rig_bone[4].affmat3Global, vec_rig_bone[4].affmat3Global + 16, lockAffine3GlobalL);
    feature_vectorL.push_back(pos0Lv);
    feature_vectorL.push_back(pos0Lv);
    auto pos0R = vec_rig_bone[10].RootPosition();
    dfm2::CVec3d pos0Rv = dfm2::CVec3d(pos0R[0], pos0R[1], pos0R[2]);
    std::copy(vec_rig_bone[10].affmat3Global, vec_rig_bone[10].affmat3Global + 16, lockAffine3GlobalR);
    feature_vectorR.push_back(pos0Rv);
    feature_vectorR.push_back(pos0Rv);
  //const float dt = 0.0083f;
  //std::vector<CParticle> aParticle;

  // initialize particle
  std::mt19937 rndeng(std::random_device{}());
  std::uniform_real_distribution<float> dist01(0, 1);
  //aParticle.resize(N);
  CParticle p;
  //p.pos[0] = dist01(rndeng);
  //p.pos[1] = dist01(rndeng);
  p.pos[0] = -100.0f;
  p.pos[1] = 0.0f;
  p.mass = 1.0f;

  delfem2::GetRGB_HSV(
      p.color[0], p.color[1], p.color[2],
      dist01(rndeng), 1.f, 1.f);

  p.velo[0] = 42.0f;
  p.velo[1] = 0.f;


  class MytempViewer : public delfem2::glfw::CViewer3 {
  public:
      MytempViewer() : CViewer3(45) {
      }

      void key_press(int key, int mods) override {
          delfem2::glfw::CViewer3::key_press(key, mods);
          if (key == GLFW_KEY_F) {
              if (keyboard_speed_mag == 25.0f)
                  keyboard_speed_mag = 8.0f;
              else
                  keyboard_speed_mag = 25.0f;
          }
          if (key == GLFW_KEY_X) {
              x_target_v = 0.0f;
              y_target_v = 0.f;
          }
      }

      void key_repeat(int key, int mods) override {
          delfem2::glfw::CViewer3::key_press(key, mods);
          if (key == GLFW_KEY_W) { 
              x_target_v = 0.f;
              y_target_v = -keyboard_speed_mag;
          }
          if (key == GLFW_KEY_S) {
              x_target_v = 0.f;
              y_target_v = keyboard_speed_mag;
          }
          if (key == GLFW_KEY_A) {
              x_target_v = -keyboard_speed_mag;
              y_target_v = 0.f;
          }
          if (key == GLFW_KEY_D) {
              x_target_v = keyboard_speed_mag;
              y_target_v = 0.f;
          }
      }
  };

  MytempViewer viewer_source;

  viewer_source.width = 1920;
  viewer_source.height = 1080;
  viewer_source.window_title = "Data-Driven Controller";
  viewer_source.view_rotation = std::make_unique<delfem2::ModelView_Ytop>();

  delfem2::glfw::InitGLOld();

  viewer_source.OpenWindow();

  delfem2::opengl::setSomeLighting();

  std::cout << "run starts" << std::endl;

  glfwSetErrorCallback(error_callback);
  if (!glfwInit())
      exit(EXIT_FAILURE);

  if (!viewer_source.window)
  {
      glfwTerminate();
      exit(EXIT_FAILURE);
  }

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

  glfwMakeContextCurrent(viewer_source.window);

  //======
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO(); (void)io;
  //io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
  //io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

  // Setup Dear ImGui style
  ImGui::StyleColorsDark();
  //ImGui::StyleColorsClassic();

  // Setup Platform/Renderer backends
  ImGui_ImplGlfw_InitForOpenGL(viewer_source.window, true);
  ImGui_ImplOpenGL2_Init();

  // Our state
  bool show_demo_window = true;
  bool show_another_window = false;
  ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
  //=======

  int path_count = -1;

  static float f = 0.01f;
  double speed_x = 0.0f;
  double speed_y = 0.0f;
  //damper_p the_p = damper_p();

  enum
  {
      TRAJ_MAX = 120,
      TRAJ_SUB = 20,
      PRED_MAX = 6,
      PRED_SUB = 20,
  };

  float trajx_prev[TRAJ_MAX];
  float trajy_prev[TRAJ_MAX];

  float predx[PRED_MAX], predy[PRED_MAX];
  float predxv[PRED_MAX], predyv[PRED_MAX];
  float predxa[PRED_MAX], predya[PRED_MAX];

  float halflife = 0.70f;
  float dt = 1.0 / 60.0f;
  //float timescale = 240.0f;
  float trajx = 0.0f;
  float trajy = 0.0f;
  float trajxv = 0.0, trajyv = 0.0;
  float trajxa = 0.0, trajya = 0.0;
  float traj_xv_goal = 0.0;
  float traj_yv_goal = 0.0;


  for (int i = 0; i < TRAJ_MAX; i++)
  {
      trajx_prev[i] = 0.0f;
      trajy_prev[i] = 0.0f;
  }

  float velocity_mag = 10.0f;
  dfm2::CVec2d face_dirZ(1.0f, 0.f);

  while (!glfwWindowShouldClose(viewer_source.window))
  {
      glfwMakeContextCurrent(viewer_source.window);

      manual_phase += f;
      if (manual_phase >= 1.0)
      {
          manual_phase = 0.0;
      }

      //std::cout << manual_phase << std::endl;
      bool facing_control_mode = false;


      double tempt_divs = sqrt(p.velo[0] * p.velo[0] + p.velo[1] * p.velo[1]);
      dfm2::CVec2d normal_dirZ = dfm2::CVec2d({ p.velo[0] / tempt_divs,p.velo[1] / tempt_divs });


      Floor floor{ 100, +0.1 };


      viewer_source.DrawBegin_oldGL();
      ::glEnable(GL_LINE_SMOOTH);
      ::glEnable(GL_BLEND);
      ::glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);


     
      for (int i = TRAJ_MAX - 1; i > 0; i--)
      {
          trajx_prev[i] = trajx_prev[i - 1];
          trajy_prev[i] = trajy_prev[i - 1];
      }



      float gamepad_x = 0.0f;
      float gamepad_y = 0.0f;
      float gamepad2_x = 0.0f;
      float gamepad2_y = 0.0f;

      bool buttonX = false;
      bool buttonY = false;
      bool buttonA = false;
      bool buttonB = false;


      // input bind with controller ++++++++++++
      bool check_controller = false;
      int present = 0;
      glfwSetJoystickCallback(joystick_callback);
      if (glfwJoystickPresent(GLFW_JOYSTICK_1) == GLFW_TRUE)
      {

          present = glfwJoystickPresent(GLFW_JOYSTICK_1);

          if (present == 1)
          {
              check_controller = true;
              int axesCount;
              const float* axes = glfwGetJoystickAxes(GLFW_JOYSTICK_1, &axesCount);
              gamepad_x = axes[0];
              gamepad_y = axes[1];
              gamepad2_x = abs(axes[2]) < 0.05 ? 0.0f : axes[2] / 100;
              gamepad2_y = abs(axes[3]) < 0.05 ? 0.0f : axes[3] / 100;
              float gamepad_mag = sqrtf(gamepad_x * gamepad_x + gamepad_y * gamepad_y);

              int triggerRT = axes[5];
              int triggerLT = axes[4];
              //std::cout << "Right X : " << axes[2] << std::endl;
              //std::cout << "Right Y : " << axes[3] << std::endl;
              //std::cout << "trigger LT : " << axes[4] << std::endl;
              //std::cout << "trigger RT : " << axes[5] << std::endl;
              //std::cout << "trigger R3 : " << axes[6] << std::endl;
              //std::cout << "trigger R4 : " << axes[7] << std::endl;

              if (gamepad_mag > 0.2f)
              {
                  float gamepaddirx = gamepad_x / gamepad_mag;
                  float gamepaddiry = gamepad_y / gamepad_mag;
                  float gamepadclippedmag = gamepad_mag > 1.0f ? 1.0f : gamepad_mag * gamepad_mag;
                  gamepad_x = gamepaddirx * gamepadclippedmag;
                  gamepad_y = gamepaddiry * gamepadclippedmag;
              }
              else
              {
                  gamepad_x = 0.0f;
                  gamepad_y = 0.0f;
              }
              //22.0f close to run speed 
              //
              if (triggerLT == 1) {
                  facing_control_mode = true;
              }
              if (triggerRT == 1) {
                  // run

                  f = damper(f, 0.01f, 0.1f);
                  halflife = damper(halflife, 0.7f, 0.1f);
                  velocity_mag = damper(velocity_mag, 22.0f, 0.1f);
                  traj_xv_goal = gamepad_x * velocity_mag;
                  traj_yv_goal = gamepad_y * velocity_mag;
              }
              else
              {
                  // walk 

                  f = damper(f, 0.006f, 0.1f);
                  //halflife = damper(halflife, 0.9, 0.1f);
                  halflife = damper(halflife, 0.5f, 0.1f);
                  velocity_mag = damper(velocity_mag, 10.0f, 0.1f);
                  traj_xv_goal = gamepad_x * velocity_mag;
                  traj_yv_goal = gamepad_y * velocity_mag;
              }
          }

          int buttonCont;
          const unsigned char* buttons = glfwGetJoystickButtons(GLFW_JOYSTICK_1, &buttonCont);
          //0 = A, 1 = B, 2 = X, 3 = Y
          if (GLFW_PRESS == buttons[0])
          {
              buttonA = true;
          }
          else if (GLFW_RELEASE == buttons[0])
          {
              buttonA = false;
          }
          if (GLFW_PRESS == buttons[1])
          {
              buttonB = true;
          }
          else if (GLFW_RELEASE == buttons[1])
          {
              buttonB = false;
          }
          if (GLFW_PRESS == buttons[2])
          {
              buttonX = true;
          }
          else if (GLFW_RELEASE == buttons[2])
          {
              buttonX = false;
          }
          if (GLFW_PRESS == buttons[3])
          {
              buttonY = true;
          }
          else if (GLFW_RELEASE == buttons[3])
          {
              buttonY = false;
          }
      }
      else
      {
          traj_xv_goal = damper(traj_xv_goal, x_target_v, 0.1f);
          traj_yv_goal = damper(traj_yv_goal, y_target_v, 0.1f);
      }

      spring_character_update_dt(trajx, trajxv, trajxa, traj_xv_goal, halflife, dt);
      spring_character_update_dt(trajy, trajyv, trajya, traj_yv_goal, halflife, dt);

      spring_character_predict(predx, predxv, predxa, PRED_MAX, trajx, trajxv, trajxa, traj_xv_goal, halflife, dt* PRED_SUB);
      spring_character_predict(predy, predyv, predya, PRED_MAX, trajy, trajyv, trajya, traj_yv_goal, halflife, dt* PRED_SUB);

      trajx_prev[0] = trajx;
      trajy_prev[0] = trajy;

      std::vector<dfm2::CVec2d> vec_pos2;
      vec_pos2.clear();

      for (int i = 0; i < TRAJ_MAX - TRAJ_SUB; i += TRAJ_SUB)
      {
          std::vector start = { trajx_prev[i + 0], trajy_prev[i + 0] };
          std::vector stop = { trajx_prev[i + TRAJ_SUB], trajy_prev[i + TRAJ_SUB] };
          
          if (i == 0) {
              draw_black_sphere(start);
              //std::cout << "root pos : " << start[0] << " - " << start[1] << std::endl;
          }
          else
          {
              dfm2::CVec2d tempt(start[0], start[1]);
              vec_pos2.push_back(tempt);
              draw_blue_sphere(start);
          }
          //DrawLineV(start, stop, BLUE);
          
          if (i + TRAJ_SUB == TRAJ_MAX - TRAJ_SUB)
          {
              dfm2::CVec2d tempt(stop[0], stop[1]);
              vec_pos2.push_back(tempt);
              draw_blue_sphere(stop);
          }
      }
      std::reverse(vec_pos2.begin(), vec_pos2.end());


      for (int i = 1; i < PRED_MAX; i += 1 )
      {
          std::vector start = { predx[i + 0], predy[i + 0] };
          std::vector stop = { predx[i - 1], predy[i - 1] };

          //DrawLineV(start, stop, MAROON);
          dfm2::CVec2d tempt(start[0], start[1]);
          vec_pos2.push_back(tempt);
          draw_red_sphere(start);
      }


      speed_x = (predx[1] - trajx_prev[0]) / (dt * 20);
      speed_y = (predy[1] - trajy_prev[0]) / (dt * 20);
      if (facing_control_mode == false)
        viewer_source.view_rotation->Rot_Camera(-static_cast<float>(gamepad2_x), -static_cast<float>(gamepad2_y));


      //NN input are here  
      float goal_x = 0.0f;
      float goal_y = 0.0f;
      if (facing_control_mode == false) {
          dfm2::CVec2d goal_dirZ(predx[1] - trajx_prev[0], predy[1] - trajy_prev[0]);
          goal_dirZ.normalize();
          face_dirZ.normalize();
          goal_x = damper(face_dirZ.x, goal_dirZ.x, 0.065);
          goal_y = damper(face_dirZ.y, goal_dirZ.y, 0.065);
          //goal_x = damper(face_dirZ.x, goal_dirZ.x, 0.1);
          //goal_y = damper(face_dirZ.y, goal_dirZ.y, 0.1);
          if ((goal_x >= -1) && (goal_x <= 1))
          {
              face_dirZ.x = goal_x; // get rid of initial broken data, fix it later
          }
          if ((goal_y >= -1) && (goal_y <= 1))
          {
              face_dirZ.y = goal_y;
          }
      }
      else {
          dfm2::CVec2d goal_dirZ(gamepad2_x, gamepad2_y);
          goal_dirZ.normalize();
          face_dirZ.normalize();
          goal_x = damper(face_dirZ.x, goal_dirZ.x, 0.05);
          goal_y = damper(face_dirZ.y, goal_dirZ.y, 0.05);
          if ((goal_x >= -1) && (goal_x <= 1))
          {
              face_dirZ.x = goal_x; // get rid of initial broken data, fix it later
          }
          if ((goal_y >= -1) && (goal_y <= 1))
          {
              face_dirZ.y = goal_y;
          }
      }


      //face_dirZ.x = goal_x;
      //face_dirZ.y = goal_y;
      face_dirZ.normalize();
      //std::cout << "face_dir" << face_dirZ[0] << "-" << face_dirZ[1] << std::endl;
      //

      ::glLineWidth(1);
      ::glColor3d(0, 0, 0);
      ::glBegin(GL_LINES);
      ::glVertex3d(trajx_prev[0], 0.1f, trajy_prev[1]);
      ::glVertex3d(trajx_prev[0] + face_dirZ[0] * 5, 0.1f, trajy_prev[1] + face_dirZ[1] * 5);
      ::glEnd();

      dfm2::CVec2d root_pos2(trajx_prev[0], trajy_prev[0]);
     // std::cout <<   "traj size : " << vec_pos2.size() << std::endl;

      VisBvhPhase_tomcat<ndim_input_pose, ndim_output_pose>(
          model_pose, vec_rig_bone, vec_rig_bone_render, feature_vectorL, feature_vectorR,lockAffine3GlobalL, lockAffine3GlobalR, root_pos2,
          vec_pos2, face_dirZ, manual_phase, vec_channel_bvh,
          viewer_source, floor);

      floor.draw_checkerboard();
      



      // GUI *******
      ImGui_ImplOpenGL2_NewFrame();
      ImGui_ImplGlfw_NewFrame();
      ImGui::NewFrame();

      // 1. Show the big demo window (Most of the sample code is in ImGui::ShowDemoWindow()! You can browse its code to learn more about Dear ImGui!).
      {
          //static float f = 0.0f;
          ImGui::Begin("Guide");
          ImGui::Text("Controller User");
          ImGui::BulletText("Leftstick       - Control Moving Direction");
          ImGui::BulletText("Rightstick      - Control Camera Direction");
          ImGui::BulletText("LT + Rightstick - Control Facing Direction");
          ImGui::BulletText("RT (HOLD)       - Switch to Run");

          ImGui::Separator();
          ImGui::Text("Keyboard User");
          ImGui::BulletText(" W S A D (HOLD)                 - Control Moving Direction");
          ImGui::BulletText(" ALT (HOLD) + Left Click (HOLD) - Control Camera Direction");
          ImGui::BulletText(" F                              - Switch to Run/Walk");
          ImGui::BulletText(" X                              - Stop Character");
          ImGui::End();

          ImGui::Begin("Control Panel");                          // Create a window called "Hello, world!" and append into it.
          ImGui::Checkbox("XBOX Gamepad", &check_controller);
          ImGui::Checkbox("X", &buttonX); ImGui::SameLine();
          ImGui::Checkbox("Y", &buttonY); ImGui::SameLine();
          ImGui::Checkbox("A", &buttonA); ImGui::SameLine();
          ImGui::Checkbox("B", &buttonB); ImGui::SameLine();
          ImGui::Separator();
          ImGui::Text("Phase Step");               // Display some text (you can use a format strings too)
          ImGui::BulletText("Recommendation - walk: 0.006 ; run: 0.01");
          ImGui::Separator();
          ImGui::SliderFloat("Phase Float", &f, 0.00f, 0.015f);            // Edit 1 float using a slider from 0.0f to 1.0f
          //ImGui::ColorEdit3("clear color", (float*)&clear_color); // Edit 3 floats representing a color
          ImGui::Text("Trajectory smoothness");
          ImGui::SliderFloat("Traj Half-life", &halflife, 0.0f, 0.9f);
          ImGui::Separator();
          ImGui::Text("Current Direction");
          ImGui::Text("dir x: %f ; dir y: %f", goal_x, goal_y);
          ImGui::Text("Current Speed");               // Display some text (you can use a format strings too)
          ImGui::Text("dir x: %f ; dir y: %f", speed_x, speed_y);
          ImGui::Separator();
          //ImGui::SameLine();
          //ImGui::Text("counter = %d", counter);

          ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
          ImGui::End();
      }

      // Rendering
      ImGui::Render();
      int display_w, display_h;
      glfwGetFramebufferSize(viewer_source.window, &display_w, &display_h);
      glViewport(0, 0, display_w, display_h);
      //glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
      //glClear(GL_COLOR_BUFFER_BIT);
      ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
      // GUI  *********

      viewer_source.SwapBuffers();
      //glfwSwapBuffers(viewer_source.window);
      glfwPollEvents();
  }


  glfwDestroyWindow(viewer_source.window);
  glfwTerminate();
  exit(EXIT_SUCCESS);
}
