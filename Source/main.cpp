/*
 *  Yifei Chen
 *  Motion Matching Demo
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
//#include "path.h"

#include "../traj.h"
//#include "../model.h"
#include "../gendataset.h"
#include "../draw.h"
#include "delfem2/rig_geo3.h"
#include "delfem2/glfw/viewer3.h"
#include "delfem2/glfw/util.h"
#include "delfem2/opengl/old/funcs.h"
#include "delfem2/opengl/old/rigv3.h"
#include "delfem2/rig_bvh.h"

#define DRAWDEBUG

//   bvh loader, parser and model loaders
namespace dfm2 = delfem2;

dfm2::CQuatd Quat_Lerp(dfm2::CQuatd& q1, dfm2::CQuatd& q2, float t)
{
    dfm2::CQuatd out;
    out.x = q1.x + (q2.x - q1.x) * t;
    out.y = q1.y + (q2.y - q1.y) * t;
    out.z = q1.z + (q2.z - q1.z) * t;
    out.w = q1.w + (q2.w - q1.w) * t;
    out.normalize();
    return out;
}

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

struct KeyPoint
{
    dfm2::CVec2d Position;
    float Velocity;

    //public float3 GetWorldPosition(Transform transform)
    //{
    //    return transform.position + new Vector3(Position.x, 0, Position.y);
    //}
};


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


//
//template<int ndimin_pose, int ndimout_pose, int nOctave>
//bool Hoge_tomcat(
//    std::vector<dfm2::CRigBone>& aBone,
//    std::vector<double>& vec_input_pose,
//    MLP_Fourier<ndimin_pose, nOctave>& model_pose,
//    const dfm2::CVec2d& root_pos2,
//    const std::vector<dfm2::CVec2d>& vec_pos2,
//    const dfm2::CVec2d& vec_dirz,
//    const double& vec_phase,
//    const std::vector<unsigned int>& vec_id_bones_used) {
//    //const unsigned int nframe = vec_phase.size();
//    vec_input_pose.clear();
//    {
//        const dfm2::CVec2d p0 = root_pos2;
//        const dfm2::CVec2d dz0 = vec_dirz;  // local z coord
//        assert(fabs(dz0.norm() - 1) < 1.0e-5);
//        const dfm2::CVec2d dx0(dz0.y, -dz0.x); // local x coord
//
//        vec_input_pose.push_back(vec_phase);
//        for (unsigned int ismpl = 0; ismpl < 10; ismpl++) {
//            vec_input_pose.push_back(dx0.dot(vec_pos2[ismpl] - p0));
//            vec_input_pose.push_back(dz0.dot(vec_pos2[ismpl] - p0));
//        }
//    }
//    if (vec_phase < -0.5) { return false; }
//    assert(vec_input_pose.size() == ndimin_pose);
//    // ------------------------
//    std::vector<double> vec_output_pose;
//    {
//        std::vector<float> aInF(vec_input_pose.begin(), vec_input_pose.end());
//        model_pose.predict(vec_output_pose, aInF);
//    }
//    {
//        const dfm2::CVec2d p0 = root_pos2;
//        const dfm2::CVec2d dz0 = vec_dirz;  // local z coord
//        const dfm2::CVec2d dx0(dz0.y, -dz0.x); // local x coord
//        const dfm2::CMat3d R0(
//            dx0.x, 0, dz0.x,
//            0, 1, 0,
//            dx0.y, 0, dz0.y);
//        dfm2::CQuatd q0y;
//        //R0.GetQuat_RotMatrix(q0y.p);
//        q0y = R0.GetQuaternion();
//        Encoder_Pose::decode(aBone, vec_output_pose, vec_id_bones_used, q0y, p0);
//    }
//    return true;
//}

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

void solveIK(std::vector<dfm2::CRigBone>& aBone, dfm2::CVec3d target, int start_bone, int end_bone)
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

bool VectorSerialize(const std::string& filename, const std::vector<double>& vec) {
    std::ofstream outFile(filename, std::ios::binary);
    if (!outFile.is_open()) {
        std::cerr << "Failed to open file for writing: " << filename << std::endl;
        return false;
    }

    size_t size = vec.size();
    outFile.write(reinterpret_cast<const char*>(&size), sizeof(size));
    outFile.write(reinterpret_cast<const char*>(vec.data()), size * sizeof(double));
    outFile.close();
    return true;
}


bool VectorDeserialize(const std::string& filename, std::vector<double>& vec) {
    std::ifstream inFile(filename, std::ios::binary);
    if (!inFile.is_open()) {
        std::cerr << "Failed to open file for reading: " << filename << std::endl;
        return false;
    }

    size_t size;
    inFile.read(reinterpret_cast<char*>(&size), sizeof(size));
    vec.resize(size);
    inFile.read(reinterpret_cast<char*>(vec.data()), size * sizeof(double));
    inFile.close();
    return true;
}

bool fileExistsInDirectory(const std::string& directory, const std::string& fileName) {
    std::filesystem::path dirPath(directory);
    if (std::filesystem::exists(dirPath) && std::filesystem::is_directory(dirPath)) {
        for (const auto& entry : std::filesystem::directory_iterator(dirPath)) {
            if (entry.is_regular_file() && entry.path().filename() == fileName) {
                return true;
            }
        }
    }
    return false;
}


dfm2::CVec3d AffmatLerp(double* x, double* y, double a)
{
    dfm2::CVec3d temp({ x[3] + a * (y[3] - x[3]) , x[7] + a * (y[7] - x[7]), x[11] + a * (y[11] - x[11]) });
    return temp;
}

// Modify the RigBone here, postprocessing
void PostProcessRigBone(
    std::vector<dfm2::CRigBone>& aBone,
    std::vector<dfm2::CRigBone>& aBone_render,
    std::vector<dfm2::CVec3d>& feature_vectorL,
    std::vector<dfm2::CVec3d>& feature_vectorR,
    double* lockAffmat3GlobalL,
    double* lockAffmat3GlobalR,
    double& LerpL,
    double* lockAffmat3GlobalLGoal
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
        float color[3] = { 1.0f,1.0f,0.6f };
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
        float color[3] = { 1.0f,1.0f,0.6f };
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
    if (distanceLeft < 0.17f) {
        solveIK(aBone_render, targetL, 3, 2);
        //std::copy(aBone[4].affmat3Global, aBone[4].affmat3Global + 16, lockAffmat3GlobalLGoal);
        //LerpL = 0.0f;
    }
    else
    {
        //if (LerpL < 1.0f)
        //{
        //    dfm2::CVec3d targetL_Lerp = AffmatLerp(lockAffmat3GlobalLGoal, lockAffmat3GlobalL, LerpL);
        //    //std::cout << LerpL << std::endl;
        //    LerpL += 0.01f;
        //    solveIK(aBone_render, targetL_Lerp, 3, 2);
        //    //float color[3] = { 0.1f,0.1f,1.0f };
        //    //::glColor3fv(color);
        //    //::delfem2::opengl::DrawSphereAt(32, 32, 0.5, targetL_Lerp.x, targetL_Lerp.y, targetL_Lerp.z);
        //}
        //else
        //{
        //    std::copy(aBone[4].affmat3Global, aBone[4].affmat3Global + 16, lockAffmat3GlobalLGoal);
        //}
    }

    if (distanceRight < 0.17f) {
        solveIK(aBone_render, targetR, 9, 8);
    }

}

void ChangeBoneRoot(std::vector<dfm2::CRigBone>& aBone, double x , double y, double z){
    aBone[0].SetTranslation(aBone[0].transRelative[0] + x, aBone[0].transRelative[1] + y, aBone[0].transRelative[2] + z);
    //dfm2::UpdateBoneRotTrans(aBone);
}

dfm2::CVec3d ComputeForward(dfm2::CVec3d leftShoulder, dfm2::CVec3d rightShoulder, dfm2::CVec3d leftLeg, dfm2::CVec3d rightLeg)
{
    dfm2::CVec3d shoulder = leftShoulder - rightShoulder;
    dfm2::CVec3d leg = leftLeg - rightLeg;
    dfm2::CVec3d average = shoulder + leg;
    dfm2::CVec3d up(0.0f,1.0f,0.0f);
    dfm2::CVec3d forward = average.cross(up);
    forward.y = 0.0f;
    forward.normalize();
    return forward;
}

dfm2::CQuatd LookRotation(dfm2::CVec3d forward, dfm2::CVec3d up)
{
    forward.normalize();
    dfm2::CVec3d right = up.cross(forward);
    right.normalize();
    up = forward.cross(right);

    dfm2::CMat3d lookRotation(right.x,up.x,forward.x,
                              right.y, up.y,forward.y, 
                              right.z, up.z,forward.z);
    dfm2::CQuatd orientation;
    dfm2::Quat_Mat3(orientation.p, lookRotation.data());
    orientation.normalize();
    return orientation;
}

void Path_Draw(std::vector<KeyPoint>& Path)
{
    for (int index = 0; index < Path.size(); index++)
    {
        float yellowcolor[3] = { 0.9f,0.9f,0.1f };
        ::glColor3fv(yellowcolor);
        ::dfm2::opengl::DrawSphereAt(32, 32, 4, Path[index].Position.x, 0.0f, Path[index].Position.y);
        ::glBegin(GL_LINES);
        //yellow lines
        ::glColor3d(0.9f, 0.9f, 0.1f);
        ::glVertex3d(Path[index].Position.x, 0.2, Path[index].Position.y);
        ::glVertex3d(Path[(index + 1) % Path.size()].Position.x, 0.2, Path[(index + 1) % Path.size()].Position.y);
        ::glEnd();
    }
}

void SetPose_BioVisionHierarchy_VirtualBone_lerp(
    const double* virtual_root,
    const double* virtual_root_dir,
    const double* virtual_root_for,
    std::vector<dfm2::CRigBone>& bones,
    const std::vector<dfm2::CChannel_BioVisionHierarchy>& channels,
    const double* values,
    bool first_frame,
    dfm2::CQuatd& facing,
    dfm2::CQuatd c_facing,
    float t) {
    dfm2::CQuatd p_q;
    std::vector<dfm2::CQuatd> old;
    for (auto& bone : bones) {
        dfm2::CQuatd q;
        q.x = bone.quatRelativeRot[0];
        q.y = bone.quatRelativeRot[1];
        q.z = bone.quatRelativeRot[2];
        q.w = bone.quatRelativeRot[3];
        bone.quatRelativeRot[0] = 0.0;
        bone.quatRelativeRot[1] = 0.0;
        bone.quatRelativeRot[2] = 0.0;
        bone.quatRelativeRot[3] = 1.0;
        old.push_back(q);
    }
    const size_t nch = channels.size();
    for (unsigned int ich = 0; ich < nch; ++ich) {
        const int ibone = channels[ich].ibone;
        const int iaxis = channels[ich].iaxis;
        const bool isrot = channels[ich].isrot;
        const double val = values[ich];
        assert(ibone < (int)bones.size());
        assert(iaxis >= 0 && iaxis < 3);
        if (!isrot) {
            bones[ibone].transRelative[iaxis] = val;
        }
        else {
            const double ar = val * M_PI / 180.0;
            double v0[3] = { 0, 0, 0 };
            v0[iaxis] = 1.0;
            double dq[4] = { v0[0] * sin(ar * 0.5), v0[1] * sin(ar * 0.5), v0[2] * sin(ar * 0.5), cos(ar * 0.5) };
            double qtmp[4];
            dfm2::QuatQuat(qtmp,
                bones[ibone].quatRelativeRot, dq);
            dfm2::Copy_Quat(bones[ibone].quatRelativeRot, qtmp);
        }
    }
    if (!first_frame)
    {
        double qy_diff[4];
        dfm2::CQuatd q0;
        q0.x = virtual_root_for[0];
        q0.y = virtual_root_for[1];
        q0.z = virtual_root_for[2];
        q0.w = virtual_root_for[3];
        dfm2::CQuatd q1;
        q1.x = virtual_root_for[4];
        q1.y = virtual_root_for[5];
        q1.z = virtual_root_for[6];
        q1.w = virtual_root_for[7];

        dfm2::CQuatd q0_inv;
        dfm2::Inverse_Quat(q0_inv.p, q0.p);
        q0_inv.normalize();
        q0_inv.SetSmallerRotation();
        dfm2::QuatQuat(qy_diff, q0_inv.p, q1.p);
        double new_face[4];
        dfm2::QuatQuat(new_face, facing.p, qy_diff);
        facing.x = new_face[0];
        facing.y = new_face[1];
        facing.z = new_face[2];
        facing.w = new_face[3];

        double xz[4] = { virtual_root_dir[0],virtual_root_dir[1],virtual_root_dir[2],virtual_root_dir[3] };
        double xz_for[4];
        dfm2::QuatQuat(xz_for, new_face, xz);
        dfm2::Copy_Quat(bones[0].quatRelativeRot, xz_for);
    }
    else
    {
        double xz[4] = { virtual_root_dir[0],virtual_root_dir[1],virtual_root_dir[2],virtual_root_dir[3] };
        double xz_for[4];
        dfm2::QuatQuat(xz_for, facing.p, xz);
        dfm2::Copy_Quat(bones[0].quatRelativeRot, xz_for);
    }
    bones[0].transRelative[0] += virtual_root[0];
    bones[0].transRelative[2] += virtual_root[1];
    
    int bone_index = 0;
    for (auto& bone : bones) {
        if (bone_index == 0)
        {
            bone_index += 1;
            continue;
        }
        dfm2::CQuatd c_q;
        c_q.x = bone.quatRelativeRot[0];
        c_q.y = bone.quatRelativeRot[1];
        c_q.z = bone.quatRelativeRot[2];
        c_q.w = bone.quatRelativeRot[3];
        dfm2::CQuatd interpolate = Quat_Lerp(old[bone_index], c_q, t);
        bone.quatRelativeRot[0] = interpolate.x;
        bone.quatRelativeRot[1] = interpolate.y;
        bone.quatRelativeRot[2] = interpolate.z;
        bone.quatRelativeRot[3] = interpolate.w;
        bone_index += 1;
    }
    old.clear();
    UpdateBoneRotTrans(bones);
}

void SetPose_BioVisionHierarchy_VirtualBone(
    const double* virtual_root,
    const double* virtual_root_dir,
    const double* virtual_root_for,
    std::vector<dfm2::CRigBone>& bones,
    const std::vector<dfm2::CChannel_BioVisionHierarchy>& channels,
    const double* values,
    bool first_frame,
    dfm2::CQuatd& facing,
    dfm2::CQuatd c_facing) {
    dfm2::CQuatd p_q;

    for (auto& bone : bones) {
        bone.quatRelativeRot[0] = 0.0;
        bone.quatRelativeRot[1] = 0.0;
        bone.quatRelativeRot[2] = 0.0;
        bone.quatRelativeRot[3] = 1.0;
    }
    const size_t nch = channels.size();
    for (unsigned int ich = 0; ich < nch; ++ich) {
        const int ibone = channels[ich].ibone;
        const int iaxis = channels[ich].iaxis;
        const bool isrot = channels[ich].isrot;
        const double val = values[ich];
        assert(ibone < (int)bones.size());
        assert(iaxis >= 0 && iaxis < 3);
        if (!isrot) {
            bones[ibone].transRelative[iaxis] = val;
        }
        else {
            const double ar = val * M_PI / 180.0;
            double v0[3] = { 0, 0, 0 };
            v0[iaxis] = 1.0;
            double dq[4] = { v0[0] * sin(ar * 0.5), v0[1] * sin(ar * 0.5), v0[2] * sin(ar * 0.5), cos(ar * 0.5) };
            double qtmp[4];
            dfm2::QuatQuat(qtmp,
                bones[ibone].quatRelativeRot, dq);
            dfm2::Copy_Quat(bones[ibone].quatRelativeRot, qtmp);
        }
    }
    if (!first_frame)
    {
        double qy_diff[4];
        dfm2::CQuatd q0;
        q0.x = virtual_root_for[0];
        q0.y = virtual_root_for[1];
        q0.z = virtual_root_for[2];
        q0.w = virtual_root_for[3];
        dfm2::CQuatd q1;
        q1.x = virtual_root_for[4];
        q1.y = virtual_root_for[5];
        q1.z = virtual_root_for[6];
        q1.w = virtual_root_for[7];

        dfm2::CQuatd q0_inv;
        dfm2::Inverse_Quat(q0_inv.p,q0.p);
        q0_inv.normalize();
        q0_inv.SetSmallerRotation();
        dfm2::QuatQuat(qy_diff, q0_inv.p, q1.p);
        double new_face[4];
        dfm2::QuatQuat(new_face, facing.p, qy_diff);
        facing.x = new_face[0];
        facing.y = new_face[1];
        facing.z = new_face[2];
        facing.w = new_face[3];

        double xz[4] = { virtual_root_dir[0],virtual_root_dir[1],virtual_root_dir[2],virtual_root_dir[3] };
        double xz_for[4];
        dfm2::QuatQuat(xz_for, new_face, xz);
        dfm2::Copy_Quat(bones[0].quatRelativeRot, xz_for);
    }
    else
    {
        double xz[4] = { virtual_root_dir[0],virtual_root_dir[1],virtual_root_dir[2],virtual_root_dir[3] };
        double xz_for[4];
        dfm2::QuatQuat(xz_for, facing.p, xz);
        dfm2::Copy_Quat(bones[0].quatRelativeRot, xz_for);
    }

    //dfm2::CVec3d worldPosition = { virtual_root[0] , 0.0f,virtual_root[1] };
    //dfm2::CVec3d localPosition;
    //::dfm2::QuatConjVec(localPosition.p, forwardQuat.p, worldPosition.p);
    bones[0].transRelative[0] += virtual_root[0];
    bones[0].transRelative[2] += virtual_root[1];

    //bones[0].transRelative[0] = localPosition.x;
    //bones[0].transRelative[1] = localPosition.y;
    //bones[0].transRelative[2] = localPosition.z;

    //double localRotation[4];
    //dfm2::QuatQuat(localRotation,
    //    bones[0].quatRelativeRot, forwardQuat.p);
    //dfm2::Copy_Quat(bones[0].quatRelativeRot, localRotation);

    UpdateBoneRotTrans(bones);
}

void DeNormalizeTraj(
    const double* sd,
    const double* mean,
    const double* feature_vector,
    dfm2::CVec3d& future_point,
    const int traj_index
)
{
    future_point.x = feature_vector[0] * sd[2 * traj_index] + mean[2 * traj_index];
    future_point.y = 0.0f;
    future_point.z = feature_vector[1] * sd[2 * traj_index + 1] + mean[2 * traj_index + 1];
}

void SimulatePath(float remainingTime, int currentKeypoint, float currentKeyPointT,
    int& nextKeypoint, float& nextKeyPointTime,
    dfm2::CVec2d& nextPos, dfm2::CVec2d& nextDir, std::vector<KeyPoint>& Path, bool debugDraw)
{

    nextPos.setZero();
    nextDir.setZero();
    if (remainingTime <= 0)
    {
        KeyPoint& current = Path[currentKeypoint];
        KeyPoint& next = Path[(currentKeypoint + 1) % Path.size()];
        dfm2::CVec2d dir = next.Position - current.Position;
        nextPos = current.Position + dir * currentKeyPointT;
        nextDir = dir.normalized();
    }

    while (remainingTime > 0)
    {
        KeyPoint& current = Path[currentKeypoint];
        KeyPoint& next = Path[(currentKeypoint + 1) % Path.size()];
        dfm2::CVec2d dir = next.Position - current.Position;
        dfm2::CVec2d dirNorm = dir.normalized();
        dfm2::CVec2d currentPos = current.Position + dir * currentKeyPointT;
        double dis = dfm2::Distance2(currentPos.p, next.Position.p);
        float timeToNext = dis / current.Velocity;
        float dt = std::min(remainingTime, timeToNext);
        remainingTime -= dt;
        if (remainingTime <= 0)
        {
            currentPos += dirNorm * current.Velocity * dt;
            double dis = dfm2::Distance2(current.Position.p, currentPos.p);
            double dis2 = dfm2::Distance2(current.Position.p, next.Position.p);
            currentKeyPointT = dis / dis2;
            nextPos = currentPos;
            nextDir = dirNorm;
        }
        else
        {
            currentKeypoint = (currentKeypoint + 1) % Path.size();
            currentKeyPointT = 0;
        }
    }
    nextKeypoint = currentKeypoint;
    nextKeyPointTime = currentKeyPointT;

    if (debugDraw)
    {
        float greencolor[3] = { 0.1f,0.9f,0.1f };
        ::glColor3fv(greencolor);
        ::dfm2::opengl::DrawSphereAt(32, 32, 4, nextPos.x, 0.0f, nextPos.y);
    }
}
void TrajectDebug(
    std::vector<double> sd,
    std::vector<double> mean,
    dfm2::CVec3d& centerCharacter,
    dfm2::CQuatd& virtual_face,
    std::vector<double>& feature_vector,
    int frame
)
{
    float redcolor[3] = { 0.9f,0.1f,0.1f };
    ::glColor3fv(redcolor);
    ::dfm2::opengl::DrawSphereAt(32, 32, 4, centerCharacter.x, 0.0f, centerCharacter.z);

    for (int i = 0; i < 3; i++)
    {
        dfm2::CVec3d local_future_point;
        DeNormalizeTraj(sd.data(), mean.data(), feature_vector.data() + (frame * 27 + i * 2), local_future_point, i);
        dfm2::CVec3d global_future_point;
        //::dfm2::QuatConjVec(global_future_point.p, virtual_face.p, local_future_point.p);
        dfm2::QuatVec(global_future_point.p, virtual_face.p, local_future_point.p);
        float bluecolor[3] = { 0.1f,0.1f,0.9f };
        ::glColor3fv(bluecolor);
        ::dfm2::opengl::DrawSphereAt(32, 32, 4, centerCharacter.x + global_future_point.x, global_future_point.y, centerCharacter.z + global_future_point.z);
        //::dfm2::opengl::DrawSphereAt(32, 32, 1, futurePos.x, futurePos.y, futurePos.z);
    }
}

void VisBvhMotion(
    std::vector<dfm2::CRigBone>& aBone,
    const std::vector<dfm2::CChannel_BioVisionHierarchy>& vec_bvh_channel_info,
    delfem2::glfw::CViewer3& viewer2,
    std::vector<double>& vec_bvh_time_series_data,
    std::vector<double>& vec_bvh_feature_data,
    Floor& floor,
    const int frame,
    int index_highlight,
    std::vector<double> virtual_bone_localpos,
    std::vector<double> virtual_bone_localdir,
    std::vector<double> virtual_bone_localfor,
    dfm2::CVec2d& virtual_bone,
    dfm2::CQuatd& virtual_face,
    std::vector<double> sd,
    std::vector<double> mean,
    float lerp_t,
    bool useInterpolate
    ){

    //virtual_bone.x += virtual_bone_localpos[frame * 2 + 0];
    //virtual_bone.y += virtual_bone_localpos[frame * 2 + 1];
    //ChangeBoneRoot(aBone, 0.0f, 100.0f, -300.0f);

    //SetPose_BioVisionHierarchy_VirtualBone(
    //    virtual_bone.data(),
    //    aBone, vec_bvh_channel_info,
    //    vec_bvh_time_series_data.data() + frame * vec_bvh_channel_info.size(),dfm2::CQuatd facing);

    DrawFloorShadow(aBone, floor, +0.1);

    dfm2::CVec3d leftShoulder(aBone[14].RootPosition()[0], aBone[14].RootPosition()[1], aBone[14].RootPosition()[2]);
    dfm2::CVec3d rightShoulder(aBone[9].RootPosition()[0], aBone[9].RootPosition()[1], aBone[9].RootPosition()[2]);
    dfm2::CVec3d leftLeg(aBone[23].RootPosition()[0], aBone[23].RootPosition()[1], aBone[23].RootPosition()[2]);
    dfm2::CVec3d rightLeg(aBone[18].RootPosition()[0], aBone[18].RootPosition()[1], aBone[18].RootPosition()[2]);
    dfm2::CVec3d forward = ComputeForward(leftShoulder, rightShoulder, leftLeg, rightLeg);
    dfm2::CVec3d centerCharacter(aBone[0].RootPosition()[0], aBone[0].RootPosition()[1], aBone[0].RootPosition()[2]);

    dfm2::CVec3d joint(aBone[index_highlight].RootPosition()[0],aBone[index_highlight].RootPosition()[1], aBone[index_highlight].RootPosition()[2]);
    dfm2::CVec3d face = joint + forward * 25;

    float color[3] = { 0.95f,0.95f,0.95f };
    float bluecolor[3] = { 0.1f,0.1f,0.9f };
    ::glColor3fv(color);
    ::dfm2::opengl::DrawSphereAt(32, 32, 3, joint.x, joint.y, joint.z);

    ::dfm2::opengl::DrawSphereAt(32, 32, 3, aBone[20].RootPosition()[0], aBone[20].RootPosition()[1], aBone[20].RootPosition()[2]);
    ::dfm2::opengl::DrawSphereAt(32, 32, 3, aBone[25].RootPosition()[0], aBone[25].RootPosition()[1], aBone[25].RootPosition()[2]);
    ::glBegin(GL_LINES);
    ::glColor3d(0.0f, 0.0f, 1.0f);
    ::glVertex3d(joint.x, joint.y, joint.z);
    ::glVertex3d(face.x, face.y, face.z);
    ::glEnd();

    dfm2::CQuatd forwardQuat = LookRotation(forward, dfm2::CVec3d(0.0f, 1.0f, 0.0f));

    //for (int i = 1; i < 4; i++)
    //{
    //    SetPose_BioVisionHierarchy_VirtualBone(
    //        virtual_bone.data(),
    //        aBone, vec_bvh_channel_info,
    //        vec_bvh_time_series_data.data() + (frame + 20 * i) * vec_bvh_channel_info.size(), forwardQuat);
    //    dfm2::CVec3d futurePos(aBone[0].RootPosition()[0], aBone[0].RootPosition()[1], aBone[0].RootPosition()[2]);
    //    dfm2::CVec3d globalDifference = futurePos - centerCharacter;
    //    //dfm2::CQuatd invForward = forwardQuat.conjugate();
    //    dfm2::CVec3d localPosition;
    //    ::dfm2::QuatConjVec(localPosition.p, forwardQuat.p, globalDifference.p);
    //    ::glColor3fv(bluecolor);
    //    ::dfm2::opengl::DrawSphereAt(32, 32, 1, localPosition.x, localPosition.y + centerCharacter.y, localPosition.z);
    //    ::dfm2::opengl::DrawSphereAt(32, 32, 1, futurePos.x, futurePos.y, futurePos.z);

    //}

    dfm2::CVec3d manual_face = { 1.0f, 0.0f, 0.0f };
    dfm2::CQuatd manual_faceq = LookRotation(manual_face, dfm2::CVec3d(0.0f, 1.0f, 0.0f));

    if (frame == 0)
    {
        virtual_face.x = manual_faceq.x;
        virtual_face.y = manual_faceq.y;
        virtual_face.z = manual_faceq.z;
        virtual_face.w = manual_faceq.w;

        dfm2::CVec3d localPosTemp = { virtual_bone_localpos[frame * 2 + 0] , 0.0f, virtual_bone_localpos[frame * 2 + 1] };
        dfm2::CVec3d localPosDir;
        dfm2::QuatVec(localPosDir.p, manual_faceq.p, localPosTemp.p);
        virtual_bone.x += localPosDir.x;
        virtual_bone.y += localPosDir.z;

        SetPose_BioVisionHierarchy_VirtualBone(
            virtual_bone.data(),
            virtual_bone_localdir.data() + frame * 4,
            virtual_bone_localfor.data() + frame * 4,
            aBone, vec_bvh_channel_info,
            vec_bvh_time_series_data.data() + frame * vec_bvh_channel_info.size(), true, virtual_face, forwardQuat);
        //SetPose_BioVisionHierarchy_VirtualBone_lerp(
        //    virtual_bone.data(),
        //    virtual_bone_localdir.data() + frame * 4,
        //    virtual_bone_localfor.data() + frame * 4,
        //    aBone, vec_bvh_channel_info,
        //    vec_bvh_time_series_data.data() + frame * vec_bvh_channel_info.size(), true, virtual_face, forwardQuat,lerp_t);
    }
    else
    {

        dfm2::CQuatd diffRotation;
        diffRotation.x = virtual_bone_localdir[frame * 4 + 0];
        diffRotation.y = virtual_bone_localdir[frame * 4 + 1];
        diffRotation.z = virtual_bone_localdir[frame * 4 + 2];
        diffRotation.w = virtual_bone_localdir[frame * 4 + 3];
        dfm2::CQuatd newFace;
        dfm2::QuatQuat(newFace.p, forwardQuat.p, diffRotation.p);

        dfm2::CVec3d localPosTemp = { virtual_bone_localpos[frame * 2 + 0] , 0.0f, virtual_bone_localpos[frame * 2 + 1] };
        dfm2::CVec3d localPosDir;
        dfm2::QuatVec(localPosDir.p, newFace.p, localPosTemp.p);
        virtual_bone.x += localPosDir.x;
        virtual_bone.y += localPosDir.z;


        //SetPose_BioVisionHierarchy_VirtualBone(
        //    virtual_bone.data(),
        //    virtual_bone_localdir.data() + frame * 4,
        //    virtual_bone_localfor.data() + (frame-1) * 4,
        //    aBone, vec_bvh_channel_info,
        //    vec_bvh_time_series_data.data() + frame * vec_bvh_channel_info.size(), false, virtual_face, forwardQuat);
        if(useInterpolate)
            SetPose_BioVisionHierarchy_VirtualBone_lerp(
                virtual_bone.data(),
                virtual_bone_localdir.data() + frame * 4,
                virtual_bone_localfor.data() + frame * 4,
                aBone, vec_bvh_channel_info,
                vec_bvh_time_series_data.data() + frame * vec_bvh_channel_info.size(), false, virtual_face, forwardQuat, lerp_t);
        else
            SetPose_BioVisionHierarchy_VirtualBone(
                virtual_bone.data(),
                virtual_bone_localdir.data() + frame * 4,
                virtual_bone_localfor.data() + (frame - 1) * 4,
                aBone, vec_bvh_channel_info,
                vec_bvh_time_series_data.data() + frame * vec_bvh_channel_info.size(), false, virtual_face, forwardQuat);
    }

    TrajectDebug(sd, mean, centerCharacter, virtual_face, vec_bvh_feature_data, frame);

    // AXIS Gizmo
    ::glBegin(GL_LINES);
    ::glColor3d(0.0f, 0.0f, 1.0f);
    ::glVertex3d(0.0f, 0.0f, 0.0f);
    ::glVertex3d(0.0f, 0.1f, 10.0f);
    ::glEnd();
    ::glLineWidth(1);

    dfm2::opengl::DrawBone_Octahedron(
        aBone,
        12, -1,
        0.1, 1.0);
}

void Remove_Root_X_Z(
    std::vector<double>& vec_bvh_time_series_data,
    int maxframe,
    size_t channel_size
)
{
    for (int i = 0; i < maxframe; i++)
    {
        vec_bvh_time_series_data[i * channel_size + 0] = 0.0f;
        vec_bvh_time_series_data[i * channel_size + 2] = 0.0f;
    }
}

void Normalize_Features(
    std::vector<double>& bvh_feature,
    int feature_size,
    std::vector<double>& mean,
    std::vector<double>& sd
)
{
    int count = bvh_feature.size() / feature_size;
    for (int i = 0; i < count; i++)
    {
        int featureIndex = i * feature_size;
        for (int j = 0; j < feature_size; j++)
        {
            bvh_feature[featureIndex + j] = (bvh_feature[featureIndex + j] - mean[j]) / sd[j];
        }
    }
}

void Init_Normalize_Factors(
    int feature_size,
    std::vector<double>& mean,
    std::vector<double>& variance,
    std::vector<double>& sd)
{
    for (int i = 0; i < feature_size; i++)
    {
        mean.push_back(0.0f);
        variance.push_back(0.0f);
        sd.push_back(0.0f);
    }
}

void Update_Normalize_Factors(
    std::vector<double>& bvh_feature,
    int feature_size,
    std::vector<double>& mean,
    std::vector<double>& variance,
    std::vector<double>& sd
)
{
    int count = bvh_feature.size() / feature_size;
    for (int i = 0; i < count; i++)
    {
        for (int j = 0; j < feature_size; j++)
        {
            mean[j] += bvh_feature[(i * feature_size) + j];
        }
    }
    for (int i = 0; i < feature_size; i++)
    {
        mean[i] /= count;
    }

    for (int i = 0; i < count; i++)
    {
        for (int j = 0; j < feature_size; j++)
        {
            double diff = bvh_feature[(i * feature_size) + j] - mean[j];
            variance[j] += diff * diff;
        }
    }
    for (int i = 0; i < feature_size; i++)
    {
        variance[i] /= count;
    }   


    // trajectory std
    int offSet[2] = { 0,6 };
    int nDimension = 6;
    for (int d = 0; d < 2; d++)
    {
        double std = 0;
        for (int j = 0; j < nDimension; j++)
        {
            std += sqrt(variance[offSet[d] + j]);
        }
        std /= nDimension;
        if (std <= 0)
            std::cout << "Error : std cannot be 0" << std::endl;
        for (int j = 0; j < nDimension; j++)
        {
            sd[offSet[d] + j] = std;
        }
    }

    // pose std
    int numPose = 5;
    int poseOffSet[5] = { 12,15,18,21,24 };
    int numFloatPerFeature = 3;
    for (int d = 0; d < numPose; d++)
    {
        double std = 0;
        for (int j = 0; j < numFloatPerFeature; j++)
        {
            std += sqrt(variance[poseOffSet[d] + j]);
        }
        std /= numFloatPerFeature;
        if (std <= 0)
            std::cout << "Error : std cannot be 0" << std::endl;
        for (int j = 0; j < numFloatPerFeature; j++)
        {
            sd[poseOffSet[d] + j] = std;
        }
    }
}

void Read_Feature(
    std::vector<dfm2::CRigBone>& aBone,
    std::vector<dfm2::CChannel_BioVisionHierarchy>& vec_bvh_channel_info,
    int num_traj_feature,
    int num_pos_feature,
    int maxframe,
    double frame_time,
    std::vector<double>& bvh_feature,
    std::vector<double>& vec_bvh_time_series_data,
    size_t channel_size,
    std::vector<double>& virtual_bone_localpos,
    std::vector<double>& virtual_bone_localdir,
    std::vector<double>& virtual_bone_face )
{
    for (int iframe = 0; iframe < maxframe - 60; iframe++)
    {
        // reset fk to current frame
        SetPose_BioVisionHierarchy(
            aBone, vec_bvh_channel_info,
            vec_bvh_time_series_data.data() + iframe * vec_bvh_channel_info.size());
        
        // separate Y rot
        dfm2::CQuatd rootRot;
        rootRot.x = aBone[0].quatRelativeRot[0];
        rootRot.y = aBone[0].quatRelativeRot[1];
        rootRot.z = aBone[0].quatRelativeRot[2];
        rootRot.w = aBone[0].quatRelativeRot[3];
        double qy[4];
        double qxz[4];
        SeparateYRot(qy, qxz,rootRot.p);
        virtual_bone_localdir.push_back(qxz[0]);
        virtual_bone_localdir.push_back(qxz[1]);
        virtual_bone_localdir.push_back(qxz[2]);
        virtual_bone_localdir.push_back(qxz[3]);

        virtual_bone_face.push_back(qy[0]);
        virtual_bone_face.push_back(qy[1]);
        virtual_bone_face.push_back(qy[2]);
        virtual_bone_face.push_back(qy[3]);
        // init joint variables
        dfm2::CVec3d cHip(aBone[0].RootPosition()[0], aBone[0].RootPosition()[1], aBone[0].RootPosition()[2]);
        dfm2::CVec3d cLeftFoot(aBone[25].RootPosition()[0], aBone[25].RootPosition()[1], aBone[25].RootPosition()[2]);
        dfm2::CVec3d cRightFoot(aBone[20].RootPosition()[0], aBone[20].RootPosition()[1], aBone[20].RootPosition()[2]);

        dfm2::CVec3d leftShoulder(aBone[14].RootPosition()[0], aBone[14].RootPosition()[1], aBone[14].RootPosition()[2]);
        dfm2::CVec3d rightShoulder(aBone[9].RootPosition()[0], aBone[9].RootPosition()[1], aBone[9].RootPosition()[2]);
        dfm2::CVec3d leftLeg(aBone[23].RootPosition()[0], aBone[23].RootPosition()[1], aBone[23].RootPosition()[2]);
        dfm2::CVec3d rightLeg(aBone[18].RootPosition()[0], aBone[18].RootPosition()[1], aBone[18].RootPosition()[2]);
        dfm2::CVec3d forward = ComputeForward(leftShoulder, rightShoulder, leftLeg, rightLeg);
        //virtual_bone_localdir.push_back(forward.x);
        //virtual_bone_localdir.push_back(forward.y);
        //virtual_bone_localdir.push_back(forward.z);

        dfm2::CVec3d centerCharacter(aBone[0].RootPosition()[0], aBone[0].RootPosition()[1], aBone[0].RootPosition()[2]);
        dfm2::CQuatd forwardQuat = LookRotation(forward, dfm2::CVec3d(0.0f, 1.0f, 0.0f));

        // init virtual root localpos vector
        if (iframe == 0) {
            virtual_bone_localpos.push_back(0.0f);
            virtual_bone_localpos.push_back(0.0f);
            dfm2::CVec3d localDiff(vec_bvh_time_series_data[(iframe + 1) * channel_size + 0] - vec_bvh_time_series_data[iframe * channel_size + 0], 0.0f, vec_bvh_time_series_data[(iframe + 1) * channel_size + 2] - vec_bvh_time_series_data[iframe  * channel_size + 2]);
            dfm2::CVec3d localPosDiff;
            ::dfm2::QuatConjVec(localPosDiff.p, forwardQuat.p, localDiff.p);
            virtual_bone_localpos.push_back(localPosDiff.x);
            virtual_bone_localpos.push_back(localPosDiff.z);
            //virtual_bone_localdir.push_back(forwardQuat.x);
            //virtual_bone_localdir.push_back(forwardQuat.y);
            //virtual_bone_localdir.push_back(forwardQuat.z);
            //virtual_bone_localdir.push_back(forwardQuat.w);
        }
        else
        {
            dfm2::CVec3d localDiff(vec_bvh_time_series_data[(iframe + 1) * channel_size + 0] - vec_bvh_time_series_data[iframe * channel_size + 0], 0.0f, vec_bvh_time_series_data[(iframe + 1) * channel_size + 2] - vec_bvh_time_series_data[iframe * channel_size + 2]);
            dfm2::CVec3d localPosDiff;
            ::dfm2::QuatConjVec(localPosDiff.p, forwardQuat.p, localDiff.p);
            virtual_bone_localpos.push_back(localPosDiff.x);
            virtual_bone_localpos.push_back(localPosDiff.z);

            //SetPose_BioVisionHierarchy(
            //    aBone, vec_bvh_channel_info,
            //    vec_bvh_time_series_data.data() + (iframe-1) * vec_bvh_channel_info.size());
            //dfm2::CQuatd p_q;
            //p_q.x = aBone[0].quatRelativeRot[0];
            //p_q.y = aBone[0].quatRelativeRot[1];
            //p_q.z = aBone[0].quatRelativeRot[2];
            //p_q.w = aBone[0].quatRelativeRot[3];


            //SetPose_BioVisionHierarchy(
            //    aBone, vec_bvh_channel_info,
            //    vec_bvh_time_series_data.data() + iframe * vec_bvh_channel_info.size());

            //dfm2::CQuatd c_q;
            //c_q.x = aBone[0].quatRelativeRot[0];
            //c_q.y = aBone[0].quatRelativeRot[1];
            //c_q.z = aBone[0].quatRelativeRot[2];
            //c_q.w = aBone[0].quatRelativeRot[3];

            //p_q.normalize();
            //c_q.normalize();

            //double invp_q[4];
            //dfm2::Inverse_Quat(invp_q,p_q.p);
            //dfm2::CQuatd q_diff;
            //dfm2::QuatQuat(q_diff.p, c_q.p, invp_q);
            //q_diff.normalize();
            //q_diff.SetSmallerRotation();
            //virtual_bone_localdir.push_back(q_diff.x);
            //virtual_bone_localdir.push_back(q_diff.y);
            //virtual_bone_localdir.push_back(q_diff.z);
            //virtual_bone_localdir.push_back(q_diff.w);

            //virtual_bone_localdir.push_back(forwardQuat.x);
            //virtual_bone_localdir.push_back(forwardQuat.y);
            //virtual_bone_localdir.push_back(forwardQuat.z);
            //virtual_bone_localdir.push_back(forwardQuat.w);
        }

        float bluecolor[3] = { 0.1f,0.1f,0.9f };

        for (int i = 1; i < 4; i++)
        {
            //set new pos
            SetPose_BioVisionHierarchy(
                aBone, vec_bvh_channel_info,
                vec_bvh_time_series_data.data() + (iframe + 20 * i) * vec_bvh_channel_info.size());

            dfm2::CVec3d futurePos(aBone[0].RootPosition()[0], aBone[0].RootPosition()[1], aBone[0].RootPosition()[2]);
            dfm2::CVec3d globalDifference = futurePos - centerCharacter;
            //dfm2::CQuatd invForward = forwardQuat.conjugate();
            dfm2::CVec3d localPosition;
            ::dfm2::QuatConjVec(localPosition.p, forwardQuat.p, globalDifference.p);
            ::glColor3fv(bluecolor);
            ::dfm2::opengl::DrawSphereAt(32, 32, 1, localPosition.x, localPosition.y + centerCharacter.y, localPosition.z);
            ::dfm2::opengl::DrawSphereAt(32, 32, 1, futurePos.x, futurePos.y, futurePos.z);
            // trajectory pos
            bvh_feature.push_back(localPosition.x);
            bvh_feature.push_back(localPosition.z);
        }

        for (int i = 1; i < 4; i++)
        {
            //set new pos
            SetPose_BioVisionHierarchy(
                aBone, vec_bvh_channel_info,
                vec_bvh_time_series_data.data() + (iframe + 20 * i) * vec_bvh_channel_info.size());

            dfm2::CVec3d futurePos(aBone[0].RootPosition()[0], aBone[0].RootPosition()[1], aBone[0].RootPosition()[2]);
            dfm2::CVec3d globalDifference = futurePos - centerCharacter;
            //dfm2::CQuatd invForward = forwardQuat.conjugate();
            dfm2::CVec3d localPosition;
            ::dfm2::QuatConjVec(localPosition.p, forwardQuat.p, globalDifference.p);
            ::glColor3fv(bluecolor);
            ::dfm2::opengl::DrawSphereAt(32, 32, 1, localPosition.x, localPosition.y + centerCharacter.y, localPosition.z);
            ::dfm2::opengl::DrawSphereAt(32, 32, 1, futurePos.x, futurePos.y, futurePos.z);

            // trajectory dir
            dfm2::CVec3d ileftShoulder(aBone[14].RootPosition()[0], aBone[14].RootPosition()[1], aBone[14].RootPosition()[2]);
            dfm2::CVec3d irightShoulder(aBone[9].RootPosition()[0], aBone[9].RootPosition()[1], aBone[9].RootPosition()[2]);
            dfm2::CVec3d ileftLeg(aBone[23].RootPosition()[0], aBone[23].RootPosition()[1], aBone[23].RootPosition()[2]);
            dfm2::CVec3d irightLeg(aBone[18].RootPosition()[0], aBone[18].RootPosition()[1], aBone[18].RootPosition()[2]);
            dfm2::CVec3d worldDir = ComputeForward(ileftShoulder, irightShoulder, ileftLeg, irightLeg);
            dfm2::CVec3d localDir;
            ::dfm2::QuatConjVec(localDir.p, forwardQuat.p, worldDir.p);
            localDir.normalize();
            bvh_feature.push_back(localDir.x);
            bvh_feature.push_back(localDir.z);
        }

        // init pose feature
        dfm2::CVec3d localLeftFootPos = cLeftFoot - cHip;
        dfm2::CVec3d localRightFootPos = cRightFoot - cHip;

        dfm2::CVec3d localLeftFoot;
        ::dfm2::QuatConjVec(localLeftFoot.p, forwardQuat.p, localLeftFootPos.p);

        dfm2::CVec3d localRightFoot;
        ::dfm2::QuatConjVec(localRightFoot.p, forwardQuat.p, localRightFootPos.p);

        // next frame info
        SetPose_BioVisionHierarchy(
            aBone, vec_bvh_channel_info,
            vec_bvh_time_series_data.data() + (iframe+1) * vec_bvh_channel_info.size());

        dfm2::CVec3d nHip(aBone[0].RootPosition()[0], aBone[0].RootPosition()[1], aBone[0].RootPosition()[2]);
        dfm2::CVec3d nLeftFoot(aBone[25].RootPosition()[0], aBone[25].RootPosition()[1], aBone[25].RootPosition()[2]);
        dfm2::CVec3d nRightFoot(aBone[20].RootPosition()[0], aBone[20].RootPosition()[1], aBone[20].RootPosition()[2]);

        dfm2::CVec3d nlocalLeftFootPos = nLeftFoot - nHip;
        dfm2::CVec3d nlocalRightFootPos = nRightFoot - nHip;

        dfm2::CVec3d nlocalLeftFoot;
        ::dfm2::QuatConjVec(nlocalLeftFoot.p, forwardQuat.p, nlocalLeftFootPos.p);

        dfm2::CVec3d nlocalRightFoot;
        ::dfm2::QuatConjVec(nlocalRightFoot.p, forwardQuat.p, nlocalRightFootPos.p);

        // leftfoot pos, left foot velo, right foot pos, right foot velo, hip velo
        dfm2::CVec3d leftFootVelo = (nlocalLeftFoot - localLeftFoot) / frame_time;
        dfm2::CVec3d rightFootVelo = (nlocalRightFoot - localRightFoot) / frame_time;

        dfm2::CVec3d hipDiff = nHip - cHip;
        dfm2::CVec3d localHipDiff;
        ::dfm2::QuatConjVec(localHipDiff.p, forwardQuat.p, hipDiff.p);
        dfm2::CVec3d hipVelo = localHipDiff / frame_time;

        bvh_feature.push_back(localLeftFoot.x);
        bvh_feature.push_back(localLeftFoot.y);
        bvh_feature.push_back(localLeftFoot.z);
        bvh_feature.push_back(leftFootVelo.x);
        bvh_feature.push_back(leftFootVelo.y);
        bvh_feature.push_back(leftFootVelo.z);

        bvh_feature.push_back(localRightFoot.x);
        bvh_feature.push_back(localRightFoot.y);
        bvh_feature.push_back(localRightFoot.z);
        bvh_feature.push_back(rightFootVelo.x);
        bvh_feature.push_back(rightFootVelo.y);
        bvh_feature.push_back(rightFootVelo.z);

        bvh_feature.push_back(hipVelo.x);
        bvh_feature.push_back(hipVelo.y);
        bvh_feature.push_back(hipVelo.z);
    }


    std::cout << "bvh feature size : " << bvh_feature.size() << std::endl;
    std::cout << "bvh feature frames : " << bvh_feature.size()/27 << std::endl;

    //for (int iframe = 0; iframe < maxframe; iframe++)
    //{
    //    for (int i = 0; i < num_traj_feature; i++)
    //    {

    //    }
    //}
}


double ComputeCost(
    int feature_size,
    const double* current_feature,
    const double* feature_database)
{
    double sqrDistance = 0.0f;
    for (int i = 0; i < feature_size; i++)
    {
        double cost = current_feature[i] - feature_database[i];
        sqrDistance += cost * cost;
    }
    return sqrDistance;
}

int ComputeBest(int currentFrame,
    dfm2::CVec2d& globalRoot,
    dfm2::CQuatd& virtual_face,
    std::vector<double>& feature_database,
    std::vector<double>& sd,
    std::vector<double>& mean,
    std::vector<dfm2::CVec2d>& predictedPos,
    std::vector<dfm2::CVec2d>& predictedDir)
{
    int target_frame = currentFrame;
    double minimal_cost = 100000;
    std::vector<double> currnet_feature;
    for (int i = 0; i < predictedPos.size(); i++)
    {
        dfm2::CVec3d localPos;
        localPos.x = predictedPos[i].x - globalRoot.x;
        localPos.y = 0.0f;
        localPos.z = predictedPos[i].y - globalRoot.y;
        dfm2::CVec3d localPosUnRot;
        dfm2::QuatConjVec(localPosUnRot.p, virtual_face.p, localPos.p);
        //::glColor3d(0.9f, 0, 0);
        //::dfm2::opengl::DrawSphereAt(32, 32, 3, localPosUnRot.x, 0.0f, localPosUnRot.z);

        double f1 = (localPosUnRot.x - mean[i * 2]) / sd[i * 2];
        double f2 = (localPosUnRot.z - mean[i * 2] + 1) / sd[i * 2 + 1];
        currnet_feature.push_back(f1);
        currnet_feature.push_back(f2);
    }
    for (int i = 0; i < predictedPos.size(); i++)
    {
        dfm2::CVec3d faceGlobal;
        faceGlobal.x = predictedDir[i].x;
        faceGlobal.y = 0.0f;
        faceGlobal.z = predictedDir[i].y;
        dfm2::CVec3d localFace;
        dfm2::QuatConjVec(localFace.p, virtual_face.p, faceGlobal.p);
        localFace.normalize();
        double f1 = (localFace.x - mean[6 + i * 2]) / sd[6 + i * 2];
        double f2 = (localFace.z - mean[6 + i * 2] + 1) / sd[6 + i * 2 + 1];
        currnet_feature.push_back(f1);
        currnet_feature.push_back(f2);
    }
    for (int i = 0; i < 15; i++)
    {
        currnet_feature.push_back(feature_database[currentFrame * 27 + 12 + i]);
    }

    for (int dbIndex = 0; dbIndex < feature_database.size() / 27; dbIndex++)
    {
        if (dbIndex == currentFrame)
            continue;
        double cost = ComputeCost(27, currnet_feature.data(), feature_database.data() + dbIndex * 27);
        if (cost < minimal_cost)
        {
            minimal_cost = cost;
            target_frame = dbIndex;
        }
    }
    return target_frame;
}

//
//template<int ndimin_pose, int ndimout_pose, int nOctave>
//void VisBvhPhase_tomcat(
//    MLP_Fourier<ndimin_pose, nOctave>& model_pose,
//    std::vector<dfm2::CRigBone>& aBone,
//    std::vector<dfm2::CRigBone>& aBone_render,
//    std::vector<dfm2::CVec3d>& feature_vectorL,
//    std::vector<dfm2::CVec3d>& feature_vectorR,
//    double* lockAffine3GlobalL,
//    double* lockAffine3GlobalR,
//    double& LerpL,
//    double* lockAffmat3GlobalLGoal,
//    const dfm2::CVec2d& root_pos2,
//    const std::vector<dfm2::CVec2d>& vec_pos2,
//    const dfm2::CVec2d& vec_dirz,
//    const double& vec_phase,
//    const std::vector<dfm2::CChannel_BioVisionHierarchy>& vec_bvh_channel_info,
//    delfem2::glfw::CViewer3& viewer2,
//    Floor& floor) {
//    //const size_t nframe = vec_pos2.size();
//    const std::vector<unsigned int> vec_id_bones_used = GetIdBoneUsed(vec_bvh_channel_info);
//
//        // ------------
//        std::vector<double> vec_input_pose;
//
//        bool res = Hoge_tomcat<ndimin_pose, ndimout_pose, nOctave>(
//            aBone, vec_input_pose, model_pose, 
//            root_pos2, vec_pos2, vec_dirz, vec_phase, vec_id_bones_used);
//
//        if (!res) { std::cout << "Error Hugotomcat" << std::endl; }
//        // ---------
//
//        //viewer2.DrawBegin_oldGL();
//        //::glEnable(GL_LINE_SMOOTH);
//        //::glEnable(GL_BLEND);
//        //::glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//        DrawFloorShadow(aBone, floor, +0.1);
//        DrawPhase(
//            vec_phase,
//            static_cast<double>(viewer2.width) / static_cast<double>(viewer2.height));
//        //    ::glLineWidth(1);
//        //    delfem2::opengl::DrawAxis(30);
//        {
//            const dfm2::CVec2d p0 = root_pos2;
//            const dfm2::CVec2d dz0 = vec_dirz;  // local z coord
//            const dfm2::CVec2d dx0(dz0.y, -dz0.x); // local x coord
//            //std::cout << "sum of square :" << dz0.x * dz0.x + dz0.y * dz0.y << std::endl;
//
//            const dfm2::CMat3d R0(
//                dx0.x, 0, dz0.x,
//                0, 1, 0,
//                dx0.y, 0, dz0.y);
//            dfm2::CQuatd q0y;
//            //R0.GetQuat_RotMatrix(q0y.p);
//            q0y = R0.GetQuaternion();
//            Encoder_Pose::draw_in(vec_input_pose, q0y, p0);
//            PostProcessRigBone(aBone, aBone_render, feature_vectorL, feature_vectorR, lockAffine3GlobalL, lockAffine3GlobalR, LerpL, lockAffmat3GlobalLGoal);
//        }
//        dfm2::CVec3d target = dfm2::CVec3d(0.0f,0.0f,0.0f);
//
//        ::glBegin(GL_LINES);
//        ::glColor3d(0.0f, 0.0f, 1.0f);
//        ::glVertex3d(0.0f, 0.0f, 0.0f);
//        ::glVertex3d(0.0f, 0.1f, 10.0f);
//        ::glEnd();
//        ::glLineWidth(1);
//
//        dfm2::opengl::DrawBone_Octahedron(
//            aBone_render,
//            12, -1,
//            0.1, 1.0);
//    
//}

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
//    double manual_phase = 0.0;
//
//    constexpr int ndim_input_pose = 21;
//    constexpr int ndim_output_pose = 183;
//    //constexpr int noctave = 1;
//    constexpr int noctave = 2;
//    constexpr int size_hidden_pose = 512;
//    const std::string mode = "ff";
//
//    std::filesystem::path path = std::filesystem::current_path();
//#if defined(WIN32) || defined(_WIN32)
//    MLP_Fourier<ndim_input_pose, noctave> model_pose(
//        std::string(PATH_SOURCE_DIR) +
//        "/../data/pose_" + mode +
//        "_no" + std::to_string(noctave) +
//        "_nh" + std::to_string(size_hidden_pose) +
//        ".pt");
//#else
//    MLP_Fourier<ndim_input_pose, noctave> model_pose(
//        path.string() +
//        "/../data/pose_" + mode +
//        "_no" + std::to_string(noctave) +
//        "_nh" + std::to_string(size_hidden_pose) +
//        ".pt");
//#endif
//
//    constexpr int ndim_input_phase = 20;
//    constexpr int num_layer_phase = 2;
//    //constexpr int num_layer_phase = 1;
//    constexpr int size_hidden_phase = 128;

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
        "/../data/jl_motionMatching_hd.bvh"
    };

    int feature_size = 27;
    std::vector<dfm2::CRigBone> vec_rig_bone;
    double lockAffine3GlobalL[16];
    double lockAffine3GlobalR[16];
    double lockAffmat3GlobalLGoal[16];
    double LerpL = 0.0f;
    std::vector<dfm2::CChannel_BioVisionHierarchy> vec_channel_bvh;
    std::vector<double> vec_phase;
    std::vector<double> real_phase;
    std::vector<dfm2::CVec2d> real_DirZ;
    std::vector<dfm2::CVec2d> aRootDirZ, aRootPos2;
    int maxFrame = 0;
    std::vector<double> vec_bvh_time_series_data;
    std::vector<double> vec_bvh_feature_data;
    std::vector<double> virtual_bone_localpos;
    std::vector<double> virtual_bone_localdir;
    std::vector<double> virtual_bone_localfor;

    std::vector<double> mean;
    std::vector<double> variance;
    std::vector<double> std;

    dfm2::CVec2d virtual_bone = {0.0f,0.0f};
    dfm2::CQuatd virtual_face;
    //virtual_bone.name = "virtualbone";
    int num_traj_feature = 3;
    int num_post_feature = 5;
    int index_highlight = 0;
    std::string directory = "./"; // Current directory
    std::string fileName = "bvh_feature.bin";
    std::string fileNameRoot = "root_local.bin";
    std::string fileNameMean = "mean_local.bin";
    std::string fileNameVar = "var_local.bin";
    std::string fileNameStd = "std_local.bin";
    std::string fileNameDir = "dir_local.bin";
    std::string fileNameFor = "for_local.bin";

    std::vector<KeyPoint> Path;
    int CurrentKeyPoint = 0;
    float CurrentKeyPointT = 0.0f;
    int NextKeyPoint;
    float NextKeyPointT;
    dfm2::CVec2d CurrentPosition;
    dfm2::CVec2d CurrentDirection;

    KeyPoint p1;
    p1.Position.x = 0.0f;
    p1.Position.y = 0.0f;
    p1.Velocity = 100.0f;
    KeyPoint p2;
    p2.Position.x = 600.0f;
    p2.Position.y = 0.0f;
    p2.Velocity = 100.0f;
    KeyPoint p3;
    p3.Position.x = 600.0f;
    p3.Position.y = 600.0f;
    p3.Velocity = 100.0f;
    KeyPoint p4;
    p4.Position.x = 0.0f;
    p4.Position.y = 600.0f;
    p4.Velocity = 100.0f;

    Path.push_back(p1);
    Path.push_back(p2);
    Path.push_back(p3);
    Path.push_back(p4);
    std::vector<int> TrajectoryPredictFrames = { 20,40,60 };
    dfm2::CVec2d start(0.0f, 0.0f);
    std::vector<dfm2::CVec2d> predictedPos;
    std::vector<dfm2::CVec2d> predictedDir;
    int discardPos;
    float discardT;
    dfm2::CVec2d globalRoot(0.0f,0.0f);

    int frameCount = 0;
    int targetFrame = 0;
    int currentFrame = 0;
    bool runMM = true;
    int ignoreSurround = 20;
    float lerp_t = 0.0f;
    bool linearInter = false;
    float interSpeed = 0.1f;

    std::chrono::duration<double, std::milli> elapsed;



    for (int i = 0; i < TrajectoryPredictFrames.size(); i++)
    {
        predictedPos.push_back(start);
        predictedDir.push_back(start);
    }
    {
        std::string path_bvh = std::string(PATH_SOURCE_DIR) + vec_path_bvh[0];
        size_t nframe = 0;
        double frame_time;
        std::string header_bvh;
        Read_BioVisionHierarchy(
            vec_rig_bone,
            vec_channel_bvh,
            nframe,
            frame_time,
            vec_bvh_time_series_data,
            header_bvh,
            path_bvh);

        if (fileExistsInDirectory(directory, fileName)) {
            std::cout << fileName << " feature file existed, loading... " << directory << std::endl;
            if (VectorDeserialize(fileName, vec_bvh_feature_data))
            {
                std::cout << "Vector loaded successfully." << std::endl;
                std::cout << "bvh feature size : " << vec_bvh_feature_data.size() << std::endl;
                std::cout << "bvh feature frames : " << vec_bvh_feature_data.size() / 27 << std::endl;
            }
            if (VectorDeserialize(fileNameRoot, virtual_bone_localpos))
                std::cout << "virtual bone Vector loaded successfully." << std::endl;
            if (VectorDeserialize(fileNameMean, mean))
                std::cout << "mean Vector loaded successfully." << std::endl;
            if (VectorDeserialize(fileNameVar, variance))
                std::cout << "variance Vector loaded successfully." << std::endl;
            if (VectorDeserialize(fileNameStd, std))
                std::cout << "std Vector loaded successfully." << std::endl;
            if (VectorDeserialize(fileNameDir, virtual_bone_localdir))
                std::cout << "virtual bone xz dir loaded successfully." << std::endl;
            if (VectorDeserialize(fileNameFor, virtual_bone_localfor))
                std::cout << "virtual bone facing loaded successfully." << std::endl;
        }
        else {
            std::cout << fileName << " feature file not existed, import manually..." << directory << std::endl;
            Init_Normalize_Factors(feature_size, mean, variance, std);
            Read_Feature(
                vec_rig_bone,
                vec_channel_bvh,
                num_traj_feature,
                num_post_feature,
                nframe,
                frame_time,
                vec_bvh_feature_data,
                vec_bvh_time_series_data,
                vec_channel_bvh.size(),
                virtual_bone_localpos,
                virtual_bone_localdir,
                virtual_bone_localfor
            );
            Update_Normalize_Factors(vec_bvh_feature_data, feature_size, mean, variance, std);
            Normalize_Features(vec_bvh_feature_data, feature_size, mean, std);
            if (VectorSerialize(fileName, vec_bvh_feature_data))
                std::cout << "feature Vector saved successfully." << std::endl;
            if (VectorSerialize(fileNameRoot, virtual_bone_localpos))
                std::cout << "virtual bone Vector saved successfully." << std::endl;
            if (VectorSerialize(fileNameMean, mean))
                std::cout << "mean Vector saved successfully." << std::endl;
            if (VectorSerialize(fileNameVar, variance))
                std::cout << "variance Vector saved successfully." << std::endl;
            if (VectorSerialize(fileNameStd, std))
                std::cout << "std Vector saved successfully." << std::endl;
            if (VectorSerialize(fileNameDir, virtual_bone_localdir))
                std::cout << "virtual bone xz dir saved successfully." << std::endl;
            if (VectorSerialize(fileNameFor, virtual_bone_localfor))
                std::cout << "virtual bone facing saved successfully." << std::endl;
        }
        std::cout << "pass read feature" << std::endl;
        Remove_Root_X_Z(vec_bvh_time_series_data, nframe, vec_channel_bvh.size());

        maxFrame = nframe;
        dfm2::SetPose_BioVisionHierarchy(
            vec_rig_bone, vec_channel_bvh,
            vec_bvh_time_series_data.data() + 0 * vec_channel_bvh.size());

        assert(vec_rig_bone.size() == 38 && vec_channel_bvh.size() == 96);
        //
        std::vector<double> aRootHeight;
        std::vector<delfem2::CMat3d> aRootRot;
        //TransRotRoot(aRootPos2, aRootHeight, aRootRot, aRootDirZ,
        //    vec_channel_bvh, vec_bvh_time_series_data);
    }
    //std::copy(vec_rig_bone[4].affmat3Global, vec_rig_bone[4].affmat3Global + 16, lockAffine3GlobalL);
    //std::copy(vec_rig_bone[4].affmat3Global, vec_rig_bone[4].affmat3Global + 16, lockAffmat3GlobalLGoal);

    //std::copy(vec_rig_bone[10].affmat3Global, vec_rig_bone[10].affmat3Global + 16, lockAffine3GlobalR);

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
      MytempViewer() : CViewer3(300) {
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
  float velocity_mag_g_run = 180.0f;
  float velocity_mag_g = 160.0f;
  //float velocity_mag_g_run = 22.0f;
  //float velocity_mag_g = 10.0f;
  float velocity_mag = 10.0f;
  dfm2::CVec2d face_dirZ(1.0f, 0.f);
  int iframe = 0;

  bool ifPressed = false;

  while (!glfwWindowShouldClose(viewer_source.window))
  {
      glfwMakeContextCurrent(viewer_source.window);

      //manual_phase += f;
      //if (manual_phase >= 1.0)
      //{
      //    manual_phase = 0.0;
      //}

      //std::cout << manual_phase << std::endl;
      bool facing_control_mode = false;


      double tempt_divs = sqrt(p.velo[0] * p.velo[0] + p.velo[1] * p.velo[1]);
      dfm2::CVec2d normal_dirZ = dfm2::CVec2d({ p.velo[0] / tempt_divs,p.velo[1] / tempt_divs });


      Floor floor{ 1000, +0.1 };


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
                  velocity_mag = damper(velocity_mag, velocity_mag_g_run, 0.1f);
                  traj_xv_goal = gamepad_x * velocity_mag;
                  traj_yv_goal = gamepad_y * velocity_mag;
              }
              else
              {
                  // walk 

                  f = damper(f, 0.006f, 0.1f);
                  //halflife = damper(halflife, 0.9, 0.1f);
                  halflife = damper(halflife, 0.5f, 0.1f);
                  velocity_mag = damper(velocity_mag, velocity_mag_g, 0.1f);
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

      int state_a = glfwGetKey(viewer_source.window, GLFW_KEY_A);
      int state_d = glfwGetKey(viewer_source.window, GLFW_KEY_D);
      int state_w = glfwGetKey(viewer_source.window, GLFW_KEY_W);
      int state_s = glfwGetKey(viewer_source.window, GLFW_KEY_S);
      if ((state_a == GLFW_PRESS) && (!ifPressed))
      {
          if (index_highlight > 0)
              index_highlight -= 1;
          ifPressed = true;
      }
      if ((state_a == GLFW_RELEASE) && (ifPressed))
          ifPressed = false;
      if ((state_d == GLFW_PRESS) && (!ifPressed))
      {
          index_highlight += 1;
          ifPressed = true;
      }
      if ((state_d == GLFW_RELEASE) && (ifPressed))
          ifPressed = false;

      //if (state_w == GLFW_PRESS)
      //    virtual_bone.y -= 1.0f;
      //if (state_s == GLFW_PRESS)
      //    virtual_bone.y += 1.0f;
      
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

      // Neural Network Model
      //VisBvhPhase_tomcat<ndim_input_pose, ndim_output_pose>(
      //    model_pose, vec_rig_bone, vec_rig_bone_render, feature_vectorL, feature_vectorR,lockAffine3GlobalL, lockAffine3GlobalR, LerpL, lockAffmat3GlobalLGoal, root_pos2,
      //    vec_pos2, face_dirZ, manual_phase, vec_channel_bvh,
      //    viewer_source, floor);

      //::glColor3d(0, 0, 0);
      //::dfm2::opengl::DrawSphereAt(32, 32, 0.5, 68.4f, 128.f, 367.f);

      if (runMM)
      {
          Path_Draw(Path);
          for (int i = 0; i < TrajectoryPredictFrames.size(); i++)
          {
              SimulatePath(0.01667f * TrajectoryPredictFrames[i], CurrentKeyPoint, CurrentKeyPointT, discardPos, discardT, predictedPos[i], predictedDir[i], Path, true);
          }
          SimulatePath(0.01667f, CurrentKeyPoint, CurrentKeyPointT, CurrentKeyPoint, CurrentKeyPointT, CurrentPosition, CurrentDirection, Path, false);
          globalRoot.x = vec_rig_bone[0].transRelative[0];
          globalRoot.y = vec_rig_bone[0].transRelative[2];

          if (iframe == 0) {
              VisBvhMotion(vec_rig_bone, vec_channel_bvh, viewer_source, vec_bvh_time_series_data, vec_bvh_feature_data, floor, iframe, index_highlight, virtual_bone_localpos, virtual_bone_localdir, virtual_bone_localfor, virtual_bone, virtual_face, std, mean, 1.0f, linearInter);
              targetFrame = ComputeBest(iframe, globalRoot, virtual_face, vec_bvh_feature_data, std, mean, predictedPos, predictedDir);
              currentFrame = targetFrame;
              VisBvhMotion(vec_rig_bone, vec_channel_bvh, viewer_source, vec_bvh_time_series_data, vec_bvh_feature_data, floor, currentFrame, index_highlight, virtual_bone_localpos, virtual_bone_localdir, virtual_bone_localfor, virtual_bone, virtual_face, std, mean, 1.0f, linearInter);
          }
          else
          {
              if (frameCount == 0)
              {
                  lerp_t = 0.0f;
                  auto start = std::chrono::high_resolution_clock::now();
                  targetFrame = ComputeBest(currentFrame, globalRoot, virtual_face, vec_bvh_feature_data, std, mean, predictedPos, predictedDir);
                  auto finish = std::chrono::high_resolution_clock::now();
                  elapsed = finish - start;
                  if(abs(targetFrame-currentFrame) > ignoreSurround)
                    currentFrame = targetFrame;
                  VisBvhMotion(vec_rig_bone, vec_channel_bvh, viewer_source, vec_bvh_time_series_data, vec_bvh_feature_data, floor, currentFrame, index_highlight, virtual_bone_localpos, virtual_bone_localdir, virtual_bone_localfor, virtual_bone, virtual_face, std, mean, lerp_t, linearInter);
              }
              else
              {
                  lerp_t += 0.2f;
                  lerp_t = std::clamp(lerp_t,0.0f,1.0f);
                  currentFrame += 1;
                  VisBvhMotion(vec_rig_bone, vec_channel_bvh, viewer_source, vec_bvh_time_series_data, vec_bvh_feature_data, floor, currentFrame, index_highlight, virtual_bone_localpos, virtual_bone_localdir, virtual_bone_localfor, virtual_bone, virtual_face, std, mean, lerp_t, linearInter);
              }
          }


          frameCount += 1;
          if (frameCount == 11)
              frameCount = 0;
      }
      iframe += 1;
      if (iframe > maxFrame)
          iframe == 0;
      floor.draw_checkerboard();
      

      //std::cout << "pass rendering" << std::endl;


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

          ImGui::Begin("Path Coordinates");
          ImGui::Text("------ Linear Interpolation ------");
          ImGui::Checkbox("Lerp", &linearInter);
          ImGui::SliderFloat("Lerp Speed", &interSpeed, 0.1f, 0.5f);
          ImGui::Text("------ Path ------");
          ImGui::InputDouble("Path[1] x", &Path[1].Position.x, 100.0, 100.0, "%.1f");
          ImGui::InputDouble("Path[1] z", &Path[1].Position.y, 100.0, 100.0, "%.1f");
          ImGui::Separator();
          ImGui::InputDouble("Path[2] x", &Path[2].Position.x, 100.0, 100.0, "%.1f");
          ImGui::InputDouble("Path[2] z", &Path[2].Position.y, 100.0, 100.0, "%.1f");
          ImGui::Separator();
          ImGui::InputDouble("Path[3] x", &Path[3].Position.x, 100.0, 100.0, "%.1f");
          ImGui::InputDouble("Path[3] z", &Path[3].Position.y, 100.0, 100.0, "%.1f");
          ImGui::Text("------------------");
          ImGui::Text("------ Speed ------");
          ImGui::InputFloat("Path[0]", &Path[0].Velocity, 10.0, 10.0, "%.1f");
          ImGui::Separator();
          ImGui::InputFloat("Path[1]", &Path[1].Velocity, 10.0, 10.0, "%.1f");
          ImGui::Separator();
          ImGui::InputFloat("Path[2]", &Path[2].Velocity, 10.0, 10.0, "%.1f");
          ImGui::Separator();
          ImGui::InputFloat("Path[3]", &Path[3].Velocity, 10.0, 10.0, "%.1f");
          ImGui::Text("------------------");
          ImGui::End();

          ImGui::Begin("Control Panel");                          // Create a window called "Hello, world!" and append into it.
          ImGui::Checkbox("XBOX Gamepad", &check_controller);
          ImGui::Checkbox("X", &buttonX); ImGui::SameLine();
          ImGui::Checkbox("Y", &buttonY); ImGui::SameLine();
          ImGui::Checkbox("A", &buttonA); ImGui::SameLine();
          ImGui::Checkbox("B", &buttonB); ImGui::SameLine();
          ImGui::Separator();
          //ImGui::Text("Bone Idnex : %d", index_highlight);
          if (ImGui::Button(runMM ? "Display" : "Off")) // Button label changes based on the boolean state
          {
              runMM = !runMM; 
          }
          ImGui::Separator();
          ImGui::InputFloat("Game Pad Trajectory Speed", &velocity_mag_g, 1.0, 1.0);
          ImGui::InputFloat("Game Pad Trajectory Run Speed", &velocity_mag_g_run, 1.0, 1.0);
          ImGui::Separator();
          //ImGui::Text("Phase Step");               // Display some text (you can use a format strings too)
          //ImGui::BulletText("Recommendation - walk: 0.006 ; run: 0.01");
          //ImGui::Separator();
          //ImGui::SliderFloat("Phase Float", &f, 0.00f, 0.015f);            // Edit 1 float using a slider from 0.0f to 1.0f
          //ImGui::ColorEdit3("clear color", (float*)&clear_color); // Edit 3 floats representing a color
          ImGui::Text("Trajectory smoothness");
          ImGui::SliderFloat("Traj Half-life", &halflife, 0.0f, 0.9f);
          ImGui::Separator();
          ImGui::Text("Current Direction");
          ImGui::Text("face x: %f ; face y: %f ; face z: %f ; face w: %f", virtual_face.x, virtual_face.y, virtual_face.z, virtual_face.w);
          ImGui::Text("Global Root X: %f ; Global Root Y: %f ", globalRoot.x, globalRoot.y);
          ImGui::Text("Predicted X[0]: %f ; Predicted Y[0]: %f ", predictedPos[0].x, predictedPos[0].y);
          ImGui::Text("Predicted X[1]: %f ; Predicted Y[1]: %f ", predictedPos[1].x, predictedPos[1].y);
          ImGui::Text("Predicted X[2]: %f ; Predicted Y[2]: %f ", predictedPos[2].x, predictedPos[2].y);

          ImGui::Text("dir x: %f ; dir y: %f", goal_x, goal_y);
          ImGui::Text("Current Speed");               // Display some text (you can use a format strings too)
          ImGui::Text("dir x: %f ; dir y: %f", speed_x, speed_y);
          ImGui::Separator();
          ImGui::Text("Query Time : %f ms", elapsed.count());
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
