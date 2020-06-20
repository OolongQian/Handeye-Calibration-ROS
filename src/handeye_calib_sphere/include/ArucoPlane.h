#ifndef ARUCOPLANE_H
#define ARUCOPLANE_H

#include "CommonInclude.h"

using namespace std;
using namespace cv;

namespace handeye
{

class ArucoPlane
{
public:
    
    ArucoPlane (float tag_len, float plane_len);

    /** Member Variables */
    float TAG_SIDE_LEN; //meter
    float PLANE_SIDE_LEN;
    float GAP_LEN;
    float len1;
    float len2;
    float len3;
    vector<Eigen::Vector3f> tagPositions;
    vector<Eigen::Vector4f> cubeVertexInWorld;

    double _m_fx;
    double _m_fy;
    double _m_px;
    double _m_py;

    cv::Mat _cameraMatrix;
    cv::Mat _distCoeffs;
    cv::Mat _rvec, _tvec;

    Eigen::Matrix<float,3,4> _cameraIntrinsic;
    Eigen::Matrix3d _rotation;
    Eigen::Vector3d _translation;
    Eigen::Matrix4d _transform;

    void setCameraIntrinsic (cv::Mat& cameraMatrix, cv::Mat& distCoeffs);

    bool calculateExtrinsicFromImage(cv::Mat& _colorImg);

    void drawingCube(cv::Mat& _tempImg);
    void drawingAxis(cv::Mat& _tempImg);

    Eigen::Matrix4d getTransform();

    float getCubeSide();
    float getCubeHight();

    void buildTransformMatrix (const cv::Mat& rvec,
                               const cv::Mat& tvec,
                               Eigen::Matrix3d& rotation,
                               Eigen::Vector3d& translation,
                               Eigen::Matrix4d& transform);

};


}


#endif // CUBEECTRACTOR_H
