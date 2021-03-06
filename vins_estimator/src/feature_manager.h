#ifndef FEATURE_MANAGER_H
#define FEATURE_MANAGER_H

#include <list>
#include <algorithm>
#include <vector>
#include <numeric>
using namespace std;

#include <eigen3/Eigen/Dense>
using namespace Eigen;

#include <ros/console.h>
#include <ros/assert.h>

#include "parameters.h"

/**
* @class FeaturePerFrame
* @brief 特征类
* detailed 
*/
class FeaturePerFrame
{
  public:
    //_point:[x,y,z,u,v,vx,vy]
    FeaturePerFrame(const Eigen::Matrix<double, 7, 1> &_point, double td)
    {
        point.x() = _point(0);
        point.y() = _point(1);
        point.z() = _point(2);
        uv.x() = _point(3);
        uv.y() = _point(4);
        velocity.x() = _point(5); 
        velocity.y() = _point(6); 
        cur_td = td;
    }
    double cur_td;
    Vector3d point;
    Vector2d uv;
    Vector2d velocity;
    double z;
    bool is_used;
    double parallax;
    MatrixXd A;
    VectorXd b;
    double dep_gradient;
};

/**
* @class FeaturePerId
* @brief 某feature_id下的所有FeaturePerFrame
* detailed
*/
class FeaturePerId
{
  public:
    const int feature_id; //(cxn)关键点的id（注意每个关键点的编号都是唯一的）
    int start_frame;      //(cxn)第一个看到该关键点的帧编号
    vector<FeaturePerFrame> feature_per_frame;  //关键点在每个观测到它的帧中的信息，包括 像素坐标(u,v)，归一化平面坐标(x,y,z)，在像素坐标系中速度(vx,vy)
    //(cxn)比如对于编号为2的特征点，就会为其建立这么一个FeaturePerId的对象，对象中会记录第一个看到该关键点的帧编号，
    //以及用一个vector记录  该特征点 在所有能观测到它的帧的 坐标信息，（注意，能观测到某个特征点的帧序列一定是连续的，毕竟是用光流在追）

    int used_num;
    bool is_outlier;
    bool is_margin;
    double estimated_depth;
    int solve_flag; // 0 haven't solve yet; 1 solve succ; 2 solve fail;

    Vector3d gt_p;

    FeaturePerId(int _feature_id, int _start_frame)  //以feature_id为索引，并保存了出现该角点的第一帧的id
        : feature_id(_feature_id), start_frame(_start_frame),
          used_num(0), estimated_depth(-1.0), solve_flag(0)
    {
    }

    int endFrame();
};

class FeatureManager
{
  public:
    FeatureManager(Matrix3d _Rs[]);

    void setRic(Matrix3d _ric[]);

    void clearState();

    int getFeatureCount();

    bool addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, double td);
    void debugShow();
    vector<pair<Vector3d, Vector3d>> getCorresponding(int frame_count_l, int frame_count_r);

    //void updateDepth(const VectorXd &x);
    void setDepth(const VectorXd &x);
    void removeFailures();
    void clearDepth(const VectorXd &x);
    VectorXd getDepthVector();
    void triangulate(Vector3d Ps[], Vector3d tic[], Matrix3d ric[]);
    void removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P);
    void removeBack();
    void removeFront(int frame_count);
    void removeOutlier();
    list<FeaturePerId> feature;// 通过FeatureManager可以得到滑动窗口内所有的角点信息
    int last_track_num;

  private:
    double compensatedParallax2(const FeaturePerId &it_per_id, int frame_count);
    const Matrix3d *Rs;
    Matrix3d ric[NUM_OF_CAM];
};

#endif