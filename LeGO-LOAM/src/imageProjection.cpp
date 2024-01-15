// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <boost/circular_buffer.hpp>
#include "imageProjection.h"

ImageProjection::ImageProjection(ros::NodeHandle& nh,
                                 Channel<ProjectionOut>& output_channel)
    : _nh(nh),
      _output_channel(output_channel) {
  _sub_laser_cloud = nh.subscribe<sensor_msgs::PointCloud2>(
      "/lidar_points", 1, &ImageProjection::cloudHandler, this);

  _pub_full_cloud =
      nh.advertise<sensor_msgs::PointCloud2>("/full_cloud_projected", 1);
  _pub_full_info_cloud =
      nh.advertise<sensor_msgs::PointCloud2>("/full_cloud_info", 1);

  _pub_ground_cloud = nh.advertise<sensor_msgs::PointCloud2>("/ground_cloud", 1);
  _pub_segmented_cloud =
      nh.advertise<sensor_msgs::PointCloud2>("/segmented_cloud", 1);
  _pub_segmented_cloud_pure =
      nh.advertise<sensor_msgs::PointCloud2>("/segmented_cloud_pure", 1);
  _pub_segmented_cloud_info =
      nh.advertise<cloud_msgs::cloud_info>("/segmented_cloud_info", 1);
  _pub_outlier_cloud = nh.advertise<sensor_msgs::PointCloud2>("/outlier_cloud", 1);

  nh.getParam("/lego_loam/laser/num_vertical_scans", _vertical_scans);
  nh.getParam("/lego_loam/laser/num_horizontal_scans", _horizontal_scans);
  nh.getParam("/lego_loam/laser/vertical_angle_bottom", _ang_bottom);
  float vertical_angle_top;
  nh.getParam("/lego_loam/laser/vertical_angle_top", vertical_angle_top);

  _ang_resolution_X = (M_PI*2) / (_horizontal_scans);
  _ang_resolution_Y = DEG_TO_RAD*(vertical_angle_top - _ang_bottom) / float(_vertical_scans-1);
  _ang_bottom = -( _ang_bottom - 0.1) * DEG_TO_RAD;
  _segment_alpha_X = _ang_resolution_X;
  _segment_alpha_Y = _ang_resolution_Y;

  nh.getParam("/lego_loam/imageProjection/segment_theta", _segment_theta);
  _segment_theta *= DEG_TO_RAD;

  nh.getParam("/lego_loam/imageProjection/segment_valid_point_num",
              _segment_valid_point_num);
  nh.getParam("/lego_loam/imageProjection/segment_valid_line_num",
              _segment_valid_line_num);

  nh.getParam("/lego_loam/laser/ground_scan_index",
              _ground_scan_index);

  nh.getParam("/lego_loam/laser/sensor_mount_angle",
              _sensor_mount_angle);
  _sensor_mount_angle *= DEG_TO_RAD;

  const size_t cloud_size = _vertical_scans * _horizontal_scans;

  _laser_cloud_in.reset(new pcl::PointCloud<PointType>());
  _full_cloud.reset(new pcl::PointCloud<PointType>());
  _full_info_cloud.reset(new pcl::PointCloud<PointType>());

  _ground_cloud.reset(new pcl::PointCloud<PointType>());
  _segmented_cloud.reset(new pcl::PointCloud<PointType>());
  _segmented_cloud_pure.reset(new pcl::PointCloud<PointType>());
  _outlier_cloud.reset(new pcl::PointCloud<PointType>());

  _full_cloud->points.resize(cloud_size);
  _full_info_cloud->points.resize(cloud_size);

  std::printf("ImageProjection object initialized\n");

}

void ImageProjection::resetParameters() {
  const size_t cloud_size = _vertical_scans * _horizontal_scans;
  PointType nanPoint;
  nanPoint.x = std::numeric_limits<float>::quiet_NaN();
  nanPoint.y = std::numeric_limits<float>::quiet_NaN();
  nanPoint.z = std::numeric_limits<float>::quiet_NaN();
  nanPoint.intensity = -1;
  // 这里为何缺少设置nanPoint.intensity = -1;  而后续地面分割需要用到

  _laser_cloud_in->clear();
  _ground_cloud->clear();
  _segmented_cloud->clear();
  _segmented_cloud_pure->clear();
  _outlier_cloud->clear();

  _range_mat.resize(_vertical_scans, _horizontal_scans);
  _ground_mat.resize(_vertical_scans, _horizontal_scans);
  _label_mat.resize(_vertical_scans, _horizontal_scans);

  _range_mat.fill(FLT_MAX);
  _ground_mat.setZero();
  _label_mat.setZero();

  _label_count = 1;

  std::fill(_full_cloud->points.begin(), _full_cloud->points.end(), nanPoint);
  std::fill(_full_info_cloud->points.begin(), _full_info_cloud->points.end(), nanPoint);

  _seg_msg.startRingIndex.assign(_vertical_scans, 0);
  _seg_msg.endRingIndex.assign(_vertical_scans, 0);

  _seg_msg.segmentedCloudGroundFlag.assign(cloud_size, false);
  _seg_msg.segmentedCloudColInd.assign(cloud_size, 0);
  _seg_msg.segmentedCloudRange.assign(cloud_size, 0);
}

void ImageProjection::cloudHandler(
    const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg) {
  // Reset parameters
  std::printf("receive a laserCloudMsg\n");
  resetParameters();

  // Copy and remove NAN points
  pcl::fromROSMsg(*laserCloudMsg, *_laser_cloud_in);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*_laser_cloud_in, *_laser_cloud_in, indices);
  _seg_msg.header = laserCloudMsg->header;
  _seg_msg.header.stamp = ros::Time::now();        // 所有时间戳统一使用系统时间，防止重复
  dbg(_seg_msg.header.frame_id);

  findStartEndAngle();
  // Range image projection
  // 添加了labeled点云的平面投影
  projectPointCloud();
  // Mark ground points
  // groundRemoval();
  // Point cloud segmentation
  cloudSegmentation();
  //publish (optionally)
  publishClouds();

  std::printf("one laserCloudMsg imageProjection done\n\n");
}


void ImageProjection::projectPointCloud() {
  // range image projection
  const size_t cloudSize = _laser_cloud_in->points.size();

  for (size_t i = 0; i < cloudSize; ++i) {
    PointType thisPoint = _laser_cloud_in->points[i];

    float range = sqrt(thisPoint.x * thisPoint.x +
                       thisPoint.y * thisPoint.y +
                       thisPoint.z * thisPoint.z);

    // find the row and column index in the image for this point(雷达第几线)
    float verticalAngle = std::asin(thisPoint.z / range);
        //std::atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y));

    // 从上往下计数，-15度即为初始线0, 共16线，当前点所属行的id
    int rowIdn = (verticalAngle + _ang_bottom) / _ang_resolution_Y;
    if (rowIdn < 0 || rowIdn >= _vertical_scans) {
      continue;
    }

    // 水平方向夹角
    float horizonAngle = std::atan2(thisPoint.x, thisPoint.y);

    // 当前点所属列的id
    int columnIdn = -std::round((horizonAngle - M_PI_2) / _ang_resolution_X) + _horizontal_scans * 0.5;

    if (columnIdn >= _horizontal_scans){
      columnIdn -= _horizontal_scans;
    }

    if (columnIdn < 0 || columnIdn >= _horizontal_scans){
      continue;
    }

    if (range < 0.1){
      continue;
    }

    // 每个像素点对应位置设置为范围range值
    _range_mat(rowIdn, columnIdn) = range;
    // 根据labeled的点云(intensity field)来直接进行2D range image赋值
    if (thisPoint.intensity == 1) {    // groundLabeled
      _ground_mat(rowIdn, columnIdn) = 1;
      _ground_cloud->push_back(thisPoint);   // 将当前帧得到的地面点添加到_ground_cloud中
    }

    // intensity field被设置为row, column索引相关
    thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;

    size_t index = columnIdn + rowIdn * _horizontal_scans;
    // 按照空间位置对应索引存储的点云，方便通过range图像像素位置直接索引出3D空间点坐标
    _full_cloud->points[index] = thisPoint;    // 强度intensity field为行加列的索引
    // the corresponding range of a point is saved as "intensity"
    // 对应点的范围值被存储为强度intensity field中
    _full_info_cloud->points[index] = thisPoint;
    _full_info_cloud->points[index].intensity = range;
  }

  // extract ground cloud (_ground_mat == 1)
  // mark entry that doesn't need to label (ground and invalid point) for
  // segmentation note that ground remove is from 0~_N_scan-1, need _range_mat
  // for mark label matrix for the 16th scan
  // 地面点和非法点被标记为-1(不需要标签)
  for (size_t i = 0; i < _vertical_scans; ++i) {
    for (size_t j = 0; j < _horizontal_scans; ++j) {
      if (_ground_mat(i, j) == 1 ||
          _range_mat(i, j) == FLT_MAX) {
        _label_mat(i, j) = -1;
      }
    }
  }
  // dbg(_ground_cloud->size());
}

void ImageProjection::findStartEndAngle() {
  // start and end orientation of this cloud 从输入点云中读出一个第一个点
  auto point = _laser_cloud_in->points.front();
  // std::atan2() 返回值的范围是(-PI, PI]， 表示复数x + yi的幅角
  // _seg_Msg.startOrientation范围为(-PI, PI]
  _seg_msg.startOrientation = -std::atan2(point.y, point.x);

  // 从输入点云中读出最后一个点
  point = _laser_cloud_in->points.back();
  // _seg_msg.endOrientation范围为(PI, 3PI]
  // 因为内部雷达旋转方向原因，所以atan2()前需要添加负号
  _seg_msg.endOrientation = -std::atan2(point.y, point.x) + 2 * M_PI;

  // 将_seg_msg.endOrientation - _seg_msg.startOrientation调整为[PI, 3PI]之间
  if (_seg_msg.endOrientation - _seg_msg.startOrientation > 3 * M_PI) {
    _seg_msg.endOrientation -= 2 * M_PI;
  } else if (_seg_msg.endOrientation - _seg_msg.startOrientation < M_PI) {
    _seg_msg.endOrientation += 2 * M_PI;
  }
  // _seg_msg.orientationDiff的范围是(PI, 3PI)，一圈大小为2PI, 应该在2PI左右
  _seg_msg.orientationDiff =
      _seg_msg.endOrientation - _seg_msg.startOrientation;
}

void ImageProjection::groundRemoval() {
  // _ground_mat
  // -1, no valid info to check if ground of not
  //  0, initial value, after validation, means not ground
  //  1, ground
  for (size_t j = 0; j < _horizontal_scans; ++j) {  // colIdx
    // _ground_scan_index有config.yaml设置
    for (size_t i = 0; i < _ground_scan_index; ++i) {    // rowIdx
      size_t lowerInd = j + (i) * _horizontal_scans;
      size_t upperInd = j + (i + 1) * _horizontal_scans;

      if (_full_cloud->points[lowerInd].intensity == -1 ||
          _full_cloud->points[upperInd].intensity == -1) {
        // no info to check, invalid points
        _ground_mat(i, j) = -1;
        continue;
      }

      float dX =
          _full_cloud->points[upperInd].x - _full_cloud->points[lowerInd].x;
      float dY =
          _full_cloud->points[upperInd].y - _full_cloud->points[lowerInd].y;
      float dZ =
          _full_cloud->points[upperInd].z - _full_cloud->points[lowerInd].z;

      // 根据上下两线之间点的XYZ位置得到两线之间的俯仰角
      float vertical_angle = std::atan2(dZ , sqrt(dX * dX + dY * dY + dZ * dZ));

      // TODO: review this change

      // 如果两点所在线之间俯仰角在10度之内，则认为其是地面点
      if ( (vertical_angle - _sensor_mount_angle) <= 10 * DEG_TO_RAD) {
        _ground_mat(i, j) = 1;
        _ground_mat(i + 1, j) = 1;
      }
    }
  }

  // extract ground cloud (_ground_mat == 1)
  // mark entry that doesn't need to label (ground and invalid point) for
  // segmentation note that ground remove is from 0~_N_scan-1, need _range_mat
  // for mark label matrix for the 16th scan
  // 地面点和非法点被标记为-1(不需要标签)
  for (size_t i = 0; i < _vertical_scans; ++i) {
    for (size_t j = 0; j < _horizontal_scans; ++j) {
      if (_ground_mat(i, j) == 1 ||
          _range_mat(i, j) == FLT_MAX) {
        _label_mat(i, j) = -1;
      }
    }
  }

  // 将当前帧得到的地面点添加到_ground_cloud中
  for (size_t i = 0; i <= _ground_scan_index; ++i) {
    for (size_t j = 0; j < _horizontal_scans; ++j) {
      if (_ground_mat(i, j) == 1)
        _ground_cloud->push_back(_full_cloud->points[j + i * _horizontal_scans]);
    }
  }
}

void ImageProjection::cloudSegmentation() {
  // segmentation process
  // 对所有点进行聚类分割
  for (size_t i = 0; i < _vertical_scans; ++i) {
    for (size_t j = 0; j < _horizontal_scans; ++j) {
      // 如果labelMat[i][j] == 0 表示没有对该点进行分类，需要对该点进行聚类
      if (_label_mat(i, j) == 0) labelComponents(i, j);
    }
  }

  int sizeOfSegCloud = 0;
  // extract segmented cloud for lidar odometry
  for (size_t i = 0; i < _vertical_scans; ++i) {
    // _seg_msg.startRingIndex[i], _seg_msg.endRingIndex[i]表示第i线的点云的起始和终止序列
    // 以开始线后的第5线为开始，以结束线前的第6线为结束
    _seg_msg.startRingIndex[i] = sizeOfSegCloud - 1 + 5;

    for (size_t j = 0; j < _horizontal_scans; ++j) {
      if (_label_mat(i, j) > 0 || _ground_mat(i, j) == 1) {
        // outliers that will not be used for optimization (always continue)
        if (_label_mat(i, j) == 999999) {    // 聚类舍弃的点
          // 对于聚类数量不足的点，当列数为5的倍数时，且行数较大时，将其保存到边界外点云中，之后跳过
          if (i > _ground_scan_index && j % 5 == 0) {
            _outlier_cloud->push_back(
                _full_cloud->points[j + i * _horizontal_scans]);
            continue;
          } else {
            continue;
          }
        }
        // majority of ground points are skipped
        // 对于地面点，对于列数不为5的倍数的，直接跳过不处理
        if (_ground_mat(i, j) == 1) {
          if (j % 5 != 0 && j > 5 && j < _horizontal_scans - 5) continue;
        }
        // mark ground points so they will not be considered as edge features
        // later
        // 对于segmentedCloud进行区分是否为Ground点
        _seg_msg.segmentedCloudGroundFlag[sizeOfSegCloud] =
            (_ground_mat(i, j) == 1);
        // mark the points' column index for marking occlusion later
        _seg_msg.segmentedCloudColInd[sizeOfSegCloud] = j;
        // save range info
        _seg_msg.segmentedCloudRange[sizeOfSegCloud] =
            _range_mat(i, j);
        // save seg cloud
        _segmented_cloud->push_back(_full_cloud->points[j + i * _horizontal_scans]);
        // size of seg cloud
        ++sizeOfSegCloud;
      }
    }

    // 以结束线前的第5线为结束
    _seg_msg.endRingIndex[i] = sizeOfSegCloud - 1 - 5;
  }

  // extract segmented cloud for visualization
  // 将点云数据保存到_segmented_cloud_pure中
  for (size_t i = 0; i < _vertical_scans; ++i) {
    for (size_t j = 0; j < _horizontal_scans; ++j) {
      if (_label_mat(i, j) > 0 && _label_mat(i, j) != 999999) {
        _segmented_cloud_pure->push_back(
            _full_cloud->points[j + i * _horizontal_scans]);
        _segmented_cloud_pure->points.back().intensity =
            _label_mat(i, j);
      }
    }
  }
}

void ImageProjection::labelComponents(int row, int col) {

  const float segmentThetaThreshold = tan(_segment_theta);

  std::vector<bool> lineCountFlag(_vertical_scans, false);
  const size_t cloud_size = _vertical_scans * _horizontal_scans;
  using Coord2D = Eigen::Vector2i;
  boost::circular_buffer<Coord2D> queue(cloud_size);
  boost::circular_buffer<Coord2D> all_pushed(cloud_size);

  // BFS，以(row, col)为中心扩散，判断(row, col)是否为平面中的一点 
  queue.push_back({ row, col } );
  all_pushed.push_back({ row, col } );

  const Coord2D neighborIterator[4] = {
      {0, -1}, {-1, 0}, {1, 0}, {0, 1}};

  while (queue.size() > 0) {
    // Pop point
    Coord2D fromInd = queue.front();
    queue.pop_front();

    // Mark popped point
    // _label_count的初始值为1，后续会递增
    _label_mat(fromInd.x(), fromInd.y()) = _label_count;
    // Loop through all the neighboring grids of popped grid

    // 遍历fromInd的四个邻点
    for (const auto& iter : neighborIterator) {
      // new index
      int thisIndX = fromInd.x() + iter.x();
      int thisIndY = fromInd.y() + iter.y();
      // index should be within the boundary
      if (thisIndX < 0 || thisIndX >= _vertical_scans) {
        continue;
      }
      // at range image margin (left or right side)
      // 是环状图片，左右连通
      if (thisIndY < 0) {
        thisIndY = _horizontal_scans - 1;
      }
      if (thisIndY >= _horizontal_scans) {
        thisIndY = 0;
      }
      // prevent infinite loop (caused by put already examined point back)
      // _label_mat中，-1表示无效点，0代表未进行标记过，其余为其他标记
      // 如果当前邻点已经标记过(_label_mat被标记为正数，则跳过该点)
      if (_label_mat(thisIndX, thisIndY) != 0) {
        continue;
      }

      float d1 = std::max(_range_mat(fromInd.x(), fromInd.y()),
                    _range_mat(thisIndX, thisIndY));
      float d2 = std::min(_range_mat(fromInd.x(), fromInd.y()),
                    _range_mat(thisIndX, thisIndY));

      float alpha = (iter.x() == 0) ? _ang_resolution_X : _ang_resolution_Y;
      // 计算两点之间是否存在平面特征，如果tang越大，则d1, d2之间的差距越小, 越平坦
      float tang = (d2 * sin(alpha) / (d1 - d2 * cos(alpha)));

      if (tang > segmentThetaThreshold) {
        queue.push_back( {thisIndX, thisIndY } );

        _label_mat(thisIndX, thisIndY) = _label_count;
        lineCountFlag[thisIndX] = true;

        all_pushed.push_back(  {thisIndX, thisIndY } );
      }
    }
  }

  // check if this segment is valid
  bool feasibleSegment = false;
  // 如果聚类超过30个点，直接标记为一个可用聚类，_label_count递增
  if (all_pushed.size() >= 30) {
    feasibleSegment = true;
  } else if (all_pushed.size() >= _segment_valid_point_num) {
    // 如果聚类数量小于30大于5，统计竖直方向上的聚类点数
    int lineCount = 0;
    for (size_t i = 0; i < _vertical_scans; ++i) {
      if (lineCountFlag[i] == true) ++lineCount;
    }
    // 竖直方向上超过3个也将其标记为有效聚类
    if (lineCount >= _segment_valid_line_num) feasibleSegment = true;
  }
  // segment is valid, mark these points
  if (feasibleSegment == true) {
    ++_label_count;
  } else {  // segment is invalid, mark these points
    for (size_t i = 0; i < all_pushed.size(); ++i) {
      // 标记为999999的是需要舍弃的聚类的点，因为它们的数量少于30个且不符合竖直聚类要求
      _label_mat(all_pushed[i].x(), all_pushed[i].y()) = 999999;
    }
  }
}

void ImageProjection::publishClouds() {

  sensor_msgs::PointCloud2 temp;
  auto PublishCloud = [&](ros::Publisher& pub, const pcl::PointCloud<PointType>::Ptr& cloud) {
    sensor_msgs::PointCloud2 temp;
    pcl::toROSMsg(*cloud, temp);
    temp.header.stamp = _seg_msg.header.stamp;
    temp.header.frame_id = "base_link";
    pub.publish(temp);
  };
  PublishCloud(_pub_outlier_cloud, _outlier_cloud);
  PublishCloud(_pub_segmented_cloud, _segmented_cloud);            // 少量地面点云 + 物体点云
  PublishCloud(_pub_full_cloud, _full_cloud);
  PublishCloud(_pub_ground_cloud, _ground_cloud);                  // 地面点云
  PublishCloud(_pub_segmented_cloud_pure, _segmented_cloud_pure);
  PublishCloud(_pub_full_info_cloud, _full_info_cloud);

  _pub_segmented_cloud_info.publish(_seg_msg);
  //--------------------
  ProjectionOut out;
  out.outlier_cloud.reset(new pcl::PointCloud<PointType>());
  out.segmented_cloud.reset(new pcl::PointCloud<PointType>());

  // imageProject输出内容
  std::swap(out.seg_msg, _seg_msg);
  std::swap(out.outlier_cloud, _outlier_cloud);
  std::swap(out.segmented_cloud, _segmented_cloud);

  _output_channel.send( std::move(out) );

  // std::printf("ImageProjection::publishClouds() done\n");

}


