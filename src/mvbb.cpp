/*
 * Software License Agreement (BSD License)
 *
 *   Copyright (c) 2016, Manuel Bonilla (josemanuelbonilla@gmail.com)
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of copyright holder(s) nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <mvbb/mvbb.h>

namespace mvbb
{
MVBB::MVBB(const std::string name_space)
{
    nh_ = boost::make_shared<ros::NodeHandle>(name_space);
    nh_->param<double>("cluster_tolerance", clus_tol_, 0.005);
    nh_->param<bool>("invert_z_projection", invert_, false);
    nh_->param<int>("cluster_min_size", min_size_, 1);
    nh_->param<int>("cluster_max_size", max_size_, 10000000);
    nh_->param<std::string>("input_topic", topic_, "/pacman_vision/processed_scene");
    nh_->param<std::string>("reference_frame", frame_, "/world");
    sub_ = nh_->subscribe(nh_->resolveName(topic_), 1, &MVBB::cbCloud, this);
    sub_req = nh_->subscribe("ask_for_boxes", 1, &MVBB::subCallback, this);
    std::vector<double> trasl, rot;
    nh_->param<std::vector<double> >("translation", trasl, {0, 0, 0});
    nh_->param<std::vector<double> >("rotation", rot, {0, 0, 0, 1});
    delta_trans_.setOrigin(tf::Vector3(trasl[0], trasl[1], trasl[2]));
    delta_trans_.setRotation(tf::Quaternion(rot[0], rot[1], rot[2], rot[3]));
    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("world", "/rviz_visual_markers"));
}

void MVBB::subCallback(const std_msgs::Float64::ConstPtr &msg)
{

    nh_->param<int>("cluster_min_size", min_size_, 1000);
    nh_->param<int>("cluster_max_size", max_size_, 10000);
    visual_tools_->deleteAllMarkers();
    if (msg->data == 0.0)
    {
        return;
    }
    std::vector<std::pair<Eigen::Affine3d, std::vector<double>>> info_boxes_local = info_boxes;
    std::sort(info_boxes_local.begin(), info_boxes_local.end(), [](std::pair<Eigen::Affine3d, std::vector<double>> first, std::pair<Eigen::Affine3d, std::vector<double>> second) {
        double i = first.second[2];
        double j = second.second[2];
        return (i < j);
    });
    int i = 0;
    for (auto& b : info_boxes_local)
    {
        std::vector<double> hwl = b.second;
        visual_tools_->publishWireframeCuboid(b.first, hwl[0], hwl[1], hwl[2], rviz_visual_tools::BLUE, std::string("Box_") + std::to_string(i), i);
        i++;
    }
    visual_tools_->triggerBatchPublish();
    ROS_INFO_STREAM("Publishing " << i << " Boxes");
    ros::spinOnce();
}

void MVBB::spinOnce()
{
    ros::spinOnce();
    cbSlice();
    if (!transf_.empty()) {
        std::size_t n(0);
        for (const auto &t : transf_)
        {
            brcaster_.sendTransform(tf::StampedTransform(
                                        t, ros::Time::now(),
                                        frame_,
                                        names_[n]));
            ++n;
            ros::spinOnce();
        }
    }
}

void MVBB::cbCloud(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::fromROSMsg (*msg, *cloud_);

    cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    sensor_msgs::PointCloud msg_conv, msg1;
    sensor_msgs::PointCloud2 msg2;
    sensor_msgs::convertPointCloud2ToPointCloud(*msg, msg1);
    listener_.transformPointCloud(frame_, msg1, msg_conv);
    sensor_msgs::convertPointCloudToPointCloud2(msg_conv, msg2);
    pcl::fromROSMsg (msg2, *cloud_);
}

bool MVBB::cbSlice()
{
    if (!cloud_) {
        ROS_ERROR("[MVBB::%s]\tNo cloud available, check subscriber!", __func__);
        return false;
    }
    nh_->param<int>("cluster_min_size", min_size_, 1000);
    nh_->param<int>("cluster_max_size", max_size_, 10000);
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ece;
    pcl::IndicesClustersPtr clusters = boost::make_shared<pcl::IndicesClusters>();
    ece.setInputCloud(cloud_);
    ece.setClusterTolerance(clus_tol_);
    ece.setMinClusterSize(min_size_);
    ece.setMaxClusterSize(max_size_);
    ece.extract(*clusters);
    ROS_DEBUG_STREAM("[MVBB::" << __func__ << "]\tFound " << clusters->size() << " Boxes!, from a PointCloud of " << cloud_->points.size() << " points");;
    transf_.clear();
    names_.clear();
    info_boxes.clear();
    int i(0);

    for (const auto &cl : *clusters)
    {

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        pcl::copyPointCloud(*cloud_, cl, *cluster);

        // Compute principal directions
        Eigen::Vector4f pcaCentroid;
        pcl::compute3DCentroid(*cluster, pcaCentroid);
        Eigen::Matrix3f covariance;
        computeCovarianceMatrixNormalized(*cluster, pcaCentroid, covariance);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
        Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
        eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));  /// This line is necessary for proper orientation in some cases. The numbers come out the same without it, but
        /* // Note that getting the eigenvectors can also be obtained via the PCL PCA interface with something like:
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PCA<pcl::PointXYZ> pca;
        pca.setInputCloud(cluster);
        pca.project(*cluster, *cloudPCAprojection);
        std::cerr << std::endl << "EigenVectors: " << pca.getEigenVectors() << std::endl;
        std::cerr << std::endl << "EigenValues: " << pca.getEigenValues() << std::endl;
        // In this case, pca.getEigenVectors() gives similar eigenVectors to eigenVectorsPCA.
        */

// Transform the original cloud to the origin where the principal components correspond to the axes.
        Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
        projectionTransform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
        projectionTransform.block<3, 1>(0, 3) = -1.f * (projectionTransform.block<3, 3>(0, 0) * pcaCentroid.head<3>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::transformPointCloud(*cluster, *cloudPointsProjected, projectionTransform);
// Get the minimum and maximum points of the transformed cloud.
        pcl::PointXYZRGB minPoint, maxPoint;
        pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
        const Eigen::Vector3f meanDiagonal = 0.5f * (maxPoint.getVector3fMap() + minPoint.getVector3fMap());

// Final transform
        const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
        const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

        names_.push_back(std::string("Box_" + std::to_string(i)));
        //fill in transforms

        tf::Transform t(tf::Quaternion(bboxQuaternion.x(), bboxQuaternion.y(), bboxQuaternion.z(), bboxQuaternion.w()), tf::Vector3(bboxTransform[0], bboxTransform[1], bboxTransform[2]));
        transf_.push_back(t);

        Eigen::Quaterniond q_box(bboxQuaternion.w(), bboxQuaternion.x(), bboxQuaternion.y(), bboxQuaternion.z());
        Eigen::Translation3d t_box(bboxTransform[0], bboxTransform[1], bboxTransform[2]);
        Eigen::Affine3d pose_box = Eigen::Affine3d::Identity() * t_box * q_box;
        double h = maxPoint.x - minPoint.x;
        double w = maxPoint.y - minPoint.y;
        double l = maxPoint.z - minPoint.z;;
        info_boxes.push_back(std::make_pair(pose_box, std::vector<double> {h, w, l}));
        ++i;

    }

    return true;
}

} //End namespace
