#include "sl_sensor_visualise/show_point_cloud_nodelet.hpp"

#include <pcl/conversions.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vtkRenderWindow.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <sl_sensor_calibration/calibration_data.hpp>

using namespace pcl::visualization;

namespace sl_sensor
{
namespace visualise
{
ShowPointCloudNodelet::ShowPointCloudNodelet(){};

void ShowPointCloudNodelet::onInit()
{
  // Get node handles
  nh_ = getNodeHandle();
  private_nh_ = getPrivateNodeHandle();

  // Obtain information from private node handle
  private_nh_.param<std::string>("input_topic", pc_sub_topic_, pc_sub_topic_);
  private_nh_.param<std::string>("screen_title", screen_title_, screen_title_);
  private_nh_.param<std::string>("calibration_filename", calibration_filename_, calibration_filename_);

  // Setup subscriber
  image_array_sub_ = nh_.subscribe(pc_sub_topic_, 10, &ShowPointCloudNodelet::PointCloudCb, this);

  // Create thread for main loop
  main_loop_thread_ptr_ =
      boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&ShowPointCloudNodelet::Update, this)));
};

void ShowPointCloudNodelet::PointCloudCb(const sensor_msgs::PointCloud2ConstPtr& pc_msg_ptr)
{
  boost::mutex::scoped_lock lock(mutex_);

  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*pc_msg_ptr, pcl_pc2);
  pcl::PointCloud<pcl::PointXYZI>::Ptr pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromPCLPointCloud2(pcl_pc2, *pc_ptr);

  colour_handler_ptr->setInputCloud(pc_ptr);
  if (!visualiser_ptr_->updatePointCloud(pc_ptr, *colour_handler_ptr, screen_title_))
  {
    visualiser_ptr_->addPointCloud(pc_ptr, *colour_handler_ptr, screen_title_);
  };
}

void ShowPointCloudNodelet::Update()
{
  // Setup visualiser and colour handler
  // Note: Visualiser must be initialise within the thread or else it will crash
  visualiser_ptr_ = std::make_unique<PCLVisualizer>(screen_title_, true);
  // visualiser_ptr_->getRenderWindow()->SetDoubleBuffer(1);
  // visualiser_ptr_->getRenderWindow()->SetErase(1);
  visualiser_ptr_->setShowFPS(true);

  // Camera coordinate frame
  visualiser_ptr_->addCoordinateSystem(50, "camera", 0);
  visualiser_ptr_->setCameraPosition(0, 0, -50, 0, 0, 0, 0, -1, 0);

  // Projector coordinate frame
  calibration::CalibrationData calibration_data;

  if (calibration_data.Load(calibration_filename_))
  {
    cv::Mat T_proj_cam_cv(3, 4, CV_32F);
    cv::Mat(calibration_data.Rp()).copyTo(T_proj_cam_cv.colRange(0, 3));
    cv::Mat(calibration_data.Tp()).copyTo(T_proj_cam_cv.col(3));
    Eigen::Affine3f T_proj_cam;
    cv::cv2eigen(T_proj_cam_cv, T_proj_cam.matrix());

    visualiser_ptr_->addCoordinateSystem(50, T_proj_cam.inverse(), "projector", 0);
  }

  visualiser_ptr_->setBackgroundColor(0, 0, 0);
  visualiser_ptr_->setCameraClipDistances(0.001, 10000000);

  // Initialize point cloud color handler
  colour_handler_ptr = std::make_unique<PointCloudColorHandlerGenericField<pcl::PointXYZI>>("intensity");

  while (!visualiser_ptr_->wasStopped())
  {
    {
      boost::mutex::scoped_lock lock(mutex_);
      visualiser_ptr_->spinOnce(1);
    }

    ros::Duration(0.001f).sleep();
  }
}

}  // namespace visualise
}  // namespace sl_sensor