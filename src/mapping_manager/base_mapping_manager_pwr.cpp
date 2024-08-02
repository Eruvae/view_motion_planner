//////////////////////////////////////
#include <view_motion_planner/mapping_manager/base_mapping_manager_pwr.h>
//////////////////////////////////////
#include <view_motion_planner/vmp_utils.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace view_motion_planner
{

void BaseMappingManagerPwr::updateMap(double max_wait)
{
  pointcloud_roi_msgs::PointcloudWithRoiConstPtr msg = ros::topic::waitForMessage<pointcloud_roi_msgs::PointcloudWithRoi>("/detect_roi/results", nh, ros::Duration(max_wait));
  if (msg == nullptr)
  {
    ROS_WARN("updateMap: ros::topic::waitForMessage failed!");
    return;
  }

  geometry_msgs::Transform pc_transform;
  if (msg->cloud.header.frame_id != map_frame)
  {
    try
    {
      geometry_msgs::TransformStamped tmp_ = tfBuffer.lookupTransform(map_frame, msg->cloud.header.frame_id, msg->cloud.header.stamp);
      pc_transform = tmp_.transform;
    }
    catch (const tf2::TransformException &e)
    {
      ROS_ERROR_STREAM("Couldn't find transform to map frame in registerPointcloudWithRoi: " << e.what());
      return;
    }
  }
  else
  {
    pc_transform.rotation.x = 0.0;
    pc_transform.rotation.y = 0.0;
    pc_transform.rotation.z = 0.0;
    pc_transform.rotation.w = 1.0;
    pc_transform.rotation.x = 0.0;
    pc_transform.rotation.y = 0.0;
    pc_transform.rotation.z = 0.0;
  }
  
  bool result = registerPointcloudWithRoi(msg, pc_transform);
  if (result)
  {
    octomap::point3d sr_min_ws(config.sr_min_x, config.sr_min_y, config.sr_min_z);
    octomap::point3d sr_max_ws(config.sr_max_x, config.sr_max_y, config.sr_max_z);

    octomap::point3d sr_min_map = transformToFrame(sr_min_ws, ws_frame, map_frame, tfBuffer);
    octomap::point3d sr_max_map = transformToFrame(sr_max_ws, ws_frame, map_frame, tfBuffer);

    updateTargets(sr_min_map, sr_max_map);

    // Publish targets
    if (roi_targets_pub.getNumSubscribers() > 0)
    {
      auto roi_targets = getRoiTargets();
      auto roi_targets_msg = targetsToPointCloud2Msg<pcl::PointXYZ>(*roi_targets, map_frame);
      //auto roi_targets_msg = targetsToPointCloud2Msg<pcl::PointXYZRGB>(*roi_targets, map_frame, 255, 0 , 0); // RED
      roi_targets_pub.publish(roi_targets_msg);
    }
    if (expl_targets_pub.getNumSubscribers() > 0)
    {
      auto expl_targets = getExplTargets();
      auto expl_targets_msg = targetsToPointCloud2Msg<pcl::PointXYZ>(*expl_targets, map_frame);
      //auto expl_targets_msg = targetsToPointCloud2Msg<pcl::PointXYZRGB>(*expl_targets, map_frame, 0, 255 , 0); // GREEN
      expl_targets_pub.publish(expl_targets_msg);
    }
    if (border_targets_pub.getNumSubscribers() > 0)
    {
      auto border_targets = getBorderTargets();
      auto border_targets_msg = targetsToPointCloud2Msg<pcl::PointXYZ>(*border_targets, map_frame);
      //auto border_targets_msg = targetsToPointCloud2Msg<pcl::PointXYZRGB>(*border_targets, map_frame, 0, 0, 255); // BLUE
      border_targets_pub.publish(border_targets_msg);
    }

    //need_to_publish_map = need_to_publish_map || result;
  }
}

} // namespace view_motion_planner