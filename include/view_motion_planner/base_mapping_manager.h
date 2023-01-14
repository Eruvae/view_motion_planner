#ifndef _BASE_MAPPING_MANAGER__
#define _BASE_MAPPING_MANAGER__


class BaseMappingManager {
   public:
      virtual void registerPointcloudWithRoi(const pointcloud_roi_msgs::PointcloudWithRoiConstPtr &msg) = 0;
      virtual void waitForPointcloudWithRoi() = 0;
      virtual std::string saveToFile(const std::string &name, bool name_is_prefix) = 0;
      virtual int loadFromFile(const std::string &filename, const geometry_msgs::Transform &offset = geometry_msgs::Transform()) = 0;
      virtual void resetMap() = 0;
      virtual void publishMap() = 0;

      // ;_;
      virtual std::vector<ViewposePtr> sampleObservationPoses(double sensorRange) = 0;
      virtual void updateRoiTargets() = 0;
      virtual void updateExplTargets() = 0;
      // .........

};



#endif // _BASE_MAPPING_MANAGER__