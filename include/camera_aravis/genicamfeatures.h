#ifndef GENICAMFEATURES_H
#define GENICAMFEATURES_H

#include <string>
#include <vector>
#include <unordered_map>
#include <memory>

#include <ros/ros.h>
#include <arv.h>

namespace genicam_features {

enum FeatureType{BOOL,INTEGER,FLOAT,STRING,ENUMARATION};

class FeatureProperties
{
public:
  FeatureProperties() = default;
  FeatureProperties(const std::string& feature_name, const FeatureType& feature_type);
  FeatureType get_feature_type();

  virtual std::string get_current_value(ArvDevice* pDevice);
  virtual bool set_current_value(ArvDevice* pDevice, const std::string& value);

protected:
  std::string feature_name;
  FeatureType feature_type;
};

class GenicamFeatures
{
public:
  GenicamFeatures();

  bool init(std::shared_ptr<ros::NodeHandle>& phNode, ArvDevice *pDevice, std::string serial_number);

  bool is_implemented(const std::string& feature_name);

  std::shared_ptr<FeatureProperties> get_feature(const std::string& feature_name);

  void write_features_from_rosparam(std::shared_ptr<ros::NodeHandle>& phNode, ArvDevice *pDevice, std::string serial_number);

protected:
  // methods
  void add_feature_to_map(ArvGc *pGenicam, const char* name, int level);

  // attributes
  bool isInitialized;
  std::unordered_map<std::string, std::shared_ptr<FeatureProperties>> features;
};

}

#endif // GENICAMFEATURES_H
