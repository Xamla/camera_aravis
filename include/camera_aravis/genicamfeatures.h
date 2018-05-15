#ifndef GENICAMFEATURES_H
#define GENICAMFEATURES_H

#include <string>
#include <vector>
#include <unordered_map>

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
  virtual void set_current_value(ArvDevice* pDevice, const std::string& value);

protected:
  std::string feature_name;
  FeatureType feature_type;
};

class GenicamFeatures
{
public:
  GenicamFeatures();

  bool init(ArvDevice *pDevice);

  bool is_implemented(const std::string& feature_name);

  FeatureProperties& get_feature(const std::string& feature_name);
protected:
  // methods
  void add_feature_to_map(ArvGc *pGenicam, const char* name, int level);

  // attributes
  bool isInitialized;

  std::unordered_map<std::string, FeatureProperties> features;
};

}

#endif // GENICAMFEATURES_H
