#ifndef GENICAMFEATURES_H
#define GENICAMFEATURES_H

#include <string>
#include <vector>
#include <unordered_map>

#include <arv.h>

namespace genicam_features {

enum FeatureType{INTEGER,FLOAT,STRING,ENUMARATION};

class FeatureProperties
{
public:
  FeatureProperties(const std::string& feature_name, const FeatureType& feature_type);
  FeatureType get_feature_type();

protected:
  std::string feature_name;
  FeatureType feature_type;
};

class BoolFeatureProperties : public FeatureProperties
{
public:
  BoolFeatureProperties(const std::string& feature_name, const FeatureType& feature_type);
  bool get_current_value(ArvDevice* pDevice);

};

class IntegerFeatureProperties : public FeatureProperties
{
public:
  IntegerFeatureProperties(const std::string& feature_name, const FeatureType& feature_type);
  int get_current_value(ArvDevice* pDevice);
  void get_value_bounds(ArvDevice* pDevice, int& min, int& max);
};

class FloatFeatureProperties : public FeatureProperties
{
public:
  FloatFeatureProperties(const std::string& feature_name, const FeatureType& feature_type);
  double get_current_value(ArvDevice* pDevice);
  void get_value_bounds(ArvDevice* pDevice, double &min, double &max);
};

class StringFeatureProperties : public FeatureProperties
{
public:
  StringFeatureProperties(const std::string& feature_name, const FeatureType& feature_type);
  std::string get_current_value(ArvDevice* pDevice);
};

class EnumerationFeatureProperties : public FeatureProperties
{
public:
  EnumerationFeatureProperties(const std::string& feature_name, const FeatureType& feature_type);
  std::string get_current_value(ArvDevice* pDevice);
  bool check_if_value_exists(ArvDevice* pDevice, const std::string& value);
};

class GenicamFeatures
{
public:
  GenicamFeatures();

  bool init(ArvDevice *pDevice);

  bool is_implemented(const std::string& feature_name);

protected:
  // methods
  void add_feature_to_map(ArvGc *pGenicam, const char* name, int level);

  // attributes
  bool isInitialized;
  std::unordered_map<std::string, FeatureProperties> features;

};

}

#endif // GENICAMFEATURES_H
