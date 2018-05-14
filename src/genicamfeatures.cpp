#include "camera_aravis/genicamfeatures.h"

namespace genicam_features {

// methods of class FeatureProperites
FeatureProperties::FeatureProperties(const std::string &feature_name, const FeatureType &feature_type)
{
  this->feature_type = feature_type;
}

FeatureType FeatureProperties::get_feature_type()
{
  return feature_type;
}

//methods of class BoolFeatureProperties

BoolFeatureProperties::BoolFeatureProperties(const std::string &feature_name, const FeatureType &feature_type) :
  FeatureProperties(feature_name, feature_type)
{

}

bool BoolFeatureProperties::get_current_value(ArvDevice *pDevice)
{
  return bool(arv_device_get_integer_feature_value(pDevice, feature_name.c_str()));
}

//methods of class IntegerFeatureProperties

IntegerFeatureProperties::IntegerFeatureProperties(const std::string &feature_name, const FeatureType &feature_type) :
  FeatureProperties(feature_name, feature_type)
{

}

int IntegerFeatureProperties::get_current_value(ArvDevice *pDevice)
{
  return int(arv_device_get_integer_feature_value(pDevice, feature_name.c_str()));
}

void IntegerFeatureProperties::get_value_bounds(ArvDevice *pDevice, int &min, int &max)
{
  gint64 gmin, gmax;
  arv_device_get_integer_feature_bounds(pDevice, feature_name.c_str(), &gmin, &gmax);
  min = int(gmin);
  max = int(gmax);
}

//methods of class FloatFeatureProperties
FloatFeatureProperties::FloatFeatureProperties(const std::string &feature_name, const FeatureType &feature_type) :
  FeatureProperties(feature_name, feature_type)
{

}

double FloatFeatureProperties::get_current_value(ArvDevice *pDevice)
{
  return double(arv_device_get_float_feature_value(pDevice, feature_name.c_str()));
}

void FloatFeatureProperties::get_value_bounds(ArvDevice *pDevice, double &min, double &max)
{
  arv_device_get_float_feature_bounds(pDevice, feature_name.c_str(), &min, &max);
}

//methods of class StringFeatureProperties
StringFeatureProperties::StringFeatureProperties(const std::string &feature_name, const FeatureType &feature_type) :
  FeatureProperties(feature_name, feature_type)
{

}

std::string StringFeatureProperties::get_current_value(ArvDevice *pDevice)
{
  return std::string(arv_device_get_string_feature_value(pDevice, feature_name.c_str()));
}

//methods of class EnumerationFeatureProperties
EnumerationFeatureProperties::EnumerationFeatureProperties(const std::string &feature_name, const FeatureType &feature_type) :
  FeatureProperties(feature_name, feature_type)
{

}

std::string EnumerationFeatureProperties::get_current_value(ArvDevice *pDevice)
{
  return std::string(arv_device_get_string_feature_value(pDevice, feature_name.c_str()));
}

bool EnumerationFeatureProperties::check_if_value_exists(ArvDevice* pDevice, const std::string& value)
{
  guint size = 0;
  const char** enum_list = arv_device_get_available_enumeration_feature_values_as_strings(pDevice, feature_name.c_str(), &size);
  for(guint i=0; i<size; i++)
  {
    if(value.compare(std::string(enum_list[i]))==0)
    {
      return true;
    }
  }

  return false;
}

// methods of class GenicamFeatures

// public methods
GenicamFeatures::GenicamFeatures()
{
  isInitialized = false;
}

bool GenicamFeatures::init(ArvDevice *pDevice)
{
  ArvGc	*pGenicam=NULL;
  pGenicam = arv_device_get_genicam(pDevice);

  if(!pGenicam){
    return false;
  }

  add_feature_to_map(pGenicam, "Root", 0);
  isInitialized = true;
  return true;
}

bool GenicamFeatures::is_implemented(const std::string &feature_name)
{
  auto iter = features.find(feature_name);
  if(iter != features.end())
    return true;
  else
    return false;
}


// protected methods
void GenicamFeatures::add_feature_to_map(ArvGc *pGenicam, const char* name, int level)
{
  ArvGcNode *node;

  node = arv_gc_get_node(pGenicam, name);
  if (ARV_IS_GC_FEATURE_NODE(node) &&
      arv_gc_feature_node_is_implemented(ARV_GC_FEATURE_NODE(node), NULL)) {

    if (ARV_IS_GC_CATEGORY(node)) {
      const GSList *features;
      const GSList *iter;

      features = arv_gc_category_get_features(ARV_GC_CATEGORY(node));

      for (iter = features; iter != NULL; iter = iter->next)
        add_feature_to_map(pGenicam, (char*) iter->data, level + 1);

    }
    else if(ARV_IS_GC_INTEGER(node))
    {
      std::string name = arv_gc_feature_node_get_name(ARV_GC_FEATURE_NODE(node));
      features.emplace(name,
        IntegerFeatureProperties(name, FeatureType::INTEGER));
    }
    else if (ARV_IS_GC_FLOAT(node))
    {
      std::string name = arv_gc_feature_node_get_name(ARV_GC_FEATURE_NODE(node));
      features.emplace(name,
        FloatFeatureProperties(name, FeatureType::FLOAT));
    }
    else if(ARV_IS_GC_STRING(node))
    {
      std::string name = arv_gc_feature_node_get_name(ARV_GC_FEATURE_NODE(node));
      features.emplace(name,
        StringFeatureProperties(name, FeatureType::STRING));
    }
    else if (ARV_IS_GC_ENUMERATION(node)) {
      std::string name = arv_gc_feature_node_get_name(ARV_GC_FEATURE_NODE(node));
      features.emplace(name,
        EnumerationFeatureProperties(name, FeatureType::ENUMARATION));
    }
  }
}

}
