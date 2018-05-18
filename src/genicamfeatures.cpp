#include "camera_aravis/genicamfeatures.h"
#include "XmlRpc.h"
#include <cctype>

namespace genicam_features {

// bool feature class
class BoolFeatureProperties : public FeatureProperties
{
public:
  BoolFeatureProperties(const std::string& feature_name, const FeatureType& feature_type) :
    FeatureProperties(feature_name, feature_type){}

  std::string get_current_value(ArvDevice* pDevice) override
  {
    return std::to_string(arv_device_get_integer_feature_value(pDevice, feature_name.c_str()));
  }

  bool set_current_value(ArvDevice* pDevice, const std::string& value) override
  {
    try
    {
      if(iequals(value, "true") || std::stoi(value) >= 1)
      {
        arv_device_set_integer_feature_value(pDevice, feature_name.c_str(), 1);
      }
      else if(iequals(value, "false") || std::stoi(value) < 1)
      {
        arv_device_set_integer_feature_value(pDevice, feature_name.c_str(), 0);
      }
      else
        throw std::runtime_error("value could not be converted to bool");
    } catch (const std::exception& e)
    {
      ROS_WARN(("Could not set value for feature " + feature_name + " because of following exception: " + e.what()).c_str());
      return false;
    }
    return true;
  }

protected:

  bool iequals(const std::string & value1, const std::string &value2)
  {
    std::string str1 = value1;
    std::string str2 = value2;
    return ((str1.size() == str2.size()) && std::equal(str1.begin(), str1.end(), str2.begin(), [](char & c1, char & c2){
                return (c1 == c2 || std::toupper(c1) == std::toupper(c2));
                  }));
  }

};

class IntegerFeatureProperties : public FeatureProperties
{
public:
  IntegerFeatureProperties(const std::string& feature_name, const FeatureType& feature_type):
    FeatureProperties(feature_name, feature_type){}

  std::string get_current_value(ArvDevice* pDevice)
  {
    return std::to_string(arv_device_get_integer_feature_value(pDevice, feature_name.c_str()));
  }

  bool set_current_value(ArvDevice* pDevice, const std::string& value) override
  {
    gint64 valueI;
    gint64 min, max;

    try
    {
      valueI = std::stoi(value);
    } catch(const std::exception &e)
    {
      ROS_WARN(("Warning: Try to set feature" + feature_name +
                " but it was not possible to convert " + value +
                " to int. Do nothing").c_str());
      return false;
    }

    arv_device_get_integer_feature_bounds(pDevice, feature_name.c_str(), &min, &max);

    if(valueI > max)
    {
      ROS_WARN(("Boundary Warning: Try to set feature "+ feature_name
               + "to " + std::to_string(valueI) + "but max value is " +
               std::to_string(max) + ". Therefore it is set to max value").c_str());
      arv_device_set_integer_feature_value(pDevice, feature_name.c_str(), max);
    }
    else if(valueI < min)
    {
      ROS_WARN(("Boundary Warning: Try to set feature "+ feature_name
               + "to " + std::to_string(valueI) + "but min value is "
               + std::to_string(max) + ". Therefore it is set to min value").c_str());
      arv_device_set_integer_feature_value(pDevice, feature_name.c_str(), min);
    }
    else
      arv_device_set_integer_feature_value(pDevice, feature_name.c_str(), valueI);

    return true;
  }
};

class FloatFeatureProperties : public FeatureProperties
{
public:
  FloatFeatureProperties(const std::string& feature_name, const FeatureType& feature_type):
    FeatureProperties(feature_name, feature_type){}

  std::string get_current_value(ArvDevice* pDevice)
  {
    return std::to_string(arv_device_get_float_feature_value(pDevice, feature_name.c_str()));
  }

  bool set_current_value(ArvDevice* pDevice, const std::string& value) override
  {
    double valueD;
    double min, max;

    try
    {
      valueD = std::stod(value);
    } catch(const std::exception &e)
    {
      ROS_WARN(("Warning: Try to set feature" + feature_name +
                " but it was not possible to convert " + value +
                " to double. Do nothing").c_str());
      return false;
    }

    arv_device_get_float_feature_bounds(pDevice, feature_name.c_str(), &min, &max);

    if(valueD > max)
    {
      ROS_WARN(("Boundary Warning: Try to set feature "+ feature_name
               + "to " + std::to_string(valueD) + "but max value is " +
               std::to_string(max) + ". Therefore it is set to max value").c_str());
      arv_device_set_float_feature_value(pDevice, feature_name.c_str(), max);
    }
    else if(valueD < min)
    {
      ROS_WARN(("Boundary Warning: Try to set feature "+ feature_name
               + "to " + std::to_string(valueD) + "but min value is " +
               std::to_string(max) + ". Therefore it is set to min value").c_str());
      arv_device_set_float_feature_value(pDevice, feature_name.c_str(), min);
    }
    else
      arv_device_set_float_feature_value(pDevice, feature_name.c_str(), valueD);

    return true;
  }
};

class StringFeatureProperties : public FeatureProperties
{
public:
  StringFeatureProperties(const std::string& feature_name, const FeatureType& feature_type):
    FeatureProperties(feature_name, feature_type){}

  std::string get_current_value(ArvDevice* pDevice)
  {
    return std::string(arv_device_get_string_feature_value(pDevice, feature_name.c_str()));
  }

  bool set_current_value(ArvDevice* pDevice, const std::string& value) override
  {
    arv_device_set_string_feature_value(pDevice, feature_name.c_str(), value.c_str());
    return true;
  }
};

class EnumerationFeatureProperties : public FeatureProperties
{
public:
  EnumerationFeatureProperties(const std::string& feature_name, const FeatureType& feature_type):
    FeatureProperties(feature_name, feature_type){}
  std::string get_current_value(ArvDevice* pDevice)
  {
    return std::string(arv_device_get_string_feature_value(pDevice, feature_name.c_str()));
  }

  bool set_current_value(ArvDevice* pDevice, const std::string& value) override
  {
    guint i;
    guint size = 0;
    const char** enum_list = arv_device_get_available_enumeration_feature_values_as_strings(pDevice, feature_name.c_str(), &size);
    for(i=0; i<size; i++)
    {
      if(value.compare(std::string(enum_list[i]))==0)
      {
        break;
      }
    }

    if (i<size)
    {
      arv_device_set_string_feature_value(pDevice, feature_name.c_str(),value.c_str());
      return true;
    }
    else
    {
      ROS_WARN(("Enum Warning: Try to set enum feature "+ feature_name
               + " to " + value + " but this is not available. Do nothing").c_str());
    }
    return false;
  }
};


// methods of class FeatureProperites
FeatureProperties::FeatureProperties(const std::string &feature_name, const FeatureType &feature_type)
{
  this->feature_name = feature_name;
  this->feature_type = feature_type;
}

FeatureType FeatureProperties::get_feature_type()
{
  return feature_type;
}

std::string FeatureProperties::get_current_value(ArvDevice* pDevice)
{
  ROS_WARN("use get_current_value function of base class FeatureProperties");
  return "";
}

bool FeatureProperties::set_current_value(ArvDevice* pDevice, const std::string& value)
{
  ROS_WARN("use set_current_value function of base class FeatureProperties");
}


// methods of class GenicamFeatures

// public methods
GenicamFeatures::GenicamFeatures()
{
  isInitialized = false;
}

bool GenicamFeatures::init(std::shared_ptr<ros::NodeHandle> &phNode, ArvDevice *pDevice, std::string serial_number)
{
  ArvGc	*pGenicam=NULL;
  pGenicam = arv_device_get_genicam(pDevice);

  if(!pGenicam){
    return false;
  }

  add_feature_to_map(pGenicam, "Root", 0);
  isInitialized = true;
  write_features_from_rosparam(phNode, pDevice, serial_number);
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
  GType value_type;

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
    else
    {
      std::string name;
      value_type = arv_gc_feature_node_get_value_type (ARV_GC_FEATURE_NODE(node));

      switch(value_type){
      case G_TYPE_BOOLEAN:
        name = arv_gc_feature_node_get_name(ARV_GC_FEATURE_NODE(node));
        features.emplace(name, std::make_shared<BoolFeatureProperties>(name, FeatureType::BOOL));
        break;

      case G_TYPE_INT64:
        if (ARV_IS_GC_ENUMERATION(node))
        {
          name = arv_gc_feature_node_get_name(ARV_GC_FEATURE_NODE(node));
          features.emplace(name, std::make_shared<EnumerationFeatureProperties>(name, FeatureType::ENUMARATION));
        }
        else
        {
          name = arv_gc_feature_node_get_name(ARV_GC_FEATURE_NODE(node));
          features.emplace(name, std::make_shared<IntegerFeatureProperties>(name, FeatureType::INTEGER));
          break;
        }

      case G_TYPE_DOUBLE:
        name = arv_gc_feature_node_get_name(ARV_GC_FEATURE_NODE(node));
        features.emplace(name, std::make_shared<FloatFeatureProperties>(name, FeatureType::FLOAT));
        break;

      case G_TYPE_STRING:
        name = arv_gc_feature_node_get_name(ARV_GC_FEATURE_NODE(node));
        features.emplace(name, std::make_shared<StringFeatureProperties>(name, FeatureType::STRING));
        break;
      }
    }
  }
}

std::shared_ptr<FeatureProperties> GenicamFeatures::get_feature(const std::string& feature_name)
{
  return features[feature_name];
}



void GenicamFeatures::write_features_from_rosparam(std::shared_ptr<ros::NodeHandle> &phNode, ArvDevice *pDevice, std::string serial_number)
{
  XmlRpc::XmlRpcValue xmlrpcParams;

  phNode->getParam(ros::this_node::getName()+"/"+ serial_number + "/feature", xmlrpcParams);

  if (xmlrpcParams.getType() == XmlRpc::XmlRpcValue::TypeStruct)
  {
    for (auto xmlrpcParam : xmlrpcParams)
    {
      if(is_implemented(xmlrpcParam.first))
      {
        if(xmlrpcParam.second.getType() == XmlRpc::XmlRpcValue::TypeString)
        {
          features[xmlrpcParam.first]->set_current_value(pDevice, xmlrpcParam.second);
        }
        else
        {
          ROS_WARN(("write_features_from_rosparam: try to set feature " + xmlrpcParam.first +
                    " for camera " + serial_number + " but value has to be string (conversion is internally handled)").c_str());
        }
      }
      else
      {
        ROS_WARN(("write_features_from_rosparam: can set value for command" +
                 xmlrpcParam.first + " ,because it is not " +
                 "supported by camera with serial number: " + serial_number).c_str());
      }
    }
  }
}

} //end of namespace

