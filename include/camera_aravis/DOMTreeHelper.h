#ifndef CAMERA_ARAVIS_DOMTREEHELPER
#define CAMERA_ARAVIS_DOMTREEHELPER

#include <arv.h>
#include <string.h>

#include <ros/ros.h>

typedef struct
{
  const char* szName;
  const char* szTag;
  ArvDomNode* pNode;
  ArvDomNode* pNodeSibling;
} NODEEX;

// Get the child and the child's sibling, where <p___> indicates an indirection.
NODEEX GetGcFirstChild(ArvGc* pGenicam, NODEEX nodeex)
{
  const char* szName = 0;

  if (nodeex.pNode)
  {
    nodeex.pNode = arv_dom_node_get_first_child(nodeex.pNode);
    if (nodeex.pNode)
    {
      nodeex.szName = arv_dom_node_get_node_name(nodeex.pNode);
      nodeex.pNodeSibling = arv_dom_node_get_next_sibling(nodeex.pNode);

      // Do the indirection.
      if (nodeex.szName[0] == 'p' && strcmp("pInvalidator", nodeex.szName))
      {
        szName = arv_dom_node_get_node_value(
            arv_dom_node_get_first_child(nodeex.pNode));
        nodeex.pNode = (ArvDomNode*)arv_gc_get_node(pGenicam, szName);
        nodeex.szTag = arv_dom_node_get_node_name(nodeex.pNode);
      }
      else
      {
        nodeex.szTag = nodeex.szName;
      }
    }
    else
      nodeex.pNodeSibling = NULL;
  }
  else
  {
    nodeex.szName = NULL;
    nodeex.szTag = NULL;
    nodeex.pNodeSibling = NULL;
  }

  // ROS_INFO("GFC name=%s, node=%p, sib=%p", szNameChild, nodeex.pNode,
  // nodeex.pNodeSibling);

  return nodeex;
} // GetGcFirstChild()

// Get the sibling and the sibling's sibling, where <p___> indicates an
// indirection.
NODEEX GetGcNextSibling(ArvGc* pGenicam, NODEEX nodeex)
{
  const char* szName = 0;

  // Go to the sibling.
  nodeex.pNode = nodeex.pNodeSibling;
  if (nodeex.pNode)
  {
    nodeex.szName = arv_dom_node_get_node_name(nodeex.pNode);
    nodeex.pNodeSibling = arv_dom_node_get_next_sibling(nodeex.pNode);

    // Do the indirection.
    if (nodeex.szName[0] == 'p' && strcmp("pInvalidator", nodeex.szName))
    {
      szName = arv_dom_node_get_node_value(
          arv_dom_node_get_first_child(nodeex.pNode));
      nodeex.pNode = (ArvDomNode*)arv_gc_get_node(pGenicam, szName);
      nodeex.szTag =
          nodeex.pNode ? arv_dom_node_get_node_name(nodeex.pNode) : NULL;
    }
    else
    {
      nodeex.szTag = nodeex.szName;
    }
  }
  else
  {
    nodeex.szName = NULL;
    nodeex.szTag = NULL;
    nodeex.pNodeSibling = NULL;
  }

  // ROS_INFO("GNS name=%s, node=%p, sib=%p", nodeex.szName, nodeex.pNode,
  // nodeex.pNodeSibling);

  return nodeex;
} // GetGcNextSibling()

// Walk the DOM tree, i.e. the tree represented by the XML file in the camera,
// and that contains all the various features, parameters, etc.
void PrintDOMTree(ArvGc* pGenicam, NODEEX nodeex, int nIndent)
{
  char* szIndent = 0;
  const char* szFeature = 0;
  const char* szDomName = 0;
  const char* szFeatureValue = 0;

  szIndent = new char[nIndent + 1];
  memset(szIndent, ' ', nIndent);
  szIndent[nIndent] = 0;

  nodeex = GetGcFirstChild(pGenicam, nodeex);
  if (nodeex.pNode)
  {
    do
    {
      if (ARV_IS_GC_FEATURE_NODE((ArvGcFeatureNode*)nodeex.pNode))
      {
        szDomName = arv_dom_node_get_node_name(nodeex.pNode);
        szFeature =
            arv_gc_feature_node_get_name((ArvGcFeatureNode*)nodeex.pNode);
        szFeatureValue = arv_gc_feature_node_get_value_as_string(
            (ArvGcFeatureNode*)nodeex.pNode, NULL);
        if (szFeature && szFeatureValue && szFeatureValue[0])
          ROS_INFO("FeatureName: %s%s, %s=%s", szIndent, szDomName, szFeature,
                   szFeatureValue);
      }
      PrintDOMTree(pGenicam, nodeex, nIndent + 4);

      // Go to the next sibling.
      nodeex = GetGcNextSibling(pGenicam, nodeex);

    } while (nodeex.pNode && nodeex.pNodeSibling);
  }
} // PrintDOMTree()

#endif //CAMERA_ARAVIS_DOMTREEHELPER
