/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "SceneNodeFinder.h"

using namespace std;
using namespace cnoid;

SceneNodeFinder::SceneNodeFinder()
{
    setFunction<SgNode>([&](SgNode* node){ visitNode(node); });
    setFunction<SgGroup>([&](SgGroup* group){ visitGroup(group); });
    setFunction<SgTransform>([&](SgTransform* transform){ visitTransform(transform); });
    updateDispatchTable();
}


SgNode* SceneNodeFinder::find(SgNode* node, const std::string& name)
{
    foundNode = 0;
    T.setIdentity();
    traversed.clear();
    this->name = name;
    if(node){
        dispatch(node);
    }
    return foundNode;
}


void SceneNodeFinder::visitNode(SgNode* node)
{
    if(node->name() == name){
        foundNode = node;
        position_ = T;
    }
}


void SceneNodeFinder::visitGroup(SgGroup* group)
{
    visitNode(group);

    if(foundNode){
        return;
    }
    
    for(SgGroup::const_iterator p = group->begin(); p != group->end(); ++p){
        if(traversed.insert(*p).second){
            dispatch(*p);
        } else {
            continue;
        }
    }
}


void SceneNodeFinder::visitTransform(SgTransform* transform)
{
    Affine3 T0 = T;
    Affine3 U;
    transform->getTransform(U);
    T = T * U;
    visitGroup(transform);
    T = T0;
}
