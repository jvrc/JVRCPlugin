/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "SceneNodeFinder.h"

using namespace std;
using namespace cnoid;

SgNode* SceneNodeFinder::find(SgNode* node, const std::string& name)
{
    foundNode = 0;
    T.setIdentity();
    traversed.clear();
    this->name = name;
    if(node){
        node->accept(*this);
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
            (*p)->accept(*this);
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
