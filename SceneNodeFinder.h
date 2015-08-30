/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_JVRC_PLUGIN_SCENE_NODE_FINDER_H
#define CNOID_JVRC_PLUGIN_SCENE_NODE_FINDER_H

#include <cnoid/SceneVisitor>

namespace cnoid {

class SceneNodeFinder : public SceneVisitor
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    SgNode* find(SgNode* node, const std::string& name);
    const Affine3& position() const { return position_; }

protected:
    virtual void visitNode(SgNode* node);
    virtual void visitGroup(SgGroup* group);
    virtual void visitTransform(SgTransform* transform);

private:
    SgNode* foundNode;
    Affine3 T;
    Affine3 position_;
    std::string name;
    std::set<SgNode*> traversed;
};

}

#endif
