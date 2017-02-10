/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_JVRC_PLUGIN_SCENE_NODE_FINDER_H
#define CNOID_JVRC_PLUGIN_SCENE_NODE_FINDER_H

#include <cnoid/SceneGraph>
#include <cnoid/PolymorphicFunctionSet>

namespace cnoid {

class SceneNodeFinder : public PolymorphicFunctionSet<SgNode>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    SceneNodeFinder();
    SgNode* find(SgNode* node, const std::string& name);
    const Affine3& position() const { return position_; }

protected:
    void visitNode(SgNode* node);
    void visitGroup(SgGroup* group);
    void visitTransform(SgTransform* transform);

private:
    SgNode* foundNode;
    Affine3 T;
    Affine3 position_;
    std::string name;
    std::set<SgNode*> traversed;
};

}

#endif
