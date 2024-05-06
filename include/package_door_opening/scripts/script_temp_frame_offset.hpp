#pragma once

#include <kdl/frames.hpp>
#include "cppitasc/coordination/script.hpp"
#include "cppitasc/kinematic_tree.hpp"
#include "cppitasc/utils/tf_publisher.hpp"

class ScriptTempFrameOffset : public Script {
public:
    explicit ScriptTempFrameOffset(const string& name);
    bool init(Dict& params) override;
    bool onStart() override;
    void onUpdate(const Tick& tick) override;
    void onStop() override;

protected:
    KinematicTree* tree_{nullptr};

    string joint_;
    string source_;  // the frame which pose is taken on startup for the new temporary frame
    string parent_;
    vector<double> offset;
    string frame_id_;  // the name of the new temporary frame
    KDL::Frame frame_;

    TfPublisher tf_publisher_;   
};
