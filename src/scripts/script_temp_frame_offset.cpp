#include "pitasc_pivot_taubin/scripts/script_temp_frame_offset.hpp"

ScriptTempFrameOffset::ScriptTempFrameOffset(const string& name) : Script(name)
{
}

bool ScriptTempFrameOffset::init(Dict& params)
{
    // Kinematic Tree
    auto peer = extract_type(params["kinematic_graph"], shared_ptr<RuntimeObject>);
    tree_ = dynamic_cast<KinematicTree*>(peer.get());

    frame_id_ = extract_type(params["frame"], string);
    extract(params["source"], source_);
    extract(params["parent"], parent_);
    offset = extract_type(params["offset"], vector<double>);
    bool tf_broadcast = extract_type(params["tf_broadcast"], bool);
    tf_publisher_.setBroadcast(tf_broadcast);

    auto result = tree_->addElement(frame_id_, parent_, frame_id_ + "-" + parent_ + "-temp");
    if (result) {
        joint_ = result.value();
    } else {
        throw std::runtime_error("ScriptTempFrame: Frame '" + frame_id_ + "' already exists!");
    }

    setDependencies({});

    return Script::init(params);
}

bool ScriptTempFrameOffset::onStart()
{
    // Update pose
    if (!tree_->lookup(parent_, source_, frame_)) {
        throw std::runtime_error("ScriptTempFrame: Lookup from '" + parent_ + "' to '" + source_
                                 + "' not possible");
    }

    //add offset
    KDL::Frame offset_ = KDL::Frame(KDL::Rotation::RPY(offset[3], offset[4], offset[5]),
                                    KDL::Vector(offset[0], offset[1], offset[2]));
    frame_ = frame_ * offset_;

    tree_->setTransform(joint_, frame_);
    return Script::onStart();
}

void ScriptTempFrameOffset::onUpdate(const Tick& tick)
{
    tf_publisher_.publishFrame(frame_, parent_, frame_id_);
    return Script::onUpdate(tick);
}

void ScriptTempFrameOffset::onStop()
{
    return Script::onStop();
}

RUNTIME_COMPONENT(ScriptTempFrameOffset)