#pragma once

#include <kdl/frames.hpp>
#include "cppitasc/coordination/monitor.hpp"
#include "cppitasc/kinematic_tree.hpp"
#include "cppitasc/utils/operator.hpp"

class MonitorRelativeAngularDistance : public Monitor {
public:
    explicit MonitorRelativeAngularDistance(const string& name);
    bool init(Dict& params) override;

protected:
    bool onStart() override;
    void onUpdate(const Tick& /*unused*/) override;

    string ref_frame_;
    string observed_frame_;
    KDL::Frame initial_frame_;

    vector<string> coordinates_;
    string operator_name_;
    vector<Operator<double>> operators_;

    vector<double> distances_;
    double dead_zone_;

    KinematicTree* tree_{nullptr};
    map<string, std::function<double(const KDL::Frame&)>> map_coordinate_;
};
