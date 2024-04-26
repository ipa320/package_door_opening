#include "pitasc_pivot_taubin/monitors/monitor_relative_angular_distance.hpp"

MonitorRelativeAngularDistance::MonitorRelativeAngularDistance(const string& name)
    : Monitor(name)
{
}

bool MonitorRelativeAngularDistance::init(Dict& params)
{
    pi_debug("Init component '{}'", getName());

    // Parameters
    extract(params["reference_frame"], ref_frame_);
    extract(params["frame"], observed_frame_);
    extract(params["dead_zone"], dead_zone_);

    try {
        extract(params["distances"], distances_);
    } catch (std::invalid_argument&) {
        pi_warn("Parameter type of {} is not {}_csv. Automatic conversion from {}_parameter", "distances",
                "float", "float");
        // coordinates is not a string_csv, but maybe a double_parameter, so lets try that instead
        auto distance = extract_type(params["distances"], double);
        distances_ = vector<double>{distance};  // only single distance in this case
    }

    try {
        extract(params["coordinates"], coordinates_);
    } catch (std::invalid_argument&) {
        pi_warn("Parameter type of {} is not {}_csv. Automatic conversion from {}_parameter", "coordinates",
                "string", "string");
        // coordinates is not a string_csv, but maybe a string_parameter, so lets try that instead
        auto coordinate = extract_type(params["coordinates"], string);
        coordinates_ = vector<string>{coordinate};  // only single coordinate in this case
    }

    if (distances_.size() == 1 && coordinates_.size() > 1) {
        distances_ = vector<double>(coordinates_.size(), distances_[0]);
        pi_info("Monitor '{}': Only one distance given. Using '{}' for all coordinates.", getName(),
                std::to_string(distances_[0]));
    }

    setDependencies({});

    auto peer = extract_type(params["kinematic_graph"], shared_ptr<RuntimeObject>);
    tree_ = dynamic_cast<KinematicTree*>(peer.get());

    extract(params["operator"], operator_name_);

    map_coordinate_ = map<string, std::function<double(const KDL::Frame& frame)>>{
        {"a",
         [this](const KDL::Frame& frame) {
            double angle = std::atan2(frame.p[2], frame.p[1]);
            return angle;
         }},
        {"b",
         [this](const KDL::Frame& frame) {
            double angle = std::atan2(frame.p[0], frame.p[2]);
            return angle;
         }},
        {"c", [this](const KDL::Frame& frame) {
            double angle = std::atan2(frame.p[1], frame.p[0]);
            return angle;
         }}};

    // create operators handling distances comparison function
    for (auto& dist : distances_) {
        operators_.push_back(Operator<double>(dist, operator_name_));
    }
    return Monitor::init(params);
}


bool MonitorRelativeAngularDistance::onStart()
{
    // Update pose
    if (!tree_->lookup(ref_frame_, observed_frame_, initial_frame_)) {
        throw std::runtime_error("MonitorRelativeAngularDistance: Lookup from '" + ref_frame_ + "' to '"
                                 + observed_frame_ + "' not possible");
    }
    for (auto& op : operators_) {
        op.reset();
    }
    return Monitor::onStart();
}
    
void MonitorRelativeAngularDistance::onUpdate(const Tick& /*unused*/)
{
    KDL::Frame frame;
    if (!tree_->lookup(ref_frame_, observed_frame_, frame)) {
        pi_error("Tree lookup error: {} -> {}", ref_frame_, observed_frame_);
        return;
    } else {
        for (unsigned int i = 0; i < operators_.size(); i++) {
            double distance;
            double initAngle = map_coordinate_[coordinates_[i]](initial_frame_);
            double currentAngle = map_coordinate_[coordinates_[i]](frame);
            if (initAngle > currentAngle) {
               distance = 2 * M_PI - (initAngle - currentAngle);
            } else {
               distance = currentAngle - initAngle;
            }

            //prevents jump between min/max value caused by inaccuracies near 0°/360°
            if (distance < dead_zone_ || distance > (2 * M_PI - dead_zone_) ) {return;}

            auto result = operators_[i].check(distance);
            pi_spam("Coordinate: {}, Operator: {}({}, {}), Result: {}\n", coordinates_[i], operator_name_,
                    distance, distances_[i], result);

            // Cancel loop and do not fire if first distance does not check out
            if (!result) {
                return;
            }
        }
        fire();
    }
}

RUNTIME_COMPONENT(MonitorRelativeAngularDistance)