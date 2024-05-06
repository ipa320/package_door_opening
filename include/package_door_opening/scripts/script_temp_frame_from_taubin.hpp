#pragma once

#include "package_door_opening/scripts/script_temp_frame_offset.hpp"

class ScriptTempFrameFromTaubin : public ScriptTempFrameOffset {
public:
    explicit ScriptTempFrameFromTaubin (const string& name);
    bool init(Dict& params) override;
    bool onStart() override;
    void onUpdate(const Tick& tick) override;

private:
    int n = 0;
    vector<double> toolX;
    vector<double> toolY;
    double sumX = 0;
    double sumY = 0;
    string tool_;
};