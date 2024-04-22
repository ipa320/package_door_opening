#include "pitasc_pivot_taubin/scripts/script_temp_frame_from_taubin.hpp"

ScriptTempFrameFromTaubin::ScriptTempFrameFromTaubin (const string& name)
    : ScriptTempFrameOffset(name)
{
}

bool ScriptTempFrameFromTaubin::init(Dict& params)
{
    tool_ = extract_type(params["tool_frame"], string);
    setDependencies({});
    return ScriptTempFrameOffset::init(params);
}

bool ScriptTempFrameFromTaubin::onStart()
{
    return ScriptTempFrameOffset::onStart();
}

void ScriptTempFrameFromTaubin::onUpdate(const Tick& tick)
{
    KDL::Frame frame_parent_to_tool;

    //lookup tool koordinates
    if (!tree_->lookup(parent_, tool_, frame_parent_to_tool)) {
        throw std::runtime_error("TempFrameFromTaubin: Lookup from '" + parent_ + "' to '" + tool_
                                 + "' not possible");
    }

    //add koordinates to list
    toolX.push_back(frame_parent_to_tool.p.x());
    toolY.push_back(frame_parent_to_tool.p.y());
    sumX += toolX.back();
    sumY += toolY.back();
    n++;

    if (n < 3) {return ScriptTempFrameOffset::onUpdate(tick);} 

    int i, iter, IterMAX = 99;
    double Xi, Yi, Zi;
    double Mz, Mxy, Mxx, Myy, Mxz, Myz, Mzz, Cov_xy, Var_z;
    double A0, A1, A2, A22, A3, A33;
    double Dy, xnew, x, ynew, y;
    double DET, Xcenter, Ycenter;
        
    double meanX = sumX / n;
    double meanY = sumY / n;

    for (i = 0; i < n; i++)
    {
        Xi = toolX[i] - meanX;   //  centered x-coordinates
        Yi = toolY[i] - meanY;   //  centered y-coordinates
        Zi = Xi * Xi + Yi * Yi;
        
        Mxy += Xi * Yi;
        Mxx += Xi * Xi;
        Myy += Yi * Yi;
        Mxz += Xi * Zi;
        Myz += Yi * Zi;
        Mzz += Zi * Zi;
    }

    Mxx = Mxx / n;
    Myy = Myy / n;
    Mxy = Mxy / n;
    Mxz = Mxz / n;
    Myz = Myz / n;
    Mzz = Mzz / n;

    //      computing coefficients of the characteristic polynomial
    Mz = Mxx + Myy;
    Cov_xy = Mxx * Myy - Mxy * Mxy;
    Var_z = Mzz - Mz * Mz;
    A3 = 4.0 * Mz;
    A2 = -3.0 * Mz * Mz - Mzz;
    A1 = Var_z * Mz + 4 * Cov_xy * Mz - Mxz * Mxz - Myz * Myz;
    A0 = Mxz * (Mxz * Myy - Myz * Mxy) + Myz * (Myz * Mxx - Mxz * Mxy) - Var_z * Cov_xy;
    A22 = A2 + A2;
    A33 = A3 + A3 + A3;

    //    finding the root of the characteristic polynomial
    //    using Newton's method starting at x = 0  
    //     (it is guaranteed to converge to the right root)
        
    for (x = 0., y = A0, iter = 0; iter < IterMAX; iter++)  // usually, 4-6 iterations are enough
    {
        Dy = A1 + x * (A22 + A33 * x);
        xnew = x - y / Dy;
        if ((xnew == x) || (!isfinite(xnew))) break;
        ynew = A0 + xnew * (A1 + xnew * (A2 + xnew * A3));
        if (abs(ynew) >= abs(y) || !isfinite(ynew))  break;
        x = xnew;  y = ynew;
    }
            
    //       computing paramters of the fitting circle
    DET = x * x - x * Mz + Cov_xy;
    Xcenter = (Mxz * (Myy - x) - Myz * Mxy) / DET / 2.0;
    Ycenter = (Myz * (Mxx - x) - Mxz * Mxy) / DET / 2.0;
    
    //       assembling the output
    double tempX = Xcenter + meanX;
    double tempY = Ycenter + meanY;
    
    if(isfinite(tempX) && isfinite(tempY)){
        frame_.p.x(tempX);
        frame_.p.y(tempY);
        tree_->setTransform(joint_, frame_);
    }
    
    ScriptTempFrameOffset::onUpdate(tick);
}

RUNTIME_COMPONENT(ScriptTempFrameFromTaubin)