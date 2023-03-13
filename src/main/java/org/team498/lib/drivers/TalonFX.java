package org.team498.lib.drivers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

// Credit to team 254
public class TalonFX extends com.ctre.phoenix.motorcontrol.can.WPI_TalonFX {
    private ControlMode currentControlMode = null;
    private double currentSetpoint = Double.NaN;
    private DemandType currentDemandType = null;
    private double currentDemand1 = Double.NaN;

    public TalonFX(int deviceNumber) {
        super(deviceNumber);
    }
/*

    @Override
    public void set(ControlMode mode, double outputValue) {
        if (mode != currentControlMode || outputValue != currentSetpoint) {
            this.currentControlMode = mode;
            this.currentSetpoint = outputValue;
            super.set(mode, outputValue);
        }
    }

    @Override
    public void set(TalonFXControlMode mode, double value) {
        set(mode.toControlMode(), value);
    }

    @Override
    public void set(ControlMode mode, double demand0, DemandType demand1Type, double demand1) {
        if (mode != currentControlMode || currentSetpoint != demand0 || currentDemandType != demand1Type || currentDemand1 != demand1) {
            this.currentControlMode = mode;
            this.currentSetpoint = demand0;
            this.currentDemandType = demand1Type;
            this.currentDemand1 = demand1;
            super.set(mode, demand0, demand1Type, demand1);
        }
    }

    public void set(TalonFXControlMode mode, double demand0, DemandType demand1Type, double demand1) {
        set(mode.toControlMode(), demand0, demand1Type, demand1);
    }*/
}
