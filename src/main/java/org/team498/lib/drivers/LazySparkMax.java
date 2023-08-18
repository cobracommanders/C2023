package org.team498.lib.drivers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;

public class LazySparkMax extends CANSparkMax {
    private double currentSetpoint = Double.NaN;
    private IdleMode currentIdleMode = null;
    private final double epsilon = 0.01; // Minimum allowable change in speed per loop

    public LazySparkMax(int deviceNumber, MotorType motorType) {
        super(deviceNumber, motorType);
    }

    @Override
    public void set(double setpoint) {
        if (setpoint != currentSetpoint) {
            this.currentSetpoint = setpoint;
            super.set(setpoint);
        }
        //super.set(setpoint);
    }

    @Override
    public REVLibError setIdleMode(IdleMode idleMode) {
        if (currentIdleMode != idleMode) {
            currentIdleMode = idleMode;
            return super.setIdleMode(idleMode);
        }
        return REVLibError.kOk;
    }
}
