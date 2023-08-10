package org.team498.lib.wpilib;

public class PController {

    private double kP;
    private double setpoint = 0;
    private double error = 0;

    public PController(double kP) {
        this.kP = kP;
    }

    public void setP(double kP) {
        this.kP = kP;
    }

    public double getError() {
        return error;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public double calculate(double measurement) {
        error = setpoint - measurement;
        return kP * error;
    }
    public double calculate(double measurement, double setpoint) {
        this.setpoint = setpoint;
        error = setpoint - measurement;
        return kP * error;
    }
}
