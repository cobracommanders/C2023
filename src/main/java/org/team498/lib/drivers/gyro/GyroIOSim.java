package org.team498.lib.drivers.gyro;

public class GyroIOSim implements GyroIO {
    private double yaw = 0.0;

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.yaw = yaw;
    }

    @Override
    public void setYaw(double yaw) {
        this.yaw = yaw;
    }
}
