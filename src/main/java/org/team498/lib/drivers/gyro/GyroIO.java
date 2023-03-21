package org.team498.lib.drivers.gyro;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    class GyroIOInputs {
        public boolean connected = false;
        public double yaw = 0.0;
        public double pitch = 0.0;
        public double roll = 0.0;
    }

    default void updateInputs(GyroIOInputs inputs) {}
    default void setYaw(double yaw) {}
}