package org.team498.lib.drivers.gyro;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.Pigeon2;

public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 pigeon;
    public GyroIOPigeon2(int CANId) {
        pigeon = new Pigeon2(CANId);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = pigeon.getLastError().equals(ErrorCode.OK);
        inputs.pitch = pigeon.getPitch();
        inputs.roll = pigeon.getRoll();
        inputs.yaw = pigeon.getYaw();
    }

    @Override
    public void setYaw(double yaw) {
        pigeon.setYaw(yaw);
    }
}
