package org.team498.lib.drivers;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.wpilibj.RobotBase;
import org.team498.lib.util.RotationUtil;

import static org.team498.C2023.Ports.Drivetrain.GYRO;

public class Gyro extends Pigeon2 {
    private double angleOffset = 0;
    private double simAngle = 0;

    /** @return yaw angle in degrees (CCW positive), ranging from -180 to 180 degrees */
    @Override
    public synchronized double getYaw() {
        return RobotBase.isReal()
               ? RotationUtil.toSignedDegrees(super.getYaw() + angleOffset )
               : -RotationUtil.toSignedDegrees(simAngle + angleOffset );
    }


    /** @return the rotation offset of the gyro */
    public double getAngleOffset() {
        return angleOffset;
    }

    /**
     * Set an offset to adjust the output of the gyro by.
     *
     * @param angleOffset the offset in degrees
     */
    public void setAngleOffset(double angleOffset) {
        this.angleOffset = angleOffset;
    }

    public void setSimAngle(double angle) {
        simAngle = angle - angleOffset;
    }

    private Gyro(int CANId) {
        super(CANId);
    }

    private static Gyro instance;

    public static Gyro getInstance() {
        if (instance == null) {
            instance = new Gyro(GYRO);
        }
        return instance;
    }

}
