package org.team498.lib.drivers.swervemodule;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    @AutoLog
    class ModuleIOInputs {
        public double drivePositionMeters = 0.0;
        public double driveVelocityRPM = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;
        public double driveTempCelsius = 0.0;

        public double turnAbsolutePositionRad = 0.0;
        public double turnPositionRad = 0.0;
        public double turnVelocityRPM = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnCurrentAmps = 0.0;
        public double turnTempCelsius = 0.0;
    }

    default void updateInputs(ModuleIOInputs inputs) {}

    default void setState(SwerveModuleState state) {}
    default SwerveModuleState getState() {return new SwerveModuleState();}
    default SwerveModulePosition getPosition() {return new SwerveModulePosition();}
    default void updateIntegratedEncoder() {}
    default void setBrakeMode(boolean enable) {}
    default String getName() {return "";}
}
