package org.team498.C2023.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    class ElevatorIOInputs {
        public double positionMeters = 0.0;
        public double velocityMetersPerSecond = 0.0;

        public double[] temperatureCelsius = new double[] {};
    }

    default void updateInputs(ElevatorIOInputs inputs) {}

    default void setSpeed(double speed) {}

    default void setInitialPosition(boolean inAutoPose) {}

    default void setPosition(double position, double feedforward) {}

    default void configPID(double kP, double kI, double kD) {}




}
