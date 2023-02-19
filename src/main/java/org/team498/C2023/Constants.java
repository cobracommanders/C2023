package org.team498.C2023;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class Constants {
    public static final class OIConstants {
        public static final int DRIVER_CONTROLLER_ID = 0;
        public static final int OPERATOR_CONTROLLER_ID = 1;
    }

    public static final class DrivetrainConstants {
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 5;

        public static final double SWERVE_MODULE_DISTANCE_FROM_CENTER = 10.75;

        public static final double MK4I_DRIVE_REDUCTION_L2 = 6.75;
        public static final double MK4I_STEER_REDUCTION_L2 = 21.428571428571428571428571428571; // 150 / 7

        public static final double DRIVE_WHEEL_DIAMETER = 4;
        public static final double DRIVE_WHEEL_CIRCUMFERENCE = DRIVE_WHEEL_DIAMETER * Math.PI;

        public static final double FL_MODULE_OFFSET = 28 - 45;//360 - 312.4 - 45; //28.0 137.1
        public static final double FR_MODULE_OFFSET = 28 - 75;//270 - 137.1; //137.1 312.4
        public static final double BL_MODULE_OFFSET = 28-45;//180 + 28.0 - 45; //312.4 74.5
        public static final double BR_MODULE_OFFSET = 28-45 + 90 - 45;//74.5 - 45 - 180; //74.5 28.0

        public static final double ROBOT_WIDTH = 26.5 + 6; // Robot width with bumpers, in inches

        public static final class AngleConstants {
            public static final double P = 5;
            public static final double I = 0;
            public static final double D = 0;
            public static final double EPSILON = 1.0;


            // Constraints for the profiled angle controller
            public static final double MAX_ANGULAR_SPEED_DEGREES_PER_SECOND = 720;
            public static final double MAX_ANGULAR_SPEED_DEGREES_PER_SECOND_SQUARED = Math.pow(MAX_ANGULAR_SPEED_DEGREES_PER_SECOND, 2);

            public static final TrapezoidProfile.Constraints CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(
                    MAX_ANGULAR_SPEED_DEGREES_PER_SECOND,
                    MAX_ANGULAR_SPEED_DEGREES_PER_SECOND_SQUARED
            );

        }

        public static final class PoseConstants {
            public static final double P = 2.5;
            public static final double I = 0;
            public static final double D = 0;

            public static final double EPSILON = 0.1;
        }
    }

    public static final class ElevatorConstants {
        public static final double P = 0.4;
        public static final double I = 0;
        public static final double D = 0.02;
    }

    public static final class IntakeConstants {
        public static final double P = 0;
        public static final double I = 0;
        public static final double D = 0;

        public static final double F = 0;
    }

    public static final class ManipulatorConstants {
        public static final double P = 0.1;
        public static final double I = 0;
        public static final double D = 0;
    }

    public static final class WristConstants {
        public static final double P = 3;
        public static final double I = 0;
        public static final double D = 0;

        public static final double F = 0;

        public static final double WRIST_RATIO = 5 * 4 * 3 * (80.0 / 36.0);
    }


}
