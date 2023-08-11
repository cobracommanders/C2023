package org.team498.C2023;

public final class Ports {

    public static class Drivetrain {
        public static final int FL_DRIVE = 1;
        public static final int FR_DRIVE = 5;
        public static final int BL_DRIVE = 3;
        public static final int BR_DRIVE = 7;

        public static final int FL_STEER = 2;
        public static final int FR_STEER = 6;
        public static final int BL_STEER = 4;
        public static final int BR_STEER = 8;

        public static final int FL_CANCODER = 11;
        public static final int FR_CANCODER = 13;
        public static final int BL_CANCODER = 12;
        public static final int BR_CANCODER = 14;

        public static final int GYRO = 20;
    }

    public static class Elevator {
        public static final int F_MOTOR = 30;
        public static final int B_MOTOR = 31;
        public static final int ENCODER = 4; // DIO
    }

    public static class Flywheel {
        public static final int L_MOTOR = 60;
        public static final int R_MOTOR = 61;
    }
}
