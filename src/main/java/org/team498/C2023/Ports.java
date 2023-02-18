package org.team498.C2023;

public final class Ports {

    public static class Drivetrain {
        public static final int FL_DRIVE = 3;
        public static final int FR_DRIVE = 5;
        public static final int BL_DRIVE = 1;
        public static final int BR_DRIVE = 7;

        public static final int FL_STEER = 4;
        public static final int FR_STEER = 6;
        public static final int BL_STEER = 2;
        public static final int BR_STEER = 8;

        public static final int FL_CANCODER = 12;
        public static final int FR_CANCODER = 13;
        public static final int BL_CANCODER = 11;
        public static final int BR_CANCODER = 14;

        public static final int GYRO = 15;
        //TODO Fix these ids AND the gyro
    }

    public static class Intake {
        public static final int ROLLERS = 20;
        public static final int L_WRIST = 21;
        public static final int R_WRIST = 22;
        public static final int ENCODER_PORT = 2; // DIO
    }

    public static class Elevator {
        public static final int L_ELEVATOR_ID = 30;
        public static final int R_ELEVATOR_ID = 31;
    }

    public static class Manipulator {
        public static final int ROLLERS = 40;
    }

    public static class Wrist {
        public static final int WRIST = 50;
        public static final int ENCODER_PORT = 5; // DIO
    }

    public static class ConeARiser {
        public static final int FRONTBACK = 60;
        public static final int LEFTRIGHT = 61;
    }
}
