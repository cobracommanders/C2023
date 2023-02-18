package org.team498.C2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.team498.C2023.Constants.ElevatorConstants.*;
import static org.team498.C2023.Ports.Elevator.L_ELEVATOR_ID;
import static org.team498.C2023.Ports.Elevator.R_ELEVATOR_ID;

public class Elevator extends SubsystemBase {
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;
    private final RelativeEncoder encoder;

    private final PIDController PID;
    private ControlMode controlMode;
    private double speed = 0;
    private State nextScoringHeight = State.HIGH;

    public enum State {
        LOW(3), // Testing
        MID(5), // Testing
        HIGH(10), // Testing
        PASS_CONE(0),
        PASS_CUBE(0),
        DOUBLE_SS_CONE(0),
        DOUBLE_SS_CUBE(0),
        AUTO_SHOT(0),
        IDLE(0);

        public final double setpoint;

        State(double setpoint) {
            this.setpoint = setpoint;
        }
    }

    public enum ControlMode {
        PID,
        MANUAL
    }

    private Elevator() {
        controlMode = ControlMode.MANUAL;

        leftMotor = new CANSparkMax(L_ELEVATOR_ID, MotorType.kBrushless);
        rightMotor = new CANSparkMax(R_ELEVATOR_ID, MotorType.kBrushless);

        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();

        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);

        encoder = leftMotor.getEncoder();
        rightMotor.follow(leftMotor, true);

        PID = new PIDController(P, I, D);
        PID.setTolerance(0.1);
    }

    @Override
    public void periodic() {
        double speed;
        if (controlMode == ControlMode.PID) speed = PID.calculate(encoder.getPosition());
        else {
            speed = this.speed;
        }

        SmartDashboard.putNumber("Elevator position", encoder.getPosition());
        SmartDashboard.putNumber("Elevator speed", speed);

        // if ((Math.signum(speed) == -1) && !limit.get()) {
        // speed = 0;
        // }

        leftMotor.set(speed - Math.pow(((0.171 * encoder.getPosition()) / -4.547614) + 0.04, 2));
    }

    public void setNextScoringHeight(State nextScoringHeight) {
        this.nextScoringHeight = nextScoringHeight;
    }

    public State getNextScoringPosition() {
        return nextScoringHeight;
    }

    public void setState(State state) {
        SmartDashboard.putNumber("Elevator setpoint", state.setpoint);
        setControlMode(ControlMode.PID);
        PID.setSetpoint(state.setpoint);
    }

    public void setControlMode(ControlMode mode) {
        this.controlMode = mode;
    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }

    public boolean atSetpoint() {
        return PID.atSetpoint();
    }

    public void resetEncoder() {
        encoder.setPosition(0);
    }


    private static Elevator instance;

    public static Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
        }

        return instance;
    }

}
