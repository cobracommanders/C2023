package org.team498.C2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team498.C2023.RobotState;
import org.team498.C2023.StateTables;
import org.team498.lib.util.LinearInterpolator;

import static org.team498.C2023.Constants.ElevatorConstants.*;
import static org.team498.C2023.Ports.Elevator.F_ELEVATOR_ID;
import static org.team498.C2023.Ports.Elevator.B_ELEVATOR_ID;

public class Elevator extends SubsystemBase {
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;
    private final RelativeEncoder encoder;

    private final ProfiledPIDController PID;

    private final ElevatorFeedforward feedforward = new ElevatorFeedforward(S, G, V);

    private final LinearInterpolator interpolator;

    private ControlMode controlMode;
    private double speed = 0;

    public enum State {
        CONEARISER(0, 0),
        SINGLE_SS(0.506641, 0.1),
        DOUBLE_SS(0.66, 0),

        LOW(0, 0),
        MID(0.95, 0.6),
        TOP(0.95, 0.95),

        AUTO_SHOT(0, 0),
        IDLE(0, 0),
        
        INTERPOLATE(0, 0);

        private final double setpointCone;
        private final double setpointCube;

        State(double setpointCone, double setpointCube) {
            this.setpointCone = setpointCone;
            this.setpointCube = setpointCube;
        }
    }

    public enum ControlMode {
        PID,
        MANUAL
    }

    private Elevator() {
        leftMotor = new CANSparkMax(F_ELEVATOR_ID, MotorType.kBrushless);
        rightMotor = new CANSparkMax(B_ELEVATOR_ID, MotorType.kBrushless);

        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();

        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);

        leftMotor.setInverted(true);
        rightMotor.setInverted(true);

        rightMotor.follow(leftMotor, false);

        encoder = leftMotor.getEncoder();


        PID = new ProfiledPIDController(P, I, D, new TrapezoidProfile.Constraints(5, 4));
        PID.reset(0);
        PID.setTolerance(0);

        interpolator = new LinearInterpolator(StateTables.elevatorHeightTable);

        SmartDashboard.putNumber("Elevator PID", 0);
    }

    @Override
    public void periodic() {
        this.controlMode= ControlMode.PID;

        // PID.setGoal(SmartDashboard.getNumber("Elevator PID", 0));
        double speed;
        if (controlMode == ControlMode.PID) {
            speed = PID.calculate(getPosition());
        } else {
            speed = this.speed;
        }
        leftMotor.set(speed + feedforward.calculate(speed));

        SmartDashboard.putData(this);
        SmartDashboard.putNumber("Elevator Position", getPosition());
        SmartDashboard.putBoolean("Elevator at Setpoint", atSetpoint());
        SmartDashboard.putNumber("Elevator Error", PID.getGoal().position - getPosition());
    }

    public State getNextState() {
        return switch (RobotState.getInstance().getNextHeight()) {
            case LOW, INTAKE -> State.LOW;
            case MID -> State.MID;
            case TOP -> State.TOP;
            case DOUBLE_SS -> State.DOUBLE_SS;
            case SINGLE_SS -> State.SINGLE_SS;
            case INTERPOLATE -> State.INTERPOLATE;
        };
    }

    public boolean aboveIntakeHeight() {
        return getPosition() > 0.4;
    }

    public void setState(State state) {
        this.controlMode = ControlMode.PID;
        if (state == State.INTERPOLATE) {
            PID.setGoal(interpolator.getInterpolatedValue(Drivetrain.getInstance().getNextDistanceToTag()));
        } else {
            PID.setGoal(RobotState.getInstance().inConeMode()
            ? state.setpointCone
            : state.setpointCube);
        }
    }

    public void setSpeed(double speed) {
        this.controlMode = ControlMode.MANUAL;
        this.speed = speed;
    }

    public double getPosition() {
        return encoder.getPosition() / MOTOR_ROTATION_TO_METERS;
    }

    public boolean atSetpoint() {
        return Math.abs(PID.getGoal().position -getPosition()) < 0.025;
    }

    public double getPower() {
        return leftMotor.get();
    }

    public void setEncoderPosition(double position) {
        encoder.setPosition(position);
    }

    public void setToNextState() {
        setState(getNextState());
    }


    private static Elevator instance;

    public static Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
        }

        return instance;
    }

}
