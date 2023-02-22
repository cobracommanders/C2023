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

import static org.team498.C2023.Constants.ElevatorConstants.*;
import static org.team498.C2023.Ports.Elevator.L_ELEVATOR_ID;
import static org.team498.C2023.Ports.Elevator.R_ELEVATOR_ID;

public class Elevator extends SubsystemBase {
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;
    private final RelativeEncoder encoder;

    private final ProfiledPIDController PID;

    private final ElevatorFeedforward feedforward = new ElevatorFeedforward(S, G, V);

    private ControlMode controlMode;
    private double speed = 0;

    public enum State {
        CONEARISER(0, 0),
        SINGLE_SS(0, 0),
        DOUBLE_SS(0, 0),

        LOW(0, 0),
        MID(0.5, 0.3),
        TOP(0.6, 0.5),

        AUTO_SHOT(0, 0),
        IDLE(0, 0);

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
        leftMotor = new CANSparkMax(L_ELEVATOR_ID, MotorType.kBrushless);
        rightMotor = new CANSparkMax(R_ELEVATOR_ID, MotorType.kBrushless);

        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();

        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);

        rightMotor.follow(leftMotor, true);

        encoder = leftMotor.getEncoder();

        PID = new ProfiledPIDController(P, I, D, new TrapezoidProfile.Constraints(.5, 1));
        PID.reset(0);
    }

    @Override
    public void periodic() {
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
        SmartDashboard.putNumber("Elevator Error", PID.getPositionError());
    }

    public State getNextState() {
        return switch (RobotState.getInstance().getNextHeight()) {
            case LOW -> State.LOW;
            case MID -> State.MID;
            case TOP -> State.TOP;
            case DOUBLE_SS -> State.DOUBLE_SS;
            case SINGLE_SS -> State.SINGLE_SS;
        };
    }

    public void setState(State state) {
        this.controlMode = ControlMode.PID;
        PID.setGoal(RobotState.getInstance().inConeMode()
                    ? state.setpointCone
                    : state.setpointCube);
    }

    public void setSpeed(double speed) {
        this.controlMode = ControlMode.MANUAL;
        this.speed = speed;
    }

    public double getPosition() {
        return encoder.getPosition() / MOTOR_ROTATION_TO_METERS;
    }

    public boolean atSetpoint() {
        return PID.getPositionError() < 0.05;
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
