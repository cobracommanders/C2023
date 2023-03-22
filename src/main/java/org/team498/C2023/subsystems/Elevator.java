package org.team498.C2023.subsystems;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team498.C2023.ShootTables;
import org.team498.C2023.State;

import static org.team498.C2023.Constants.ElevatorConstants.*;
import static org.team498.C2023.Ports.Elevator.*;

import org.team498.C2023.Robot;
import org.team498.C2023.RobotPosition;

public class Elevator extends SubsystemBase {
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    private final DutyCycle encoder;

    private final ProfiledPIDController PID;

    private final ElevatorFeedforward feedforward = new ElevatorFeedforward(S, G, V);

    private ControlMode controlMode = ControlMode.PID;
    private State.Elevator currentState = State.Elevator.IDLE;

    private double speed = 0;
    private double simPower = 0;

    public enum ControlMode {
        PID,
        MANUAL
    }

    private Elevator() {
        leftMotor = new TalonFX(F_ELEVATOR_ID);
        rightMotor = new TalonFX(B_ELEVATOR_ID);

        leftMotor.setNeutralMode(NeutralMode.Brake);
        rightMotor.setNeutralMode(NeutralMode.Brake);

        leftMotor.setInverted(true);
        rightMotor.setInverted(true);

        rightMotor.follow(leftMotor, FollowerType.PercentOutput);

        encoder = new DutyCycle(new DigitalInput(ENCODER_PORT));

        PID = new ProfiledPIDController(P, I, D, new TrapezoidProfile.Constraints(3, 5));
        PID.reset(0);
        PID.setTolerance(0);

        SmartDashboard.putNumber("Elevator PID", 0);
    }

    @Override
    public void periodic() {
        PID.setGoal(getSetpoint(currentState, RobotPosition.getFutureScoringNodeDistance()));

        double speed;
        if (controlMode == ControlMode.PID) {
            speed = PID.calculate(getPosition());
        } else {
            speed = this.speed;
        }
    
        leftMotor.set(TalonFXControlMode.PercentOutput, speed + feedforward.calculate(speed));

        SmartDashboard.putData(this);
        SmartDashboard.putBoolean("Elevator at Setpoint", atSetpoint());
        SmartDashboard.putNumber("Elevator Error", PID.getGoal().position - getPosition());

        if (Robot.isSimulation()) simPower = speed + feedforward.calculate(speed);

        SmartDashboard.putNumber("Elevator Absolute Encoder", getAbsoluteEncoderPosition());
        SmartDashboard.putNumber("Elevator Position", getPosition());
    }

    public boolean aboveIntakeHeight() {
        return getPosition() > 0.4; //TODO find out the real number here
    }

    private double getSetpoint(State.Elevator state, double interpolatedValue) {
        return switch (state) {
            case SHOOT_DRIVE_CUBE_MID -> ShootTables.midCube.elevatorHeight.getInterpolatedValue(interpolatedValue);
            case SHOOT_DRIVE_CUBE_TOP -> ShootTables.topCube.elevatorHeight.getInterpolatedValue(interpolatedValue);
            case SHOOT_DRIVE_CONE_MID -> ShootTables.midCone.elevatorHeight.getInterpolatedValue(interpolatedValue);
            default -> state.setpoint;
        };
    }

    public void setState(State.Elevator state) {
        this.controlMode = ControlMode.PID;
        this.currentState = state;
        PID.setGoal(state.setpoint);
    }

    public void setSpeed(double speed) {
        this.controlMode = ControlMode.MANUAL;
        this.speed = speed;
    }

    public double getAbsoluteEncoderPosition() {
        double angle = encoder.getOutput() + 0.75;
            if (angle < 1)
                angle += 1;
            
        return (angle - 0.744160 - 0.75);
    }

    public void updateInitialPosition(boolean inAutoPose) {
        double angle;
        if (inAutoPose) {
            angle = encoder.getOutput() + 0.5;
            if (angle < 1)
                angle += 1;
            
            setEncoderPosition(angle - 0.0);
        } else {
            angle = encoder.getOutput() + 0.75;
            if (angle < 1)
                angle += 1;
            setEncoderPosition((angle - 0.744160 - 0.75) * -5);
        }
    }

    public double getPosition() {
        return (leftMotor.getSelectedSensorPosition() / 2048) / MOTOR_ROTATION_TO_METERS;
    }

    public boolean atSetpoint() {
        return Math.abs(PID.getGoal().position - getPosition()) < 0.05;
    }

    public double getPower() {
        if (Robot.isReal()) {
            return leftMotor.getMotorOutputPercent();
        }
        return simPower;
    }

    public void setEncoderPosition(double position) {
        leftMotor.setSelectedSensorPosition(position * 2048);
    }

    private static Elevator instance;

    public static Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
        }

        return instance;
    }

}
