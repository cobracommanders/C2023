package org.team498.C2023.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team498.C2023.RobotState;

import static org.team498.C2023.Constants.IntakeConstants.*;
import static org.team498.C2023.Ports.Intake.*;

//TODO Update to add tricks
public class Intake extends SubsystemBase {
    private final TalonFX bottomRoller;
    private final TalonFX topRoller;
    private final CANSparkMax leftWrist;
    private final CANSparkMax rightWrist;

    private final ProfiledPIDController wristPID;
    private final PIDController topRollerPID;
    private final PIDController bottomRollerPID;

    private final DutyCycle encoder;

    public enum State {
        INTAKE(0.1, 0.5, 0.5),
        SPIT(0.35, 0.45, -0.45),
        IDLE_OUT(0, 0, 0),
        TRAVEL(0.2, 0, 0),
        IDLE_IN(0.4, 0, 0),
        OUTTAKE(0.1, -0.5, -0.5);

        private final double position;
        private final double bottomRollerSpeed;
        private final double topRollerSpeed;

        State(double position, double bottomRollerSpeed, double topRollerSpeed) {
            this.position = position;
            this.bottomRollerSpeed = bottomRollerSpeed;
            this.topRollerSpeed = topRollerSpeed;
        }
    }

    private Intake() {
        bottomRoller = new TalonFX(BOTTOM_ROLLER);
        topRoller = new TalonFX(TOP_ROLLER);
        leftWrist = new CANSparkMax(L_WRIST, MotorType.kBrushless);
        rightWrist = new CANSparkMax(R_WRIST, MotorType.kBrushless);

        bottomRoller.setNeutralMode(NeutralMode.Coast);
        topRoller.setNeutralMode(NeutralMode.Coast);

        bottomRoller.setInverted(true);
        topRoller.setInverted(false);

        leftWrist.restoreFactoryDefaults();
        rightWrist.restoreFactoryDefaults();

        leftWrist.setIdleMode(IdleMode.kBrake);
        rightWrist.setIdleMode(IdleMode.kBrake);

        rightWrist.follow(leftWrist, true);

        wristPID = new ProfiledPIDController(P, I, D, new TrapezoidProfile.Constraints(2, 1));
        wristPID.reset(State.IDLE_IN.position);

        encoder = new DutyCycle(new DigitalInput(ENCODER_PORT));

        setState(State.IDLE_IN);

        bottomRollerPID = new PIDController(0, 0, 0);
        topRollerPID = new PIDController(0, 0, 0);
    }

    @Override
    public void periodic() {
        // leftWrist.set(PID.calculate(getAngle()) + (Math.cos(Math.toRadians(getAngle())) * F));
        leftWrist.set(-wristPID.calculate(getAngle()));
        // bottomRoller.set(ControlMode.PercentOutput, bottomRollerPID.calculate(bottomRoller.getSelectedSensorVelocity()));
        // topRoller.set(ControlMode.PercentOutput, topRollerPID.calculate(topRoller.getSelectedSensorVelocity()));
        SmartDashboard.putNumber("Intake Encoder", getAngle());
        SmartDashboard.putNumber("Intake Output", wristPID.calculate(getAngle()));
        SmartDashboard.putNumber("Intake Error", wristPID.getPositionError());
        SmartDashboard.putNumber("Intake Goal", wristPID.getGoal().position);
    }

    public double getAngle() {
        double angle = encoder.getOutput() + 0.5;
        if (angle > 1) angle -= 1;
        return angle - 0.5;
    }

    public void setRollers(State state) {
        bottomRoller.set(ControlMode.PercentOutput, state.bottomRollerSpeed);
        topRoller.set(ControlMode.PercentOutput, state.topRollerSpeed);

        // bottomRollerPID.setSetpoint(state.bottomRollerSpeed);
        // topRollerPID.setSetpoint(state.topRollerSpeed);
    }

    public void setState(State state) {
        wristPID.setGoal(state.position);
        setRollers(state);
    }

    public State getNextState() {
        return switch (RobotState.getInstance().getNextHeight()) {
            case MID, TOP, INTERPOLATE, DOUBLE_SS -> State.IDLE_IN;
            case SINGLE_SS -> State.TRAVEL;
            case INTAKE -> State.INTAKE;
            case LOW -> State.SPIT;
        };
    }

    public void setToNextState() {
        setState(getNextState());
    }


    private static Intake instance;

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }

        return instance;
    }
}
