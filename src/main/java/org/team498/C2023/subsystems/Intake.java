package org.team498.C2023.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenixpro.controls.NeutralOut;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.team498.C2023.Constants.IntakeConstants.*;
import static org.team498.C2023.Ports.Intake.*;

//TODO Update to add tricks
public class Intake extends SubsystemBase {
    private final TalonFX bottomRoller;
    private final TalonFX topRoller;
    private final CANSparkMax leftWrist;
    private final CANSparkMax rightWrist;

    private final ProfiledPIDController PID;

    private final DutyCycle encoder;

    public enum State {
        INTAKE(0, 0.7, 0.7),
        SPIT(0, 1, -1),
        IDLE(0.353, 0, 0);

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

        PID = new ProfiledPIDController(P, I, D, new TrapezoidProfile.Constraints(2, 1));
        PID.reset(0);

        encoder = new DutyCycle(new DigitalInput(ENCODER_PORT));

        setState(State.IDLE);
    }

    @Override
    public void periodic() {
        // leftWrist.set(PID.calculate(getAngle()) + (Math.cos(Math.toRadians(getAngle())) * F));
        leftWrist.set(-PID.calculate(getAngle()));
        SmartDashboard.putNumber("Intake Encoder", getAngle());
        SmartDashboard.putNumber("Intake Output", PID.calculate(getAngle()));
        SmartDashboard.putNumber("Intake Error", PID.getPositionError());
        SmartDashboard.putNumber("Intake Goal", PID.getGoal().position);
    }

    public double getAngle() {
        double angle = encoder.getOutput() + 0.5;
        if (angle > 1) angle -= 1;
        return angle - 0.438619;
    }

    public void setRollers(State state) {
        bottomRoller.set(ControlMode.PercentOutput, state.bottomRollerSpeed);
        topRoller.set(ControlMode.PercentOutput, state.topRollerSpeed);
    }

    public void setState(State state) {
        PID.setGoal(state.position);
        setRollers(state);
    }


    private static Intake instance;

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }

        return instance;
    }
}
