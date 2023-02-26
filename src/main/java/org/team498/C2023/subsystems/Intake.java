package org.team498.C2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.team498.C2023.Constants.IntakeConstants.*;
import static org.team498.C2023.Ports.Intake.*;

//TODO Update to add tricks
public class Intake extends SubsystemBase {
    private final CANSparkMax bottomRoller;
    private final CANSparkMax topRoller;
    private final CANSparkMax leftWrist;
    private final CANSparkMax rightWrist;

    private final ProfiledPIDController PID;

    private final DutyCycle encoder;

    public enum State {
        INTAKE(0, 1, 1),
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
        bottomRoller = new CANSparkMax(BOTTOM_ROLLER, MotorType.kBrushless);
        topRoller = new CANSparkMax(TOP_ROLLER, MotorType.kBrushless);
        leftWrist = new CANSparkMax(L_WRIST, MotorType.kBrushless);
        rightWrist = new CANSparkMax(R_WRIST, MotorType.kBrushless);
        configRollerMotor(bottomRoller);
        configRollerMotor(topRoller);
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
    private void configRollerMotor(CANSparkMax motor) {
        motor.restoreFactoryDefaults();
        motor.setIdleMode(IdleMode.kCoast);
        motor.setSmartCurrentLimit(35);
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
        return encoder.getOutput() - 0.210628;
    }

    public void setRollers(State state) {
        bottomRoller.set(state.bottomRollerSpeed);
        topRoller.set(state.topRollerSpeed);
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
