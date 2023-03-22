package org.team498.C2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team498.C2023.State;

import static org.team498.C2023.Constants.IntakeWristConstants.*;
import static org.team498.C2023.Ports.IntakeWrist.*;

import org.team498.C2023.Robot;

public class IntakeWrist extends SubsystemBase {
    private final CANSparkMax leftWrist;
    private final CANSparkMax rightWrist;

    private final ProfiledPIDController PID;

    private double simAngle = 0;

    private final DutyCycle encoder;

    private IntakeWrist() {
        leftWrist = new CANSparkMax(L_WRIST, MotorType.kBrushless);
        rightWrist = new CANSparkMax(R_WRIST, MotorType.kBrushless);

        leftWrist.restoreFactoryDefaults();
        rightWrist.restoreFactoryDefaults();

        leftWrist.setIdleMode(IdleMode.kBrake);
        rightWrist.setIdleMode(IdleMode.kBrake);

        rightWrist.follow(leftWrist, true);

        PID = new ProfiledPIDController(P, I, D, new TrapezoidProfile.Constraints(2, 1));
        PID.reset(State.IntakeWrist.IDLE_IN.position);

        encoder = new DutyCycle(new DigitalInput(ENCODER_PORT));

        setState(State.IntakeWrist.IDLE_IN);
    }

    @Override
    public void periodic() {
        leftWrist.set(-PID.calculate(getAngle()));
        SmartDashboard.putNumber("Intake Encoder", getAngle());
        SmartDashboard.putNumber("Intake Output", PID.calculate(getAngle()));
        SmartDashboard.putNumber("Intake Error", PID.getPositionError());
        SmartDashboard.putNumber("Intake Goal", PID.getGoal().position);
    }

    public double getAngle() {
        double angle = encoder.getOutput() + 0.75;
        if (angle < 1)
            angle += 1;

        return Robot.isReal() ? angle - 0.75 - 0.685961 : simAngle;
    }

    public void setState(State.IntakeWrist state) {
        PID.setGoal(state.position);
    }

    public boolean atSetpoint() {
        return PID.atSetpoint();
    }

    public void setSimAngle(double position) {
        simAngle = (position / 360);
    }

    public double getPower() {
        return leftWrist.get();
    }

    private static IntakeWrist instance;

    public static IntakeWrist getInstance() {
        if (instance == null) {
            instance = new IntakeWrist();
        }

        return instance;
    }
}
