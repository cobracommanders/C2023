package org.team498.C2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.team498.C2023.Constants.IntakeConstants.*;
import static org.team498.C2023.Ports.Intake.*;

//TODO Update to add tricks
public class Intake extends SubsystemBase {
    private final CANSparkMax rollers;
    private final CANSparkMax leftWrist;
    private final CANSparkMax rightWrist;

    private final PIDController PID;

    private final DutyCycleEncoder encoder;

    public enum State {
        INTAKE(1, 1),
        IDLE(0, 0);

        private final double position;
        private final double rollerSpeed;

        State(double position, double rollerSpeed) {
            this.position = position;
            this.rollerSpeed = rollerSpeed;
        }
    }

    private Intake() {
        rollers = new CANSparkMax(ROLLERS, MotorType.kBrushless);
        leftWrist = new CANSparkMax(L_WRIST, MotorType.kBrushless);
        rightWrist = new CANSparkMax(R_WRIST, MotorType.kBrushless);

        rollers.restoreFactoryDefaults();
        leftWrist.restoreFactoryDefaults();
        rightWrist.restoreFactoryDefaults();

        rollers.setIdleMode(IdleMode.kCoast);
        leftWrist.setIdleMode(IdleMode.kBrake);
        rightWrist.setIdleMode(IdleMode.kBrake);

        rightWrist.follow(leftWrist);

        PID = new PIDController(P, I, D);

        encoder = new DutyCycleEncoder(ENCODER_PORT);
        encoder.setDutyCycleRange(1, 1024);
    }

    @Override
    public void periodic() {
        leftWrist.set(PID.calculate(getAngle()) + (Math.cos(Math.toRadians(getAngle())) * F));
    }

    public double getAngle() {
        return encoder.getAbsolutePosition() * (360.0 / 1024.0);
    }

    public void setRollers(State state) {
        rollers.set(state.rollerSpeed);
    }

    public void setState(State state) {
        PID.setSetpoint(state.position);
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
