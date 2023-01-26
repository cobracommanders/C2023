package org.team498.C2023.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.team498.C2023.Constants.IntakeConstants.*;
import static org.team498.C2023.Ports.Intake.*;

public class Intake extends SubsystemBase {
    private final CANSparkMax rollers;
    private final CANSparkMax leftWrist;
    private final CANSparkMax rightWrist;

    private final AbsoluteEncoder encoder;

    private final PIDController PID;
    private final ArmFeedforward feedforward;

    public enum Position {
        IN(0),
        OUT(0);

        private final double setpoint;

        Position(double setpoint) {
            this.setpoint = setpoint;
        }
    }

    public enum State {
        INTAKE,
        OUTTAKE,
        IDLE
    }

    private Intake() {
        rollers = new CANSparkMax(ROLLERS, MotorType.kBrushless);
        leftWrist = new CANSparkMax(L_WRIST, MotorType.kBrushless);
        rightWrist = new CANSparkMax(R_WRIST, MotorType.kBrushless);

        rightWrist.follow(leftWrist, true);

        encoder = leftWrist.getAbsoluteEncoder(Type.kDutyCycle);

        PID = new PIDController(P, I, D);
        feedforward = new ArmFeedforward(S, G, V);
    }

    @Override
    public void periodic() {
        leftWrist.set(PID.calculate(getPositionDegrees()) + feedforward.calculate(Math.toRadians(PID.getSetpoint()), 0));
    }

    private InstantCommand setRollers(double speed) {
        return new InstantCommand(() -> rollers.set(speed));
    }

    private InstantCommand setPosition(Position position) {
        return new InstantCommand(() -> PID.setSetpoint(position.setpoint));
    }

    public ParallelCommandGroup setIntakeMode(State state) {
        ParallelCommandGroup output = new ParallelCommandGroup();
        switch (state) {
            case INTAKE:
                output = new ParallelCommandGroup(setRollers(1), setPosition(Position.OUT));
                break;
            case OUTTAKE:
                output = new ParallelCommandGroup(setRollers(-1), setPosition(Position.OUT));
                break;
            case IDLE:
                output = new ParallelCommandGroup(setRollers(0), setPosition(Position.IN));
                break;
        }
        output.addRequirements(this);
        output.setName("Intake Command: " + state.name());
        return output;
    }

    private double getPositionDegrees() {
        return encoder.getPosition() * (360.0 / 8192.0);
    }


    private static Intake instance;

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }

        return instance;
    }
}
