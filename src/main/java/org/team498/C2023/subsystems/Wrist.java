package org.team498.C2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.team498.C2023.Constants.ManipulatorConstants.*;
import static org.team498.C2023.Ports.Wrist.ENCODER_PORT;
import static org.team498.C2023.Ports.Wrist.WRIST;

public class Wrist extends SubsystemBase {
    private final CANSparkMax wrist;

    private final PIDController PID;

    private final DutyCycleEncoder encoder;

    public enum State {
        COLLECT_CONE(0),
        COLLECT_CUBE(0),
        PASS_CONE(0),
        PASS_CUBE(0),
        SCORE_CONE(0),
        SCORE_CUBE(0),
        AUTO_SHOT(0),
        IDLE(0);

        private final double position;

        State(double position) {
            this.position = position;
        }
    }

    private Wrist() {
        wrist = new CANSparkMax(WRIST, MotorType.kBrushless);

        wrist.restoreFactoryDefaults();

        wrist.setIdleMode(IdleMode.kBrake);

        PID = new PIDController(P, I, D);

        encoder = new DutyCycleEncoder(ENCODER_PORT);
        encoder.setDutyCycleRange(1, 1024);
    }

    @Override
    public void periodic() {
        wrist.set(PID.calculate(getAngle()) + (Math.cos(Math.toRadians(getAngle())) * F));
    }

    public double getAngle() {
        return encoder.getAbsolutePosition() * (360.0 / 1024.0);
    }


    public boolean atSetpoint() {
        return PID.atSetpoint();
    }

    public void setState(State state) {
        PID.setSetpoint(state.position);
    }


    private static Wrist instance;

    public static Wrist getInstance() {
        if (instance == null) {
            instance = new Wrist();
        }

        return instance;
    }
}
