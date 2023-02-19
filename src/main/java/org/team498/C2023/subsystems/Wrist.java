package org.team498.C2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.team498.C2023.Constants.WristConstants.*;
import static org.team498.C2023.Ports.Wrist.ENCODER_PORT;
import static org.team498.C2023.Ports.Wrist.WRIST;

import org.team498.C2023.Robot;

public class Wrist extends SubsystemBase {
    private final CANSparkMax wrist;

    private final PIDController PID;

    private final DutyCycle encoder;

    public enum State {
        COLLECT_CONE_CONEARISER(0),
        COLLECT_CUBE_CONEARISER(0),
        COLLECT_CONE_SUBSTATION(0.102522),
        COLLECT_CUBE_SUBSTATION(0),
        PASS_CONE(0),
        PASS_CUBE(0),
        LOW_CONE(0),
        LOW_CUBE(0),
        MID_CONE(0.204586),
        MID_CUBE(0.036977),
        TOP_CONE(0.287820),
        TOP_CUBE(0.031152),
        UP(0.25),
        AUTO_SHOT(0),
        IDLE(-(1.0/12.0));

        private final double position;

        State(double position) {
            this.position = position;
        }
    }

    private Wrist() {
        wrist = new CANSparkMax(WRIST, MotorType.kBrushless);

        wrist.restoreFactoryDefaults();

        wrist.setIdleMode(IdleMode.kBrake);

        wrist.setInverted(true);

        PID = new PIDController(P, I, D);

        // encoder = new DutyCycle(ENCODER_PORT);
        encoder = new DutyCycle(new DigitalInput(ENCODER_PORT));
        // encoder.setDutyCycleRange(1 / 1025, 1024 / 1025);

        setState(State.IDLE);
    }

    @Override
    public void periodic() {
        // wrist.set(PID.calculate(getAngle())/*+ (Math.cos(Math.toRadians(getAngle())) * F)*/);
        // wrist.set(Robot.robotContainer.operator.rightY());
        SmartDashboard.putNumber("Wrist encoder", getAngle());
        SmartDashboard.putNumber("Wrist tuning", PID.getSetpoint());
    }

    public double getAngle() {
        // return encoder.getAbsolutePosition() * (360.0 / 1024.0);
        return encoder.getOutput() - 0.538736;
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
