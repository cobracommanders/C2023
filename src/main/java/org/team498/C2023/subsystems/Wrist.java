package org.team498.C2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team498.C2023.RobotState;

import java.util.LinkedHashMap;
import java.util.Map;

import static org.team498.C2023.Constants.WristConstants.*;
import static org.team498.C2023.Ports.Wrist.ENCODER_PORT;
import static org.team498.C2023.Ports.Wrist.WRIST;

public class Wrist extends SubsystemBase {
    private final CANSparkMax wrist;

    private final PIDController PID;
    private final DutyCycle encoder;

    private ControlMode controlMode;
    private double speed = 0;

    private final ShuffleboardLayout tab = Shuffleboard.getTab("Tuning").getLayout("Wrist Tuning", BuiltInLayouts.kList);
    private final Map<State, GenericEntry> entries = new LinkedHashMap<>();
    private final GenericEntry error = Shuffleboard.getTab("Tuning").add("Wrist Error", 0).getEntry();
    private final GenericEntry setpoint = Shuffleboard.getTab("Tuning").add("Wrist Setpoint", 0).getEntry();
    private final GenericEntry position = Shuffleboard.getTab("Tuning").add("Wrist Position", 0).getEntry();

    public void initShuffleboard() {
        for (State state : State.values()) {
            entries.put(state, tab.addPersistent(state.name(), state.setpoint).getEntry());
        }
    }

    public void updateFromShuffleboard() {
        for (State state : State.values()) {
            state.setpoint = entries.get(state).get().getDouble();
        }
    }

    public enum State {
        COLLECT_CONE_CONEARISER(0),
        COLLECT_CUBE_CONEARISER(0),
        COLLECT_CONE_SUBSTATION(0.102522),
        COLLECT_CUBE_SUBSTATION(0),
        PASS_CONE(0),
        PASS_CUBE(0),
        LOW_CONE(0),
        LOW_CUBE(0),
        MID_CONE(0.122934),
        MID_CUBE(0.056553),
        TOP_CONE(0.218485),
        TOP_CUBE(0.031152),
        UP(0.25),
        AUTO_SHOT(0),
        IDLE(-(1.0 / 12.0));

        private double setpoint;

        State(double setpoint) {
            this.setpoint = setpoint;
        }
    }

    public enum ControlMode {
        PID,
        MANUAL
    }

    private Wrist() {
        wrist = new CANSparkMax(WRIST, MotorType.kBrushless);

        wrist.restoreFactoryDefaults();

        wrist.setIdleMode(IdleMode.kBrake);

        wrist.setInverted(true);

        PID = new PIDController(P, I, D);

        encoder = new DutyCycle(new DigitalInput(ENCODER_PORT));

        setState(State.IDLE);
        initShuffleboard();
    }

    @Override
    public void periodic() {
        double speed = 0;
        if (controlMode == ControlMode.PID) {
            speed = PID.calculate(getAngle());
        } else {
            speed = this.speed;
        }
        wrist.set(speed);
        // wrist.set(PID.calculate(getAngle())/*+ (Math.cos(Math.toRadians(getAngle())) * F)*/);
        SmartDashboard.putNumber("Wrist encoder", getAngle());
        SmartDashboard.putData(this);

        error.setDouble(PID.getPositionError());
        setpoint.setDouble(PID.getSetpoint());
        position.setDouble(encoder.getOutput());

        updateFromShuffleboard();
    }

    public void setControlMode(ControlMode mode) {
        this.controlMode = mode;
    }

    public double getAngle() {
        return encoder.getOutput() - 0.261493;
    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }

    public State getNextScoringPosition() {
        switch (RobotState.getInstance().getCurrentScoringHeight()) {
            case LOW:
                if (RobotState.getInstance().inConeMode()) {
                    return State.LOW_CONE;
                }
                return State.LOW_CUBE;
            case MID:
                if (RobotState.getInstance().inConeMode()) {
                    return State.MID_CONE;
                }
                return State.MID_CUBE;
            case TOP:
                if (RobotState.getInstance().inConeMode()) {
                    return State.TOP_CONE;
                }
                return State.TOP_CUBE;
            default:
                return State.IDLE;
        }
    }

    public boolean atSetpoint() {
        return PID.atSetpoint();
    }

    public void setState(State state) {
        setControlMode(ControlMode.PID);
        PID.setSetpoint(state.setpoint);
    }

    private static Wrist instance;

    public static Wrist getInstance() {
        if (instance == null) {
            instance = new Wrist();
        }

        return instance;
    }
}
