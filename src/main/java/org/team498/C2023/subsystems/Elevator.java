package org.team498.C2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team498.C2023.RobotState;

import java.util.LinkedHashMap;
import java.util.Map;

import static org.team498.C2023.Constants.ElevatorConstants.*;
import static org.team498.C2023.Ports.Elevator.L_ELEVATOR_ID;
import static org.team498.C2023.Ports.Elevator.R_ELEVATOR_ID;

public class Elevator extends SubsystemBase {

    private static Elevator instance;

    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;
    private final RelativeEncoder encoder;
    private final ShuffleboardLayout tab = Shuffleboard.getTab("Tuning").getLayout("Elevator Tuning", BuiltInLayouts.kList);
    private final Map<State, GenericEntry> entries = new LinkedHashMap<>();
    private final GenericEntry error = Shuffleboard.getTab("Tuning").add("Elevator Error", 0).getEntry();
    private final GenericEntry setpoint = Shuffleboard.getTab("Tuning").add("Elevator Setpoint", 0).getEntry();
    private final GenericEntry position = Shuffleboard.getTab("Tuning").add("Elevator Position", 0).getEntry();

    private enum State {
        LOW_CONE(0),
        LOW_CUBE(0),
        MID_CONE(21),
        MID_CUBE(7),
        TOP_CONE(28.833111),
        TOP_CUBE(20),

        PASS_CONE(0),
        PASS_CUBE(0),
        DOUBLE_SS_CONE(21.452280),
        DOUBLE_SS_CUBE(0),
        AUTO_SHOT(0),
        IDLE(0);

        public double setpoint;

        State(double setpoint) {
            this.setpoint = setpoint;
        }
    }

    public enum ControlMode {
        PID,
        MANUAL
    }

    private final PIDController PID;
    private ControlMode controlMode;
    private double speed = 0;

    private Elevator() {
        controlMode = ControlMode.MANUAL;

        leftMotor = new CANSparkMax(L_ELEVATOR_ID, MotorType.kBrushless);
        rightMotor = new CANSparkMax(R_ELEVATOR_ID, MotorType.kBrushless);

        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();

        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);

        encoder = leftMotor.getEncoder();
        rightMotor.follow(leftMotor, true);

        PID = new PIDController(P, I, D);

        initShuffleboard();
    }

    private void initShuffleboard() {
        for (State state : State.values()) {
            entries.put(state, tab.addPersistent(state.name(), state.setpoint).getEntry());
        }
    }

    private void updateFromShuffleboard() {
        for (State state : State.values()) {
            state.setpoint = entries.get(state).get().getDouble();
        }
    }

    public void setControlMode(ControlMode mode) {
        this.controlMode = mode;
    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }

    public boolean atSetpoint() {
        return PID.getPositionError() < 0.5;
    }


    @Override
    // Periodically check the setpoint of the current desired scoring position, and adjust
    //the elevator height if it does not match the current elevator height.
    public void periodic() {
        double speed;
        if (controlMode == ControlMode.PID) {
            //get the setpoint of the current scoring position, set speed based on our
            //current position
            speed = PID.calculate(encoder.getPosition(), getNextScoringPositionSetPoint());
        } else {
            speed = this.speed;
        }

        SmartDashboard.putNumber("Elevator Position", encoder.getPosition());
        SmartDashboard.putData(this);

        SmartDashboard.putNumber("Low cube setpoint", State.LOW_CUBE.setpoint);

        SmartDashboard.putBoolean("Elevator at setpoint", atSetpoint());
        leftMotor.set(speed /*- Math.pow(((0.171 * encoder.getPosition()) / -4.547614) + 0.04, 2)*/);

        error.setDouble(PID.getPositionError());
        setpoint.setDouble(PID.getSetpoint());
        position.setDouble(encoder.getPosition());
    
        updateFromShuffleboard();
    }

    private double getNextScoringPositionSetPoint() {
        switch (RobotState.getInstance().getCurrentScoringHeight()) {
            case LOW:
                if (RobotState.getInstance().inConeMode()) {
                    return State.LOW_CONE.setpoint;
                }
                return State.LOW_CUBE.setpoint;
            case MID:
                if (RobotState.getInstance().inConeMode()) {
                    return State.MID_CONE.setpoint;
                }
                return State.MID_CUBE.setpoint;
            case TOP:
                if (RobotState.getInstance().inConeMode()) {
                    return State.TOP_CONE.setpoint;
                }
                return State.TOP_CUBE.setpoint;
            case DOUBLE_SS:
                if (RobotState.getInstance().inConeMode()) {
                    return State.DOUBLE_SS_CONE.setpoint;
                }
                return State.DOUBLE_SS_CUBE.setpoint;
            case PASS_MODE:
                if (RobotState.getInstance().inConeMode()) {
                    return State.PASS_CONE.setpoint;
                }
                return State.PASS_CUBE.setpoint;
            default:
                return State.IDLE.setpoint;
        }
    }

    public static Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
        }
        return instance;
    }

}
