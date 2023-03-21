package org.team498.C2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team498.C2023.Robot;
import org.team498.C2023.RobotPositions;
import org.team498.C2023.State;
import org.team498.C2023.ShootTables;

import static org.team498.C2023.Constants.WristConstants.*;
import static org.team498.C2023.Ports.ElevatorWrist.ENCODER_PORT;
import static org.team498.C2023.Ports.ElevatorWrist.WRIST;

public class ElevatorWrist extends SubsystemBase {
    private final CANSparkMax wrist;
    private final DutyCycle encoder;

    public final PIDController PID;

    private State.ElevatorWrist currentState = State.ElevatorWrist.IDLE_CUBE;

    private ControlMode controlMode = ControlMode.PID;
    private double speed = 0;

    private double simAngle = 0;

    private boolean isEnabled = true;

    public enum ControlMode {
        PID,
        MANUAL
    }

    private ElevatorWrist() {
        wrist = new CANSparkMax(WRIST, MotorType.kBrushless);
        wrist.restoreFactoryDefaults();
        wrist.setIdleMode(IdleMode.kBrake);
        wrist.setInverted(true);
        wrist.burnFlash(); 

        encoder = new DutyCycle(new DigitalInput(ENCODER_PORT));

        PID = new PIDController(P, I, D);
        PID.setTolerance(0);

        SmartDashboard.putNumber("Wrist PID", 0);
    }

    @Override
    public void periodic() {
        PID.setSetpoint(getSetpoint(currentState, RobotPositions.getFutureScoringNodeDistance()));

        double speed;
        if (controlMode == ControlMode.PID) {
            speed = PID.calculate(getAngle());
        } else {
            speed = this.speed;
        }

        // if (isEnabled) {
            // wrist.set(speed);
        // } else {
            // wrist.set(0);
        // }

        SmartDashboard.putData(this);
        SmartDashboard.putNumber("Wrist Angle", getAngle());
        SmartDashboard.putBoolean("Wrist at Setpoint", atSetpoint());
        SmartDashboard.putNumber("Wrist Error", PID.getPositionError());
        SmartDashboard.putNumber("Wrist Output", speed);
        SmartDashboard.putNumber("Wrist PID Setpoint", PID.getSetpoint());
    }

    public void setEnabled(boolean enabled) {
        this.isEnabled = enabled;
    }

    public boolean getEnabled() {
        return isEnabled;
    }

    public void setState(State.ElevatorWrist state) {
        this.controlMode = ControlMode.PID;
        this.currentState = state;
        PID.setSetpoint(state.setpoint);
    }

    public void setSpeed(double speed) {
        this.controlMode = ControlMode.MANUAL;
        this.speed = speed;
    }

    public double getAngle() {
        double angle = Robot.isReal()
                ? encoder.getOutput() + 0.5
                : simAngle;

        if (angle < 1)
            angle += 1;

        return angle - 1.370912;
    }

    private double getSetpoint(State.ElevatorWrist state, double interpolatedValue) {
        return switch (state) {
            case SHOOT_DRIVE_CUBE_MID -> ShootTables.midCube.wristAngle.getInterpolatedValue(interpolatedValue);
            case SHOOT_DRIVE_CUBE_TOP -> ShootTables.topCube.wristAngle.getInterpolatedValue(interpolatedValue);
            case SHOOT_DRIVE_CONE_MID -> ShootTables.midCone.wristAngle.getInterpolatedValue(interpolatedValue);
            default -> state.setpoint;
        };
    }

    public boolean atSetpoint() {
        // if (isEnabled) {
            // return Math.abs(PID.getSetpoint() - getAngle()) < 0.02;
        // } else {
            return true;
        // }
    }

    public double getPower() {
        return wrist.get();
    }

    public void setSimAngle(double position) {
        simAngle = (position / 360);
    }

    private static ElevatorWrist instance;

    public static ElevatorWrist getInstance() {
        if (instance == null) {
            instance = new ElevatorWrist();
        }

        return instance;
    }
}
