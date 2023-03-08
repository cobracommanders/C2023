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
import org.team498.C2023.RobotState;
import org.team498.C2023.State;
import org.team498.C2023.StateTables;
import org.team498.lib.util.LinearInterpolator;

import static org.team498.C2023.Constants.WristConstants.*;
import static org.team498.C2023.Ports.ElevatorWrist.ENCODER_PORT;
import static org.team498.C2023.Ports.ElevatorWrist.WRIST;

public class ElevatorWrist extends SubsystemBase {
    private final CANSparkMax wrist;
    private final DutyCycle encoder;

    private final PIDController PID;

    private final LinearInterpolator interpolator;

    private ControlMode controlMode = ControlMode.PID;
    private double speed = 0;

    private double simAngle = 0;

    public enum ControlMode {
        PID,
        MANUAL
    }

    private ElevatorWrist() {
        wrist = new CANSparkMax(WRIST, MotorType.kBrushless);
        wrist.restoreFactoryDefaults();
        wrist.setIdleMode(IdleMode.kBrake);
        wrist.setInverted(true);

        encoder = new DutyCycle(new DigitalInput(ENCODER_PORT));

        PID = new PIDController(P, I, D);
        PID.setTolerance(0);

        interpolator = new LinearInterpolator(StateTables.wristAngleTable);

        SmartDashboard.putNumber("Wrist PID", 0);

    }

    @Override
    public void periodic() {
        // PID.setSetpoint(SmartDashboard.getNumber("Wrist PID", 0));
        double speed;
        if (controlMode == ControlMode.PID) {
            speed = PID.calculate(getAngle());
        } else {
            speed = this.speed;
        }
        wrist.set(speed);

        SmartDashboard.putData(this);
        SmartDashboard.putNumber("Wrist Angle", getAngle());
        SmartDashboard.putBoolean("Wrist at Setpoint", atSetpoint());
        SmartDashboard.putNumber("Wrist Error", PID.getPositionError());
    }

    public void setState(State.ElevatorWrist state) {
        this.controlMode = ControlMode.PID;
        if (state == State.ElevatorWrist.INTERPOLATE) {
            PID.setSetpoint(interpolator.getInterpolatedValue(Drivetrain.getInstance().getNextDistanceToTag()));
        } else {
            PID.setSetpoint(state.setpoint);
        }
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

    public boolean atSetpoint() {
        return Math.abs(PID.getSetpoint() - getAngle()) < 0.02;
    }

    public double getPower() {
        return wrist.get();
    }

    public void setSimAngle(double position) {
        simAngle = (position / 360);
    }

    public void setToNextState() {
        setState(RobotState.getInstance().getCurrentState().elevatorWrist);
    }

    private static ElevatorWrist instance;

    public static ElevatorWrist getInstance() {
        if (instance == null) {
            instance = new ElevatorWrist();
        }

        return instance;
    }
}
