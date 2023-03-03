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
import org.team498.C2023.StateTables;
import org.team498.lib.util.LinearInterpolator;

import static org.team498.C2023.Constants.WristConstants.*;
import static org.team498.C2023.Ports.Wrist.ENCODER_PORT;
import static org.team498.C2023.Ports.Wrist.WRIST;

public class Wrist extends SubsystemBase {
    private final CANSparkMax wrist;
    private final DutyCycle encoder;

    private final PIDController PID;

    private final LinearInterpolator interpolator;

    private ControlMode controlMode;
    private double speed = 0;

    private double simAngle = 0;

    public enum State {
        CONEARISER(0, -0.038142),
        SINGLE_SS(0, 0.05),
        DOUBLE_SS(0.109352, 0),

        LOW(0, 0),
        MID(0.075, -0.05),
        TOP(0.2, -0.05),

        TRAVEL(0.041067, 0.041067),
        INTAKE(0, -0.05),

        AUTO_SHOT(0, 0),
        IDLE(0.0, -0.08333),
        INTERPOLATE(0, 0),

        SPIT(0, -0.038142);

        private final double setpointCone;
        private final double setpointCube;

        State(double setpointCone, double setpointCube) {
            this.setpointCone = setpointCone;
            this.setpointCube = setpointCube;
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

        encoder = new DutyCycle(new DigitalInput(ENCODER_PORT));

        PID = new PIDController(P, I, D);

        interpolator = new LinearInterpolator(StateTables.wristAngleTable);

        SmartDashboard.putNumber("Wrist PID", 0);
    }

    @Override
    public void periodic() {
        this.controlMode= ControlMode.PID;
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

    public State getNextState() {
        return switch (RobotState.getInstance().getNextHeight()) {
            case LOW -> State.LOW;
            case MID -> State.MID;
            case TOP -> State.TOP;
            case DOUBLE_SS -> State.DOUBLE_SS;
            case SINGLE_SS -> State.SINGLE_SS;
            case INTAKE -> State.INTAKE;
            case INTERPOLATE -> State.INTERPOLATE;
        };
    }

    public void setState(State state) {
        this.controlMode = ControlMode.PID;
        if (state == State.INTERPOLATE) {
            PID.setSetpoint(interpolator.getInterpolatedValue(Drivetrain.getInstance().getNextDistanceToTag()));
        }
        else {
            PID.setSetpoint(RobotState.getInstance().inConeMode()
                            ? state.setpointCone
                            : state.setpointCube);
        }
    }

    public void setSpeed(double speed) {
        this.controlMode = ControlMode.MANUAL;
        this.speed = speed;
    }

    public double getAngle() {
        return Robot.isReal()
               ? encoder.getOutput() - 0.261493
               : simAngle;
    }

    public boolean atSetpoint() {
        return PID.atSetpoint();
    }

    public double getPower() {
        return wrist.get();
    }

    public void setSimAngle(double position) {
        simAngle = (position / 360);
    }

    public void setToNextState() {
        setState(getNextState());
    }


    private static Wrist instance;

    public static Wrist getInstance() {
        if (instance == null) {
            instance = new Wrist();
        }

        return instance;
    }
}
