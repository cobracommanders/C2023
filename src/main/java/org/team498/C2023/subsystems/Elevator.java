package org.team498.C2023.subsystems;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team498.C2023.RobotState;
import org.team498.C2023.StateTables;
import org.team498.C2023.State;
import org.team498.lib.util.LinearInterpolator;

import static org.team498.C2023.Constants.ElevatorConstants.*;
import static org.team498.C2023.Ports.Elevator.F_ELEVATOR_ID;
import static org.team498.C2023.Ports.Elevator.B_ELEVATOR_ID;

public class Elevator extends SubsystemBase {
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;

    private final ProfiledPIDController PID;

    private final ElevatorFeedforward feedforward = new ElevatorFeedforward(S, G, V);

    private final LinearInterpolator interpolator;

    private ControlMode controlMode = ControlMode.PID;
    private double speed = 0;

    public enum ControlMode {
        PID,
        MANUAL
    }

    private Elevator() {
        leftMotor = new TalonFX(F_ELEVATOR_ID);
        rightMotor = new TalonFX(B_ELEVATOR_ID);

        leftMotor.setNeutralMode(NeutralMode.Brake);
        rightMotor.setNeutralMode(NeutralMode.Brake);

        leftMotor.setInverted(true);
        rightMotor.setInverted(true);

        rightMotor.follow(leftMotor, FollowerType.PercentOutput);

        PID = new ProfiledPIDController(P, I, D, new TrapezoidProfile.Constraints(5, 4));
        PID.reset(0);
        PID.setTolerance(0);

        interpolator = new LinearInterpolator(StateTables.elevatorHeightTable);

        SmartDashboard.putNumber("Elevator PID", 0);
    }

    @Override
    public void periodic() {
        // PID.setGoal(SmartDashboard.getNumber("Elevator PID", 0));
        double speed;
        if (controlMode == ControlMode.PID) {
            speed = PID.calculate(getPosition());
        } else {
            speed = this.speed;
        }
        leftMotor.set(TalonFXControlMode.PercentOutput, speed + feedforward.calculate(speed));

        SmartDashboard.putData(this);
        SmartDashboard.putNumber("Elevator Position", getPosition());
        SmartDashboard.putBoolean("Elevator at Setpoint", atSetpoint());
        SmartDashboard.putNumber("Elevator Error", PID.getGoal().position - getPosition());
    }

    public boolean aboveIntakeHeight() {
        return getPosition() > 0.4; //TODO find out the real number here
    }

    public void setState(State.Elevator state) {
        this.controlMode = ControlMode.PID;
        if (state == State.Elevator.INTERPOLATE) {
            PID.setGoal(interpolator.getInterpolatedValue(Drivetrain.getInstance().getNextDistanceToTag()));
        } else {
            PID.setGoal(state.setpoint);
        }
    }

    public void setSpeed(double speed) {
        this.controlMode = ControlMode.MANUAL;
        this.speed = speed;
    }

    public double getPosition() {
        return (leftMotor.getSelectedSensorPosition() / 2048) / MOTOR_ROTATION_TO_METERS;
    }

    public boolean atSetpoint() {
        return Math.abs(PID.getGoal().position - getPosition()) < 0.025;
    }

    public double getPower() {
        return leftMotor.getMotorOutputPercent();
    }

    public void setEncoderPosition(double position) {
        leftMotor.setSelectedSensorPosition(position);
    }

    public void setToNextState() {
        setState(RobotState.getInstance().getCurrentState().elevator);
    }

    private static Elevator instance;

    public static Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
        }

        return instance;
    }

}
