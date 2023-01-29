package org.team498.C2023.subsystems;

import static org.team498.C2023.Constants.ElevatorConstants.D;
import static org.team498.C2023.Constants.ElevatorConstants.I;
import static org.team498.C2023.Constants.ElevatorConstants.P;
import static org.team498.C2023.Ports.Elevator.L_ELEVATOR_ID;
import static org.team498.C2023.Ports.Elevator.R_ELEVATOR_ID;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;
    private final RelativeEncoder encoder;

    private final PIDController PID;
    private ControlMode controlMode;
    private double speed = 0;
    // private final DigitalInput limit;
    public Position nextHeight = Position.HIGH;

    public enum Position {
        FLOOR(0),
        SUBSTATION(0),
        LOW(-3),
        MID(-5),
        HIGH(-10),
        DRIVING(0);

        private final double setpoint;

        Position(double setpoint) {
            this.setpoint = setpoint;
        }

    }

    public enum ControlMode {
        PID,
        MANUAL
    }

    private Elevator() {
        controlMode = ControlMode.MANUAL;

        leftMotor = new CANSparkMax(L_ELEVATOR_ID, MotorType.kBrushless);
        rightMotor = new CANSparkMax(R_ELEVATOR_ID, MotorType.kBrushless);

        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);

        encoder = leftMotor.getEncoder();
        rightMotor.follow(leftMotor, false);

        PID = new PIDController(P, I, D);

        // limit = new DigitalInput(ELEVATOR_LIMIT);
    }

    @Override
    public void periodic() {
        double speed;
        if (controlMode == ControlMode.PID)
            speed = PID.calculate(encoder.getPosition());
        else {
            speed = this.speed;
        }

            SmartDashboard.putNumber("Elevator position", encoder.getPosition());
            SmartDashboard.putNumber("Elevator speed", speed);

        // if ((Math.signum(speed) == -1) && !limit.get()) {
        // speed = 0;
        // }

        leftMotor.set(speed);
    }

    public InstantCommand setNextHeight(Position height) {
        return 
        new InstantCommand(() -> nextHeight = height);
    }

    public InstantCommand setPosition(Position position) {
        InstantCommand command = new InstantCommand(() -> {
            SmartDashboard.putNumber("Elevator setpoint", position.setpoint);
            setControlMode(ControlMode.PID);
            PID.setSetpoint(position.setpoint);
        });
        command.addRequirements(this);
        return command;
    }

    public void setControlMode(ControlMode mode) {
        this.controlMode = mode;
    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }

    public boolean atSetpoint() {
        return PID.atSetpoint();
    }

    public boolean getLimitSwitch() {
        // return !limit.get();
        return false;
    }

    public void resetEncoder() {
        encoder.setPosition(0);
    }

    private static Elevator instance;

    public static Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
        }

        return instance;
    }

}
