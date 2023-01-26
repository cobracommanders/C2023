package org.team498.C2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.team498.C2023.Constants.ElevatorConstants.*;
import static org.team498.C2023.Ports.Elevator.*;

import org.opencv.core.Point;

public class Elevator extends SubsystemBase {
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;
    private final RelativeEncoder encoder;

    private final ProfiledPIDController profiledController;
    private ControlMode controlMode;
    private double speed;
    private final DigitalInput limit;
    public Position nextHeight = Position.HIGH;

    public enum Position {
        FLOOR(0),
        SUBSTATION(0),
        LOW(0),
        MID(0),
        HIGH(0),
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
        speed = 0;

        leftMotor = new CANSparkMax(L_ELEVATOR_ID, MotorType.kBrushless);
        rightMotor = new CANSparkMax(R_ELEVATOR_ID, MotorType.kBrushless);
        encoder = leftMotor.getEncoder();
        rightMotor.follow(leftMotor, false);

        profiledController = new ProfiledPIDController(P, I, D, new TrapezoidProfile.Constraints(7, Math.pow(7, 2)));

        limit = new DigitalInput(ELEVATOR_LIMIT);
    }

    @Override
    public void periodic() {
        double speed;
        if (controlMode == ControlMode.PID) speed = profiledController.calculate(encoder.getPosition());
        else {
            speed = this.speed;
        }

        if ((Math.signum(speed) == -1) && !limit.get()) {
            speed = 0;
        }

        leftMotor.set(speed);
    }

    public InstantCommand setNextHeight(Position height) {
        return new InstantCommand(() -> nextHeight = height);
    }

    public InstantCommand setPosition(Position position) {
        InstantCommand command = new InstantCommand(() -> {
            setControlMode(ControlMode.PID);
            profiledController.setGoal(position.setpoint);
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
        return profiledController.atGoal();
    }

    public boolean getLimitSwitch() {
        return !limit.get();
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
