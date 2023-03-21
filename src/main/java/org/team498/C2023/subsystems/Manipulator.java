package org.team498.C2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team498.C2023.State;
import org.team498.C2023.ShootTables;

import static org.team498.C2023.Ports.Manipulator.ROLLERS;

import org.team498.C2023.RobotPosition;

public class Manipulator extends SubsystemBase {
    private final CANSparkMax rollers;

    private State.Manipulator currentState = State.Manipulator.IDLE;

    private Manipulator() {
        rollers = new CANSparkMax(ROLLERS, MotorType.kBrushless);
        rollers.restoreFactoryDefaults();
        rollers.setInverted(true);
        rollers.setIdleMode(IdleMode.kCoast);
    }

    public boolean isStalling() {
        return rollers.getOutputCurrent() > 20;
    }

    public void setState(State.Manipulator state) {
        this.currentState = state;
    }

    @Override
    public void periodic() {        
        rollers.set(getSetpoint(currentState, RobotPosition.getFutureScoringNodeDistance()));
    }

    private double getSetpoint(State.Manipulator state, double interpolatedValue) {
        return switch (state) {
            case SHOOT_DRIVE_CUBE_MID -> ShootTables.midCube.shooterRPM.getInterpolatedValue(interpolatedValue);
            case SHOOT_DRIVE_CUBE_TOP -> ShootTables.topCube.shooterRPM.getInterpolatedValue(interpolatedValue);
            case SHOOT_DRIVE_CONE_MID -> ShootTables.midCone.shooterRPM.getInterpolatedValue(interpolatedValue);
            default -> state.setpoint;
        };
    }

    public double getRPM() {
        return rollers.getEncoder().getVelocity();
    }

    private static Manipulator instance;

    public static Manipulator getInstance() {
        if (instance == null) {
            instance = new Manipulator();
        }

        return instance;
    }
}
