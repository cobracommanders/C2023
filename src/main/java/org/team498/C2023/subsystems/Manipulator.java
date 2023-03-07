package org.team498.C2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team498.C2023.RobotState;
import org.team498.C2023.State;
import org.team498.C2023.StateTables;
import org.team498.lib.util.LinearInterpolator;

import static org.team498.C2023.Ports.Manipulator.ROLLERS;

public class Manipulator extends SubsystemBase {
    private final CANSparkMax rollers;

    private final PIDController PID;
    private final LinearInterpolator interpolator;

    private Manipulator() {
        rollers = new CANSparkMax(ROLLERS, MotorType.kBrushless);
        rollers.restoreFactoryDefaults();
        rollers.setInverted(true);
        rollers.setIdleMode(IdleMode.kCoast);

        PID = new PIDController(0, 0, 0);
        interpolator = new LinearInterpolator(StateTables.shooterRPMTable);
    }

    public boolean isStalling() {
        return rollers.getOutputCurrent() > 20;
    }

    public void setState(State.Manipulator state) {
        if (state == State.Manipulator.INTERPOLATE) {
            PID.setSetpoint(interpolator.getInterpolatedValue(Drivetrain.getInstance().getNextDistanceToTag()));
            rollers.set(PID.calculate(getRPM()));
        } else {
            rollers.set(state.setpoint);
        }
    }

    @Override
    public void periodic() {
        // rollers.set(PID.calculate(getRPM()));
    }

    public void setToNextState() {
        setState(RobotState.getInstance().getCurrentState().manipulator);
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
