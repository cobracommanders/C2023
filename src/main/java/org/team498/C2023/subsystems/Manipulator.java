package org.team498.C2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team498.C2023.RobotState;
import org.team498.C2023.StateTables;
import org.team498.lib.util.LinearInterpolator;

import static org.team498.C2023.Ports.Manipulator.ROLLERS;

public class Manipulator extends SubsystemBase {
    private final CANSparkMax rollers;

    private final PIDController PID;
    private final LinearInterpolator interpolator;

    public enum State {
        CONEARISER(1, -1),
        SINGLE_SS(1, -1),
        DOUBLE_SS(1, -1),

        LOW(-1, 0.9),
        MID(-1, 0.5),
        TOP(-1, 0.5),

        AUTO_SHOT(-1, 0.5),
        INTERPOLATE(0, 0),
        HOLD(1, -1),
        SPIT(-1, 0.9),
        IDLE(0, 0);

        private final double speedCone;
        private final double speedCube;

        State(double speedCone, double speedCube) {
            this.speedCone = speedCone;
            this.speedCube = speedCube;
        }
    }

    private Manipulator() {
        rollers = new CANSparkMax(ROLLERS, MotorType.kBrushless);
        rollers.restoreFactoryDefaults();
        rollers.setInverted(true);
        rollers.setIdleMode(IdleMode.kCoast);

        PID = new PIDController(0, 0, 0);
        interpolator = new LinearInterpolator(StateTables.shooterRPMTable);
    }

    public State getNextState() {
        return switch (RobotState.getInstance().getNextHeight()) {
            case LOW -> State.LOW;
            case MID -> State.MID;
            case TOP -> State.TOP;
            case DOUBLE_SS -> State.DOUBLE_SS;
            case SINGLE_SS -> State.SINGLE_SS;
            case INTAKE -> State.CONEARISER;
            case INTERPOLATE -> State.INTERPOLATE;
        };
    }

    public boolean isStalling() {
        return rollers.getOutputCurrent() > 20;
    }

    public void setState(State state) {
        if (state == State.INTERPOLATE) {
        PID.setSetpoint(interpolator.getInterpolatedValue(Drivetrain.getInstance().getNextDistanceToTag()));
        rollers.set(PID.calculate(getRPM()));
        } else {
        rollers.set(RobotState.getInstance().inConeMode()
        ? state.speedCone
        : state.speedCube);
        }

        // PID.setSetpoint(RobotState.getInstance().inConeMode()
        //         ? state.speedCone
        //         : state.speedCube);
    }

    @Override
    public void periodic() {
        // rollers.set(PID.calculate(getRPM()));
    }

    public void setToNextState() {
        setState(getNextState());
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
