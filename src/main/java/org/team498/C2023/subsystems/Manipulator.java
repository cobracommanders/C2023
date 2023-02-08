package org.team498.C2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team498.C2023.RobotState;

import static org.team498.C2023.Ports.Manipulator.*;

public class Manipulator extends SubsystemBase {
    private final CANSparkMax rollers;
    private final CANSparkMax wrist;
    private final DigitalInput beamBreak;

    public enum Position {
        SHOOT_CUBE(0),
        SHOOT_CONE(0),
        INTAKING(0);

        private final double position;

        Position(int position) {
            this.position = position;
        }
    }

    public enum State {
        INTAKE_CONE(Position.INTAKING, 1),
        INTAKE_CUBE(Position.INTAKING, -0.45),
        SHOOT_CONE(Position.SHOOT_CUBE, -0.75),
        SHOOT_CUBE(Position.INTAKING, 0.5),
        IDLE(Position.INTAKING, 0);

        private final Position position;
        private final double rollerSpeed;

        State(Position position, double rollerSpeed) {
            this.position = position;
            this.rollerSpeed = rollerSpeed;
        }
    }

    private Manipulator() {
        rollers = new CANSparkMax(ROLLERS, MotorType.kBrushless);
        wrist = new CANSparkMax(WRIST, MotorType.kBrushless);

        rollers.restoreFactoryDefaults();
        wrist.restoreFactoryDefaults();

        rollers.setIdleMode(IdleMode.kBrake);
        wrist.setIdleMode(IdleMode.kBrake);

        beamBreak = new DigitalInput(BEAM_BREAK);
    }

    public boolean isStalling() {
        return rollers.getOutputCurrent() > 20;
    }

    public boolean hasGamePiece() {
        return !beamBreak.get();
    }

    public InstantCommand setOuttake(State state) {
        return new InstantCommand(() -> {
            setRollers(state);
        });
    }

    private void setRollers(State state) {
        rollers.set(state.rollerSpeed);
    }

    public InstantCommand intake() {
        return new InstantCommand(() -> {
            if (RobotState.getInstance().hasCone()) {
                setRollers(State.INTAKE_CONE);
            }
            setRollers(State.INTAKE_CUBE);
        });
    }

    public InstantCommand shoot() {
        return new InstantCommand(() -> {
            if (RobotState.getInstance().hasCone()) {
                setRollers(State.SHOOT_CONE);
            }
            setRollers(State.SHOOT_CUBE);
        });
    }

    public InstantCommand stop() {
        return setOuttake(State.IDLE);
    }


    private static Manipulator instance;

    public static Manipulator getInstance() {
        if (instance == null) {
            instance = new Manipulator();
        }

        return instance;
    }
}
