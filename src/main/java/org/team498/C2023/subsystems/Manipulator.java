package org.team498.C2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team498.C2023.RobotState;

import static org.team498.C2023.Ports.Manipulator.ROLLERS;

public class Manipulator extends SubsystemBase {
    private final CANSparkMax rollers;

    public enum State {
        COLLECT(1, -1),
        SCORE(-1, 1),
        AUTO_SHOT(-1, 0.5),
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
    }

    public State getNextState() {
        return switch (RobotState.getInstance().getNextHeight()) {
            case LOW, MID, TOP -> State.SCORE;
            case DOUBLE_SS, SINGLE_SS -> State.COLLECT;
        };
    }

    public boolean isStalling() {
        return rollers.getOutputCurrent() > 20;
    }

    public void setState(State state) {
        rollers.set(RobotState.getInstance().inConeMode()
                    ? state.speedCone
                    : state.speedCube);
    }

    public void setToNextState() {
        setState(getNextState());
    }


    private static Manipulator instance;

    public static Manipulator getInstance() {
        if (instance == null) {
            instance = new Manipulator();
        }

        return instance;
    }
}
