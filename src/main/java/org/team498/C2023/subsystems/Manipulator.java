package org.team498.C2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.team498.C2023.Ports.Manipulator.ROLLERS;

public class Manipulator extends SubsystemBase {
    private final CANSparkMax rollers;

    public enum State {
        COLLECT_CONE(1),
        COLLECT_CUBE(-0.45),
        SCORE_CONE(-1),
        SCORE_CUBE(0.5),
        AUTO_SHOT(0.5),
        IDLE(0);

        private final double rollerSpeed;

        State(double rollerSpeed) {
            this.rollerSpeed = rollerSpeed;
        }
    }

    private Manipulator() {
        rollers = new CANSparkMax(ROLLERS, MotorType.kBrushless);

        rollers.restoreFactoryDefaults();

        rollers.setIdleMode(IdleMode.kBrake);
        setState(State.IDLE);
    }

    public boolean isStalling() {
        return rollers.getOutputCurrent() > 20;
    }

    public void setState(State state) {
        rollers.set(state.rollerSpeed);
    }


    private static Manipulator instance;

    public static Manipulator getInstance() {
        if (instance == null) {
            instance = new Manipulator();
        }

        return instance;
    }
}
