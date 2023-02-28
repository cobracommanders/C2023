package org.team498.C2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.team498.C2023.Ports.ConeARiser.FRONT_BACK;
import static org.team498.C2023.Ports.ConeARiser.LEFT_RIGHT;

import org.team498.C2023.RobotState;

public class ConeARiser extends SubsystemBase {
    private final CANSparkMax frontBack;
    private final CANSparkMax leftRight;

    public enum State {
        COLLECT(0.5, 0.5),
        REJECT(-1, -1),
        IDLE(0, 0);

        private final double frontBackSpeed;
        private final double leftRightSpeed;

        State(double frontBackSpeed, double leftRightSpeed) {
            this.frontBackSpeed = frontBackSpeed;
            this.leftRightSpeed = leftRightSpeed;
        }
    }

    private ConeARiser() {
        frontBack = new CANSparkMax(FRONT_BACK, MotorType.kBrushless);
        leftRight = new CANSparkMax(LEFT_RIGHT, MotorType.kBrushless);

        frontBack.restoreFactoryDefaults();
        leftRight.restoreFactoryDefaults();

        frontBack.setSmartCurrentLimit(20);
        leftRight.setSmartCurrentLimit(20);

        frontBack.setIdleMode(IdleMode.kCoast);
        leftRight.setIdleMode(IdleMode.kCoast);
    }

    public State getNextState() {
        return switch (RobotState.getInstance().getNextHeight()) {
            case LOW, MID, TOP, INTERPOLATE -> State.REJECT;
            case DOUBLE_SS, SINGLE_SS -> State.COLLECT;
        };
    }

    public void setToNextState() {
        setState(getNextState());
    }

    public void setState(State state) {
        frontBack.set(state.frontBackSpeed);
        leftRight.set(state.leftRightSpeed);
    }


    private static ConeARiser instance;

    public static ConeARiser getInstance() {
        if (instance == null) {
            instance = new ConeARiser();
        }

        return instance;
    }
}
