package org.team498.C2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.team498.C2023.Ports.ConeARiser.FRONT_BACK;
import static org.team498.C2023.Ports.ConeARiser.LEFT_RIGHT;

import org.team498.C2023.RobotState;
import org.team498.C2023.State;

public class ConeARiser extends SubsystemBase {
    private final CANSparkMax frontBack;
    private final CANSparkMax leftRight;

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

    public void setToNextState() {
        setState(RobotState.getInstance().getCurrentState().coneARiser);
    }

    public void setState(State.ConeARiser state) {
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
