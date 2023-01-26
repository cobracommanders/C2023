package org.team498.C2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.team498.C2023.Ports.Prototype.PROTOTYPE;

public class Prototype extends SubsystemBase {
    private final CANSparkMax prototype;

    private Prototype() {
        prototype = new CANSparkMax(PROTOTYPE, MotorType.kBrushless);
    }

    public void setSpeed(double speed) {
        prototype.set(speed);
    }


    private static Prototype instance;

    public static Prototype getInstance() {
        if (instance == null) {
            instance = new Prototype();
        }

        return instance;
    }
}
