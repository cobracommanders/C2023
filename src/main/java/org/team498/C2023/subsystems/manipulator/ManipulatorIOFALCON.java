package org.team498.C2023.subsystems.manipulator;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.team498.C2023.Ports.Manipulator.*;

public class ManipulatorIOFALCON extends SubsystemBase implements ManipulatorIO {
    private final TalonFX motor;
    private final TalonFX motor2;
    public ManipulatorIOFALCON() {
       motor = new TalonFX(ROLLERS);
       motor2 = new TalonFX(ROLLERS2);
        motor.configfactorydefaults();
        motor.setInverted(false);
        motor2.follow(motor,true);
        motor.setNeutralMode(NeutralMode.Brake);
        motor2.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void updateInputs(ManipulatorIOInputs inputs) {
        inputs.motorCurrentAmps = motor.getOutputCurrent();
        inputs.motorTemp = (motor.getMotorTemperature() * 1.8) + 32;
        inputs.velocityRotationsPerSecond = motor.getEncoder().getVelocity();
    }

    @Override
    public void setSpeed(double speed) {
        motor.set(-speed);
    }
}
