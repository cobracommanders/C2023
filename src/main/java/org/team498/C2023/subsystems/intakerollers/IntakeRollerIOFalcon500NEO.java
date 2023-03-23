package org.team498.C2023.subsystems.intakerollers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.team498.C2023.Ports.IntakeRollers.*;

import org.team498.lib.drivers.LazySparkMax;
import org.team498.lib.drivers.LazyTalonFX;

public class IntakeRollerIOFalcon500NEO extends SubsystemBase implements IntakeRollerIO {
    private final TalonFX bottom;
    private final TalonFX top;
    private final LazySparkMax third;

    public IntakeRollerIOFalcon500NEO() {
        bottom = new TalonFX(BOTTOM_ROLLER);
        top = new TalonFX(TOP_ROLLER);
        third = new LazySparkMax(THIRD_ROLLER, MotorType.kBrushless);

        bottom.setNeutralMode(NeutralMode.Coast);
        top.setNeutralMode(NeutralMode.Coast);
        third.setIdleMode(IdleMode.kCoast);

        bottom.setInverted(true);
        top.setInverted(false);
        third.setInverted(false);

        third.setSmartCurrentLimit(20);
    }

    @Override
    public void updateInputs(IntakeRollerIOInputs inputs) {
        // inputs.topCurrentAmps = top.getStatorCurrent();
        // inputs.topTemp = (top.getTemperature() * 1.8) + 32;

        // inputs.bottomCurrentAmps = top.getStatorCurrent();
        // inputs.bottomTemp = (top.getTemperature() * 1.8) + 32;

        // inputs.thirdCurrentAmps = third.getOutputCurrent();
        // inputs.thirdTemp = (third.getMotorTemperature() * 1.8) + 32;
    }

    @Override
    public void setSpeed(double bottomSpeed, double topSpeed, double thirdSpeed) {
        bottom.set(ControlMode.PercentOutput, bottomSpeed);
        top.set(ControlMode.PercentOutput, topSpeed);
        third.set(thirdSpeed);
    }
}
