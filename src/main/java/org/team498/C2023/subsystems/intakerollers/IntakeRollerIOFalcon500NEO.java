package org.team498.C2023.subsystems.intakerollers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team498.C2023.State;

import static org.team498.C2023.Ports.IntakeRollers.*;

public class IntakeRollerIOFalcon500NEO extends SubsystemBase implements IntakeRollerIO {
    private final TalonFX bottom;
    private final TalonFX top;
    private final CANSparkMax third;

    public IntakeRollerIOFalcon500NEO() {
        bottom = new TalonFX(BOTTOM_ROLLER);
        top = new TalonFX(TOP_ROLLER);
        third = new CANSparkMax(THIRD_ROLLER, MotorType.kBrushless);

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
        inputs.topCurrentAmps = top.getStatorCurrent();
        inputs.topTemp = (top.getTemperature() * 1.8) + 32;

        inputs.bottomCurrentAmps = top.getStatorCurrent();
        inputs.bottomTemp = (top.getTemperature() * 1.8) + 32;

        inputs.thirdCurrentAmps = third.getOutputCurrent();
        inputs.thirdTemp = (third.getMotorTemperature() * 1.8) + 32;
    }

    @Override
    public void setState(State.IntakeRollers state) {
        bottom.set(ControlMode.PercentOutput, state.bottomRollerSpeed);
        top.set(ControlMode.PercentOutput, state.topRollerSpeed);
        third.set(state.thirdRollerSpeed);
    }
}
