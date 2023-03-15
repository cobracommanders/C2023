package org.team498.C2023.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team498.C2023.State;

import static org.team498.C2023.Ports.IntakeRollers.*;

public class IntakeRollers extends SubsystemBase {
    private final TalonFX bottom;
    private final TalonFX top;
    private final CANSparkMax third;

    private final PIDController topPID;
    private final PIDController bottomPID;

    private IntakeRollers() {
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

        bottomPID = new PIDController(0, 0, 0);
        topPID = new PIDController(0, 0, 0);
    }

    @Override
    public void periodic() {
        // bottomRoller.set(ControlMode.PercentOutput, bottomRollerPID.calculate(bottomRoller.getSelectedSensorVelocity()));
        // topRoller.set(ControlMode.PercentOutput, topRollerPID.calculate(topRoller.getSelectedSensorVelocity()));
        SmartDashboard.putNumber("top roller current", top.getStatorCurrent());
        SmartDashboard.putNumber("bottom roller current", bottom.getStatorCurrent());
    }

    public void setState(State.IntakeRollers state) {
        bottom.set(ControlMode.PercentOutput, state.bottomRollerSpeed);
        top.set(ControlMode.PercentOutput, state.topRollerSpeed);
        third.set(state.thirdRollerSpeed);

        // bottomRollerPID.setSetpoint(state.bottomRollerSpeed);
        // topRollerPID.setSetpoint(state.topRollerSpeed);
    }

    private static IntakeRollers instance;

    public static IntakeRollers getInstance() {
        if (instance == null) {
            instance = new IntakeRollers();
        }

        return instance;
    }
}
