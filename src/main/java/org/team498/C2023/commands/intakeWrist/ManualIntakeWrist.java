package org.team498.C2023.commands.intakeWrist;

import java.util.function.DoubleSupplier;

import org.team498.C2023.subsystems.IntakeWrist;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ManualIntakeWrist extends CommandBase {
    private DoubleSupplier speed;

    public ManualIntakeWrist(DoubleSupplier speed){
        addRequirements(IntakeWrist.getInstance());
        this.speed = speed;
    }
    @Override
    public void initialize() {
        IntakeWrist.getInstance().setManual(true);
    }

    @Override
    public void execute() {
        IntakeWrist.getInstance().setManualSpeed(-speed.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        IntakeWrist.getInstance().setManualSpeed(0);
        IntakeWrist.getInstance().setManual(false);
    }

}
