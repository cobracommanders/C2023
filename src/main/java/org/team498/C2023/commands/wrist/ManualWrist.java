package org.team498.C2023.commands.wrist;

import java.util.function.DoubleSupplier;

import org.team498.C2023.subsystems.Wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ManualWrist extends CommandBase {
    private final Wrist wrist = Wrist.getInstance();
    private final DoubleSupplier speedSupplier;

    public ManualWrist(DoubleSupplier speedSupplier) {
        this.speedSupplier = speedSupplier;
        addRequirements(wrist);
    }

    @Override
    public void initialize() {
        wrist.setControlMode(Wrist.ControlMode.MANUAL);
    }

    @Override
    public void execute() {
        wrist.setSpeed(speedSupplier.getAsDouble());        
    }

    @Override
    public void end(boolean interrupted) {
        wrist.setSpeed(0);
    }
}
