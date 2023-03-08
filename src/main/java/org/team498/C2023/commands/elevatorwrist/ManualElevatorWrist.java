package org.team498.C2023.commands.elevatorwrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team498.C2023.subsystems.ElevatorWrist;

import java.util.function.DoubleSupplier;

public class ManualElevatorWrist extends CommandBase {
    private final ElevatorWrist wrist = ElevatorWrist.getInstance();
    private final DoubleSupplier speedSupplier;

    public ManualElevatorWrist(DoubleSupplier speedSupplier) {
        this.speedSupplier = speedSupplier;
        addRequirements(wrist);
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
