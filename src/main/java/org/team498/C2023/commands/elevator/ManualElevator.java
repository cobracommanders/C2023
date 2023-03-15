package org.team498.C2023.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team498.C2023.subsystems.Elevator;

import java.util.function.DoubleSupplier;

public class ManualElevator extends CommandBase {
    private final Elevator elevator = Elevator.getInstance();
    private final DoubleSupplier speedSupplier;

    public ManualElevator(DoubleSupplier speedSupplier) {
        this.speedSupplier = speedSupplier;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.setSpeed(speedSupplier.getAsDouble() * 0.25);        
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setSpeed(0);
    }
}
