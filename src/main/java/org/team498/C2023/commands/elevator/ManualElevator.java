package org.team498.C2023.commands.elevator;

import java.util.function.DoubleSupplier;

import org.team498.C2023.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ManualElevator extends CommandBase {
    private final Elevator elevator = Elevator.getInstance();
    private final DoubleSupplier speedSupplier;

    public ManualElevator(DoubleSupplier speedSupplier) {
        this.speedSupplier = speedSupplier;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setControlMode(Elevator.ControlMode.MANUAL);
    }

    @Override
    public void execute() {
        elevator.setSpeed(speedSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setSpeed(0);
    }
}
