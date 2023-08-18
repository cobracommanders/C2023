package org.team498.C2023.commands.elevator;

import java.util.function.DoubleSupplier;

import org.team498.C2023.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ManualElevator extends CommandBase {
    private DoubleSupplier speed;

    public ManualElevator(DoubleSupplier speed){
        addRequirements(Elevator.getInstance());
        this.speed = speed;
    }
    @Override
    public void initialize() {
        Elevator.getInstance().setManual(true);
    }

    @Override
    public void execute() {
        Elevator.getInstance().setManualSpeed(-speed.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        Elevator.getInstance().setManualSpeed(0);
        Elevator.getInstance().setManual(false);
    }

}
