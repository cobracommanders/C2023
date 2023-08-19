package org.team498.C2023.commands.elevatorWrist;

import java.util.function.DoubleSupplier;

import org.team498.C2023.subsystems.ElevatorWrist;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ManualElevatorWrist extends CommandBase {
    private DoubleSupplier speed;

    public ManualElevatorWrist(DoubleSupplier speed){
        addRequirements(ElevatorWrist.getInstance());
        this.speed = speed;
    }
    @Override
    public void initialize() {
        ElevatorWrist.getInstance().setManual(true);
    }

    @Override
    public void execute() {
        ElevatorWrist.getInstance().setManualSpeed(speed.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        ElevatorWrist.getInstance().setManualSpeed(0);
        ElevatorWrist.getInstance().setManual(false);
    }

}
