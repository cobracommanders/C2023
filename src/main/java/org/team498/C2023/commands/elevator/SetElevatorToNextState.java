package org.team498.C2023.commands.elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team498.C2023.subsystems.Elevator;

public class SetElevatorToNextState extends CommandBase {
    private final Elevator elevator = Elevator.getInstance();

    private int count = 0;
    public SetElevatorToNextState() {
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setToNextState();
        count = 0;
    }


    @Override
    public void execute() {
        SmartDashboard.putNumber("count", ++count);
    }

    @Override
    public boolean isFinished() {
        return elevator.atSetpoint();
    }
}
