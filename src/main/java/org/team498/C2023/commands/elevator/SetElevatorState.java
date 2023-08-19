package org.team498.C2023.commands.elevator;

import org.team498.C2023.State;
import org.team498.C2023.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/*
 *  Most subsystems will have a Wrapper class that casts the setState() method to a command
 *  These commands typically just run the setState() method, but can be modified to do additional tasks as well
 */
public class SetElevatorState extends CommandBase {

    private final State.Elevator state;

    public SetElevatorState(State.Elevator state) {
        addRequirements(Elevator.getInstance());
        this.state = state;
    }

    // The initialize() method runs once when the command starts
    @Override
    public void initialize() {
        Elevator.getInstance().setState(state); // set the state of the elevator using setState()
    }

    public boolean isFinished(){
        return Elevator.getInstance().atSetpoint();
    }
}
