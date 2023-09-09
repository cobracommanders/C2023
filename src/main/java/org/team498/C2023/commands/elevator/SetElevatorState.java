package org.team498.C2023.commands.elevator;

import org.team498.C2023.RobotState;
import org.team498.C2023.State;
import org.team498.C2023.subsystems.Elevator;


import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/*
 *  Most subsystems will have a Wrapper class that casts the setState() method to a command
 *  These commands typically just run the setState() method, but can be modified to do additional tasks as well
 */
public class SetElevatorState extends CommandBase {

    private State.Elevator state;

    public SetElevatorState() {
        addRequirements(Elevator.getInstance());
    }

    // The initialize() method runs once when the command starts
    @Override
    public void initialize() {
        state = RobotState.getInstance().getState().elevator; 
        Elevator.getInstance().setState(state); // set the state of the elevator using setState()
    }

    public boolean isFinished(){
        return true;//Elevator.getInstance().atSetpoint();
    }
}
