package org.team498.C2023.commands.flywheel;

import org.team498.C2023.State;
import org.team498.C2023.subsystems.Flywheel;

import edu.wpi.first.wpilibj2.command.CommandBase;

/*
 *  Most subsystems will have a Wrapper class that casts the setState() method to a command
 *  These commands typically just run the setState() method, but can be modified to do additional tasks as well
 */
public class SetFlywheelState extends CommandBase {

    private final State.Flywheel state;

    public SetFlywheelState(State.Flywheel state) {
        this.state = state;
    }

    // The initialize() method runs once when the command starts
    @Override
    public void initialize() {
        Flywheel.getInstance().setState(state); // set the state of the flywheel using setState()
    }
}
