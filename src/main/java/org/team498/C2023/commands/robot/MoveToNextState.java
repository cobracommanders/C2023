package org.team498.C2023.commands.robot;

import org.team498.C2023.commands.manipulator.SetManipulatorToNextState;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class MoveToNextState extends SequentialCommandGroup {
    public MoveToNextState() {
        super(new ParallelCommandGroup(
            new CollectFromSS()),
          new WaitCommand(0.5),
          new SetManipulatorToNextState());
    }

}
