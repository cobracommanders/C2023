package org.team498.C2023.commands.robot;

import org.team498.C2023.commands.coneariser.SetConeARiserState;
import org.team498.C2023.commands.elevator.SetElevatorToNextState;
import org.team498.C2023.commands.manipulator.SetManipulatorToNextState;
import org.team498.C2023.commands.wrist.SetWristState;
import org.team498.C2023.commands.wrist.SetWristToNextState;
import org.team498.C2023.subsystems.ConeARiser;
import org.team498.C2023.subsystems.Elevator;
import org.team498.C2023.subsystems.Wrist;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class CollectFromDoubleSS extends SequentialCommandGroup {
    public CollectFromDoubleSS() {
        super(
                new SetWristState(Wrist.State.TRAVEL),
                new SetManipulatorToNextState(),
                new ParallelCommandGroup(
                        new SetElevatorToNextState(),
                        new SequentialCommandGroup(
                                new WaitUntilCommand(() -> Elevator.getInstance().aboveIntakeHeight()),
                                new SetWristToNextState())),
                new SetConeARiserState(ConeARiser.State.IDLE));
    }
}
