package org.team498.C2023.commands.robot;

import org.team498.C2023.Robot;
import org.team498.C2023.commands.drivetrain.OffenseDrive;
import org.team498.C2023.subsystems.Elevator;
import org.team498.C2023.subsystems.Manipulator;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class CollectGamePiece extends SequentialCommandGroup {

    public CollectGamePiece() {
        Elevator elevator = Elevator.getInstance();
        Manipulator manipulator = Manipulator.getInstance();
        addCommands(
                new ParallelRaceGroup(
                        new OffenseDrive(() -> 0, () -> 0, () -> Robot.rotationOffset, () -> false),
                        new SequentialCommandGroup(
                                elevator.setPosition(Elevator.Position.HIGH),
                                new WaitCommand(0.5),
                                manipulator.intake(),
                                new WaitCommand(0.25),
                                elevator.setPosition(Elevator.Position.LOW),
                                manipulator.stop()))

        );
    }
}
