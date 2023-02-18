package org.team498.C2023.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.team498.C2023.PathLib;
import org.team498.C2023.commands.drivetrain.FollowTrajectory;
import org.team498.C2023.commands.elevator.SetElevatorState;
import org.team498.C2023.commands.manipulator.SetManipulatorState;
import org.team498.C2023.commands.wrist.SetWristState;
import org.team498.C2023.subsystems.Drivetrain;
import org.team498.C2023.subsystems.Elevator;
import org.team498.C2023.subsystems.Manipulator;
import org.team498.C2023.subsystems.Wrist;

public class SingleCube extends SequentialCommandGroup {
    public SingleCube() {
        super(new InstantCommand(() -> Drivetrain.getInstance().setPose(PathLib.SingleCube.Path1.getInitialPose())),
                    new SetManipulatorState(Manipulator.State.AUTO_SHOT),
                    new SetWristState(Wrist.State.AUTO_SHOT),
                    new SetElevatorState(Elevator.State.AUTO_SHOT),
                    new ParallelCommandGroup(
                            new FollowTrajectory(PathLib.SingleCube.Path1),
                            new SequentialCommandGroup(
                                    new WaitCommand(0.25),
                                    new SetManipulatorState(Manipulator.State.IDLE),
                                    new SetWristState(Wrist.State.IDLE),
                                    new SetElevatorState(Elevator.State.IDLE)
                            )
                    )
        );
    }
}
