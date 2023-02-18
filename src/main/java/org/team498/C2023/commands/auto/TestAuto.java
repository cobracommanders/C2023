package org.team498.C2023.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team498.C2023.PathLib;
import org.team498.C2023.commands.drivetrain.FollowTrajectory;
import org.team498.C2023.commands.manipulator.SetManipulatorState;
import org.team498.C2023.subsystems.Drivetrain;
import org.team498.C2023.subsystems.Manipulator;

public class TestAuto extends SequentialCommandGroup {
    private final Drivetrain drivetrain = Drivetrain.getInstance();

    public TestAuto() {
        addRequirements(drivetrain);
        addCommands(new InstantCommand(() -> drivetrain.setPose(PathLib.unnamed.getInitialPose())),
                    new SetManipulatorState(Manipulator.State.AUTO_SHOT),
                    new FollowTrajectory(PathLib.unnamed)
        );
    }
}
