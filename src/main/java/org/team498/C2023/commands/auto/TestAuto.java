package org.team498.C2023.commands.auto;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team498.C2023.PathLib;
import org.team498.C2023.commands.drivetrain.FollowTrajectory;
import org.team498.C2023.subsystems.Drivetrain;

public class TestAuto extends SequentialCommandGroup {
    private final Drivetrain drivetrain = Drivetrain.getInstance();
    public TestAuto() {
        addRequirements(drivetrain);
        addCommands(new InstantCommand(() -> drivetrain.setInitialPose(PathLib.unnamed.getInitialPose())),
                                            new FollowTrajectory(PathLib.unnamed));
    }
}
