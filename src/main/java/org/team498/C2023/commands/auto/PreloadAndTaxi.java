package org.team498.C2023.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team498.C2023.PathLib;
import org.team498.C2023.commands.elevator.SetElevatorState;
import org.team498.C2023.commands.wrist.SetWristState;
import org.team498.C2023.subsystems.Drivetrain;
import org.team498.C2023.subsystems.Elevator;
import org.team498.C2023.subsystems.Wrist;

public class PreloadAndTaxi extends SequentialCommandGroup {
    public PreloadAndTaxi() {
        super(new InstantCommand(() -> Drivetrain.getInstance().setPose(PathLib.SingleCube.Path1.getInitialHolonomicPose())),
              new ParallelCommandGroup(
                      new SetElevatorState(Elevator.State.AUTO_SHOT),
                      new SetWristState(Wrist.State.AUTO_SHOT))
        );
    }
}
