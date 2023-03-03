package org.team498.C2023.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team498.C2023.PathLib;
import org.team498.C2023.RobotState;
import org.team498.C2023.RobotState.GamePiece;
import org.team498.C2023.RobotState.Height;
import org.team498.C2023.commands.drivetrain.LockWheels;
import org.team498.C2023.commands.drivetrain.PathPlannerFollower;
import org.team498.C2023.commands.elevator.SetElevatorState;
import org.team498.C2023.commands.robot.Score;
import org.team498.C2023.commands.wrist.SetWristState;
import org.team498.C2023.subsystems.Drivetrain;
import org.team498.C2023.subsystems.Elevator;
import org.team498.C2023.subsystems.Wrist;

public class PreloadAndTaxi extends SequentialCommandGroup {
    public PreloadAndTaxi() {
        super(
            new InstantCommand(() -> RobotState.getInstance().setNextHeight(Height.TOP)),
                new InstantCommand(() -> RobotState.getInstance().setCurrentGameMode(GamePiece.CUBE)),
                new Score(),
                new PathPlannerFollower(PathLib.singleCubeTaxi));
    }
}
