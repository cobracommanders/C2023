package org.team498.C2023.commands.robot;

import java.util.function.DoubleSupplier;

import org.team498.C2023.RobotState;
import org.team498.C2023.commands.drivetrain.TargetDrive;
import org.team498.C2023.subsystems.Photonvision;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ShootDrive extends ParallelDeadlineGroup {
    public ShootDrive(DoubleSupplier xTranslation, DoubleSupplier yTranslation) {
        super(
            //this might have to change to allow for interpolated values to continue updating
            new MoveToNextState(),
            new TargetDrive(xTranslation, yTranslation, Photonvision.getInstance().nearestTagPose())
        );
    }
    @Override
    public SequentialCommandGroup beforeStarting(Command before) {
        return new SequentialCommandGroup(new InstantCommand(()-> RobotState.getInstance().setNextHeight(RobotState.Height.INTERPOLATE)));
    }

}
