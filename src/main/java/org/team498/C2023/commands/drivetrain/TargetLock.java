package org.team498.C2023.commands.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team498.C2023.Constants;
import org.team498.C2023.RobotState;
import org.team498.C2023.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;

public class TargetLock extends CommandBase {
    private final Drivetrain drivetrain;
    private final RobotState robotState;
    private final DoubleSupplier velocity;
    private Double t;

    public TargetLock(RobotState robotState, DoubleSupplier velocity) {
        this.drivetrain = Drivetrain.getInstance();
        this.robotState = robotState;
        this.velocity = velocity;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        t = 0.0;
    }

    @Override
    public void execute() {
        Transform2d state = robotState.getRobotToTarget();
        double radius = Math.hypot(state.getX(), state.getY());
        double tVelocity = velocity.getAsDouble();

        t += tVelocity * Constants.DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND / 50; //Max velocity per 20ms
        //TODO Are these supposed to be the same?
        double x = Math.sqrt(Math.pow(t, 2) - Math.pow(radius, 2));
        double y = Math.sqrt(Math.pow(t, 2) - Math.pow(radius, 2));

        double heading = Math.atan2(x, y);
        double initialHeading = Math.atan2(state.getX(), state.getY()); //TODO: currently assumes that it is facing the target

        drivetrain.setPositionGoal(new Pose2d(x, y, Rotation2d.fromDegrees(Math.toDegrees(heading))));

        double xOutput = drivetrain.calculateXController(-state.getX());
        double yOutput = drivetrain.calculateYController(-state.getY());
        double headingOutput = drivetrain.calculateSnapController(drivetrain.getYaw() - initialHeading);

        drivetrain.drive(xOutput, yOutput, headingOutput, true);


    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
        t = 0.0;
    }
}
