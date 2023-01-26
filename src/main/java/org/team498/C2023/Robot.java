package org.team498.C2023;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.team498.C2023.RobotState.GamePiece;
import org.team498.C2023.subsystems.Drivetrain;
import org.team498.C2023.subsystems.Vision;
import org.team498.lib.drivers.Gyro;
import org.team498.lib.field.Point;

import java.util.LinkedList;

public class Robot extends TimedRobot {
    public static int rotationDirection;
    public static Field2d field = new Field2d();
    public static Alliance alliance = Alliance.Blue;
    public static RobotContainer robotContainer = new RobotContainer();

    private final Vision vision = Vision.getInstance();
    private final Drivetrain drivetrain = Drivetrain.getInstance();
    private final Gyro gyro = Gyro.getInstance();

    @Override
    public void robotInit() {
        if (isReal()) {
            rotationDirection = -1;
        } else {
            rotationDirection = 1;
        }
        drivetrain.setPose(new Pose2d(8.29, 4, Rotation2d.fromDegrees(0)));
        robotContainer.driver.setRightStickLastAngle(-gyro.getAngleOffset());

        SmartDashboard.putData(field);
        FieldPositions.displayAll();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        SmartDashboard.putNumber("Tag ID", vision.getTargetedTag());

        vision.getRobotPose().ifPresent(drivetrain::setPose);

        if (RobotPositions.inCommunity()) {
            if (RobotState.getInstance().getCurrentGamePiece() == GamePiece.CONE) {
                field.getObject("Scoring Targets").setPoses(RobotPositions.getRightPosition(),
                        RobotPositions.getLeftPosition());
            } else {
                field.getObject("Scoring Targets").setPose(RobotPositions.getCenterPosition());
            }
        } else {
            field.getObject("Scoring Targets").setPose(new Pose2d(-1, -1, new Rotation2d()));
        }

        SmartDashboard.putString("Current Game Piece", RobotState.getInstance().getCurrentGamePiece().name());
        SmartDashboard.putBoolean("In Community", RobotPositions.inCommunity());
    }

    @Override
    public void disabledPeriodic() {
        alliance = DriverStation.getAlliance();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }

}