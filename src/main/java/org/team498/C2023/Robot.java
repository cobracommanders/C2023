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

import org.team498.C2023.subsystems.Drivetrain;
import org.team498.C2023.subsystems.Manipulator;
import org.team498.C2023.subsystems.Photonvision;
import org.team498.C2023.subsystems.Vision;
import org.team498.lib.drivers.Gyro;

public class Robot extends TimedRobot {
    public static int rotationFlip = -1;
    public static int coordinateFlip = 1;
    public static int rotationOffset = 0;

    public static Field2d field = new Field2d();
    public static Alliance alliance = Alliance.Invalid;
    public static RobotContainer robotContainer = new RobotContainer();

    private final Vision vision = Vision.getInstance();
    private final Drivetrain drivetrain = Drivetrain.getInstance();
    private final Gyro gyro = Gyro.getInstance();
    private final Photonvision photonvision = Photonvision.getInstance();

    @Override
    public void robotInit() {
        if (isReal()) {
            rotationFlip = 1;
        } else {
            rotationFlip = 1;
        }
        drivetrain.setPose(new Pose2d(8.29, 4, Rotation2d.fromDegrees(0)));
        robotContainer.driver.setRightStickLastAngle(-gyro.getAngleOffset());

        gyro.setYaw(0);

        SmartDashboard.putData(field);
        FieldPositions.displayAll();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        SmartDashboard.putNumber("Targeted Tag ID", vision.getTargetedTag());

        photonvision.getEstimatedGlobalPose(drivetrain.getPose()).ifPresent(pose -> {
            drivetrain.setOdometry(pose.estimatedPose);
        });

        if (RobotPositions.inCommunity()) {
            if (RobotState.getInstance().inConeMode()) {
                field.getObject("Scoring Targets").setPoses(RobotPositions.getRightScoringPosition(), RobotPositions.getLeftScoringPosition());
            } else {
                field.getObject("Scoring Targets").setPose(RobotPositions.getCenterScoringPosition());
            }
        }  else if (RobotPositions.inLoadingZone()) {
            field.getObject("Scoring Targets").setPoses(RobotPositions.getLeftSubstationPosition(), RobotPositions.getRightSubstationPosition(), RobotPositions.getSingleSubstationPosition());
        }
        else {
            field.getObject("Scoring Targets").setPose(new Pose2d(-1, -1, new Rotation2d()));
        }

        SmartDashboard.putBoolean("Manipulator Stalling", Manipulator.getInstance().isStalling());


        //TODO: Check if alliance is actually invalid when the FMS is not connected
        if (alliance == Alliance.Invalid) {
            alliance = DriverStation.getAlliance();
            // This reverses the coordinates/direction of the drive commands on the red alliance
            coordinateFlip = alliance == Alliance.Blue
                             ? 1
                             : -1;
            // Add 180 degrees to all teleop rotation setpoints while on the red alliance
            rotationOffset = alliance == Alliance.Blue
                             ? 0
                             : 180;
        }
    }

    @Override
    public void disabledPeriodic() {
        alliance = DriverStation.getAlliance();
        // This reverses the coordinates/direction of the drive commands on the red
        // alliance
        coordinateFlip = alliance == Alliance.Blue
                ? 1
                : -1;
        // Add 180 degrees to all teleop rotation setpoints while on the red alliance
        rotationOffset = alliance == Alliance.Blue
                ? 0
                : 180;
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }

}