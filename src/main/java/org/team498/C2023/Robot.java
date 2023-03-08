package org.team498.C2023;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.team498.C2023.commands.auto.CubeEngage;
import org.team498.C2023.commands.auto.JustScore;
import org.team498.C2023.commands.auto.PreloadAndTaxi;
import org.team498.C2023.subsystems.Drivetrain;
import org.team498.C2023.subsystems.Manipulator;
import org.team498.lib.drivers.Blinkin;
import org.team498.lib.drivers.Gyro;
import org.team498.lib.drivers.Blinkin.Color;


public class Robot extends TimedRobot {
    public static int coordinateFlip = 1;
    public static int rotationOffset = 0;

    public static final Field2d field = new Field2d();
    public static Alliance alliance = Alliance.Invalid;
    public static final Controls controls = new Controls();

    public static boolean cameraEnabled = true;

    private final Drivetrain drivetrain = Drivetrain.getInstance();
    private final Gyro gyro = Gyro.getInstance();
    // private final Photonvision photonvision = Photonvision.getInstance();
    private final Blinkin blinkin = Blinkin.getInstance();
    private final RobotState robotState = RobotState.getInstance();

    private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();


    @Override
    public void robotInit() {
        drivetrain.setPose(new Pose2d(8.29, 4, Rotation2d.fromDegrees(0)));
        controls.driver.setRightStickLastAngle(-gyro.getAngleOffset());

        gyro.setYaw(0);

        SmartDashboard.putData(field);
        FieldPositions.displayAll();

        autoChooser.setDefaultOption("Score", new JustScore());
        autoChooser.addOption("Cube Engage", new CubeEngage());
        autoChooser.addOption("Preload and Taxi", new PreloadAndTaxi());

        controls.configureDefaultCommands();
        controls.configureDriverCommands();
        controls.configureOperatorCommands();

        SmartDashboard.putData(autoChooser);
    }

    @Override
    public void simulationPeriodic() {
        Simulation.getInstance().update();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        //if (cameraEnabled)
        //photonvision.getEstimatedGlobalPose(drivetrain.getPose()).ifPresent(pose -> drivetrain.setOdometry(pose.estimatedPose));


        if (RobotPositions.inCommunity()) {
            if (RobotState.getInstance().inConeMode()) {
                field.getObject("Scoring Targets").setPoses(RobotPositions.getRightScoringPosition(), RobotPositions.getLeftScoringPosition());
            } else {
                field.getObject("Scoring Targets").setPose(RobotPositions.getCenterScoringPosition());
            }
        } else if (RobotPositions.inLoadingZone()) {
            field.getObject("Scoring Targets").setPoses(RobotPositions.getLeftSubstationPosition(),
                                                        RobotPositions.getRightSubstationPosition(),
                                                        RobotPositions.getSingleSubstationPosition()
            );
        } else {
            field.getObject("Scoring Targets").setPose(new Pose2d(-1, -1, new Rotation2d()));
        }



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


        SmartDashboard.putString("Driveteam State", RobotState.getInstance().getNextDriveteamState().name());
        SmartDashboard.putString("Robot State", RobotState.getInstance().getCurrentState().name());
    }

    @Override
    public void disabledPeriodic() {
        // blinkin.setColor(Blinkin.Color.BLUE);

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

    @Override
    public void teleopPeriodic() {
        if (RobotPositions.inSSPickupArea()) {
            blinkin.setColor(Blinkin.Color.LIME);
        } else {
            if (robotState.inConeMode() && Manipulator.getInstance().isStalling()) {
                blinkin.setColor(Color.BLUE);
            } else {
                blinkin.setColor(RobotState.getInstance().inConeMode() ? Blinkin.Color.YELLOW : Blinkin.Color.PURPLE);
            }
        }
    }

    @Override
    public void autonomousInit() {
        // autoChooser.getSelected().schedule();
        Drivetrain.getInstance().setPose(new Pose2d(0, 0, Rotation2d.fromDegrees(Robot.alliance == Alliance.Blue ? 0 : 180)));
        new CubeEngage().schedule();
        // new JustScore().schedule();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }

}