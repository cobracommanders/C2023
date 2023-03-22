package org.team498.C2023;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.team498.C2023.commands.auto.*;
import org.team498.C2023.subsystems.Drivetrain;
import org.team498.C2023.subsystems.Manipulator;
import org.team498.C2023.subsystems.Photonvision;
import org.team498.lib.auto.Auto;
import org.team498.lib.drivers.Blinkin;
import org.team498.lib.drivers.Blinkin.Color;
import org.team498.lib.drivers.Gyro;
import org.team498.lib.field.Point;
import org.team498.lib.util.PoseUtil;
import org.team498.lib.util.RotationUtil;

import java.util.List;


public class Robot extends LoggedRobot {
    public static final double DEFAULT_PERIOD = 0.02;
    public static int coordinateFlip = 1;
    public static int rotationOffset = 0;

    public static final Field2d field = new Field2d();
    public static Alliance alliance = Alliance.Invalid;
    public static final Controls controls = new Controls();

    private final Drivetrain drivetrain = Drivetrain.getInstance();
    private final Gyro gyro = Gyro.getInstance();
    private final Photonvision photonvision = Photonvision.getInstance();
    private final Blinkin blinkin = Blinkin.getInstance();
    private final RobotState robotState = RobotState.getInstance();

    private final SendableChooser<Auto> autoChooser = new SendableChooser<Auto>();
    private Auto autoToRun;

    private final List<Auto> autoOptions = List.of(
            new JustScore(),
            new CubeEngage(),
            new LeftConeTaxi(),
            new LeftCubeTaxi(),
            new RightConeTaxi(),
            new RightCubeTaxi(),
            new TwoPlusOneBump(),
            new HighMidCubeEngage(),
            new HighMidCubeEngageBump(),
            new HighMidCube(),
            new HighMidCubeBump(),
            new HighHighCubeEngage(),
            new HighHighCubeEngageBump(),
            new HighHighCube(),
            new HighHighCubeBump());


    @Override
    public void robotInit() {
        Logger.getInstance().recordMetadata("ProjectName", "C2023"); // Set a metadata value

        if (isReal()) {
            Logger.getInstance().addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to a USB stick
            Logger.getInstance().addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
            new PowerDistribution(1, PowerDistribution.ModuleType.kRev); // Enables power distribution logging //TODO Check CAN ID
        } else {
            switch (Constants.mode) {
                case SIM -> {
                    Logger.getInstance().addDataReceiver(new WPILOGWriter("/"));
                    Logger.getInstance().addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
                }
                case REPLAY -> {
                    setUseTiming(true); // Set to false to run as fast as possible when replaying logs
                    String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
                    Logger.getInstance().setReplaySource(new WPILOGReader(logPath)); // Read replay log
                    Logger.getInstance().addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
                }
                default -> {}
            }

        }

        Logger.getInstance().start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

        gyro.setYaw(0);

        SmartDashboard.putData(field);
        FieldPositions.displayAll();

        autoChooser.setDefaultOption("Score", new JustScore());

        autoOptions.forEach(auto -> {
            autoChooser.addOption(auto.getName(), auto);
        });

        controls.configureDefaultCommands();
        controls.configureDriverCommands();
        controls.configureOperatorCommands();

        SmartDashboard.putData(autoChooser);

        PathLib.eighthNodeToChargeStation.getClass();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        photonvision.getEstimatedGlobalPose().ifPresent(pose -> drivetrain.setPose(PoseUtil.toPose2d(pose.estimatedPose)));

        field.getObject("Scoring Target").setPose(RobotPosition.getNextScoringNodePosition());

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


        autoToRun = autoChooser.getSelected();
    }

    @Override
    public void teleopPeriodic() {
        if ((Math.abs(RotationUtil.toSignedDegrees(Math.abs(drivetrain.getYaw()
                - RobotPosition.calculateDegreesToTarget(RobotPosition.getNextScoringNodePosition())))) < 3.5)
                && (RobotPosition.distanceTo(Point.fromPose2d(RobotPosition.getClosestScoringPosition())) < Units
                        .inchesToMeters(25))) {
            blinkin.setColor(Blinkin.Color.LIME);
            controls.driver.rumble(0.5);
        } else if (robotState.inShootDriveMode() && RobotPosition.inCommunity()) {
            blinkin.setColor(Blinkin.Color.RED);
        } else {
            if (robotState.inConeMode() && Manipulator.getInstance().isStalling()) {
                blinkin.setColor(Color.BLUE);
            } else {
                blinkin.setColor(RobotState.getInstance().inConeMode()
                        ? Blinkin.Color.YELLOW
                        : Blinkin.Color.PURPLE);
            }
            controls.driver.rumble(0);
        }
    }

    
    @Override
    public void teleopExit() {
        controls.driver.rumble(0);
    }

    @Override
    public void autonomousInit() {
        if (autoToRun == null)
            autoToRun = new JustScore();

        if (alliance == Alliance.Blue) {
            Drivetrain.getInstance().setPose(autoToRun.getInitialPose());
        } else {
            Drivetrain.getInstance().setPose(PoseUtil.flip(autoToRun.getInitialPose()));
        }

        autoToRun.getCommand().schedule();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    
    @Override
    public void simulationPeriodic() {
        // Simulation.getInstance().update();
    }

    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }

}