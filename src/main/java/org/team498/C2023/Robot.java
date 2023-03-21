package org.team498.C2023;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.team498.C2023.commands.auto.CubeEngage;
import org.team498.C2023.commands.auto.HighHighCube;
import org.team498.C2023.commands.auto.HighHighCubeBump;
import org.team498.C2023.commands.auto.HighHighCubeEngage;
import org.team498.C2023.commands.auto.HighHighCubeEngageBump;
import org.team498.C2023.commands.auto.JustScore;
import org.team498.C2023.commands.auto.LeftConeTaxi;
import org.team498.C2023.commands.auto.LeftCubeTaxi;
import org.team498.C2023.commands.auto.RightConeTaxi;
import org.team498.C2023.commands.auto.RightCubeTaxi;
import org.team498.C2023.commands.auto.HighMidCubeBump;
import org.team498.C2023.commands.auto.HighMidCubeEngage;
import org.team498.C2023.commands.auto.HighMidCubeEngageBump;
import org.team498.C2023.commands.auto.HighMidCube;
import org.team498.C2023.commands.auto.TwoPlusOneBump;
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

public class Robot extends TimedRobot {
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

    private Auto autoToRun;

    @Override
    public void robotInit() {
        drivetrain.setPose(new Pose2d(8.29, 4, Rotation2d.fromDegrees(0)));
        controls.driver.setRightStickLastAngle(-gyro.getAngleOffset());

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

        PathLib.eighthNodeToChargeStation.getClass();

        SmartDashboard.putData(autoChooser);
    }

    @Override
    public void simulationPeriodic() {
        Simulation.getInstance().update();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        photonvision.getEstimatedGlobalPose().ifPresent(pose -> drivetrain.setOdometry(pose.estimatedPose));

        // field.getObject("Scoring
        // Target").setPose(RobotPositions.getNextScoringNodePosition());

        // TODO: Check if alliance is actually invalid when the FMS is not connected
        if (alliance == Alliance.Invalid) {
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

        SmartDashboard.putString("Robot State", RobotState.getInstance().getCurrentState().name());
        SmartDashboard.putString("Blinkin Color", blinkin.getColor().name());

        SmartDashboard.putNumber("Interpolation Value",
                Drivetrain.getInstance().distanceTo(Point.fromPose2d(RobotPositions.getNextScoringNodePosition())));
    }

    @Override
    public void disabledPeriodic() {
        // blinkin.setColor(Blinkin.Color.BLUE);

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

        autoToRun = autoChooser.getSelected();

    }

    @Override
    public void teleopPeriodic() {
        if ((Math.abs(RotationUtil.toSignedDegrees(Math.abs(drivetrain.getYaw()
                - drivetrain.calculateDegreesToTarget(RobotPositions.getNextScoringNodePosition())))) < 3.5)
                && (drivetrain.distanceTo(Point.fromPose2d(RobotPositions.getClosestScoringPosition())) < Units
                        .inchesToMeters(25))) {
            blinkin.setColor(Blinkin.Color.LIME);
            controls.driver.rumble(0.5);
        } else if (robotState.inShootDriveMode() && RobotPositions.inCommunity()) {
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

        // Elevator.getInstance().updateInitialPosition(auto.getInitialState() ==
        // State.AUTO_SHOT);

        // Elevator.getInstance().setState(autoToRun.getInitialState().elevator);
        // ElevatorWrist.getInstance().setState(autoToRun.getInitialState().elevatorWrist);
        // IntakeWrist.getInstance().setState(autoToRun.getInitialState().intakeWrist);

        autoToRun.getCommand().schedule();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }

}