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
import org.team498.C2023.commands.auto.JustAutoShot;
import org.team498.C2023.commands.auto.JustScore;
import org.team498.C2023.commands.auto.PreloadAndTaxi;
import org.team498.C2023.commands.auto.TwoCubeEngage;
import org.team498.C2023.commands.auto.TwoCubePickupEngage;
import org.team498.C2023.subsystems.Drivetrain;
import org.team498.C2023.subsystems.Elevator;
import org.team498.C2023.subsystems.ElevatorWrist;
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

    private final List<Auto> autoOptions = List.of(new JustScore(), new CubeEngage(), new PreloadAndTaxi(), new TwoCubeEngage(), new TwoCubePickupEngage(), new JustAutoShot());


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

        field.getObject("Scoring Target").setPose(RobotPositions.getNextScoringNodePosition());

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


        SmartDashboard.putString("Robot State", RobotState.getInstance().getCurrentState().name());
        SmartDashboard.putString("Blinkin Color", blinkin.getColor().name());

        SmartDashboard.putNumber("Interpolation Value", Drivetrain.getInstance().distanceTo(Point.fromPose2d(RobotPositions.getNextScoringNodePosition())));
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
        SmartDashboard.putNumber("anlge error", RotationUtil.toSignedDegrees(Math.abs(drivetrain.getYaw() - drivetrain.calculateDegreesToTarget(RobotPositions.getNextScoringNodePosition()))));
        if ((Math.abs(RotationUtil.toSignedDegrees(Math.abs(drivetrain.getYaw() - drivetrain.calculateDegreesToTarget(RobotPositions.getNextScoringNodePosition())))) < 5) && (drivetrain.distanceTo(Point.fromPose2d(RobotPositions.getClosestScoringPosition())) < Units.inchesToMeters(25))) {
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
        }
    }

    @Override
    public void autonomousInit() {
        Auto auto = autoChooser.getSelected();

        if (auto == null) auto = new JustScore();
    
        if (alliance == Alliance.Blue) {
            Drivetrain.getInstance().setPose(auto.getInitialPose());
        } else {
            Drivetrain.getInstance().setPose(PoseUtil.flip(auto.getInitialPose()));
        }

        Elevator.getInstance().updateInitialPosition(auto.getInitialState() == State.AUTO_SHOT);

        Elevator.getInstance().setState(auto.getInitialState().elevator);
        ElevatorWrist.getInstance().setState(auto.getInitialState().elevatorWrist);



        auto.getCommand().schedule();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }

}