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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.team498.C2023.subsystems.Drivetrain;
import org.team498.lib.auto.Auto;
import org.team498.lib.drivers.Gyro;
import org.team498.lib.util.PoseUtil;
import java.util.List;

public class Robot extends TimedRobot {
    public static int coordinateFlip = 1;
    public static int rotationOffset = 0;

    public static final Field2d field = new Field2d();
    public static Alliance alliance = Alliance.Invalid;
    public static final Controls controls = new Controls();

    private final Drivetrain drivetrain = Drivetrain.getInstance();
    private final Gyro gyro = Gyro.getInstance();

    private final SendableChooser<Auto> autoChooser = new SendableChooser<Auto>();

    private final List<Auto> autoOptions = List.of(
            );

    private Auto autoToRun;
    private Auto defaultAutoCommand = null; //TODO set real auto

    @Override
    public void robotInit() {
        drivetrain.setPose(new Pose2d(8.29, 4, Rotation2d.fromDegrees(0)));
        controls.driver.setRightStickLastAngle(-gyro.getAngleOffset());

        gyro.setYaw(0);

        SmartDashboard.putData(field);

        autoChooser.setDefaultOption(defaultAutoCommand.getName(), defaultAutoCommand);

        autoOptions.forEach(auto -> {
            autoChooser.addOption(auto.getName(), auto);
        });

        controls.configureDefaultCommands();
        controls.configureDriverCommands();
        controls.configureOperatorCommands();

        PathLib.testPath.getClass();

        SmartDashboard.putData(autoChooser);
    }

    @Override
    public void simulationPeriodic() {
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run(); //TODO does this need to happen every loop?

        //TODO: This condition varies depending on Field Symmetry
        // alliance is invalid when the FMS is not connected
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
        
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void autonomousInit() {
        if (autoToRun == null)
            autoToRun = defaultAutoCommand;

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

    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }

}