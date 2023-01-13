package org.team498.C2023;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.team498.C2023.subsystems.Drivetrain;
import org.team498.lib.drivers.Gyro;
import org.team498.lib.field.Ellipse;
import org.team498.lib.field.Line;
import org.team498.lib.field.Point;
import org.team498.lib.field.Rectangle;
import org.team498.lib.field.Region;

import java.util.function.DoubleSupplier;

public class Robot extends TimedRobot {
    public static Field2d field = new Field2d();

    DoubleSupplier x;

    Region region;

    @Override
    public void robotInit() {
        new RobotContainer();

        var R = new Rectangle(4, 3, 3, 3);
        var E = new Ellipse(new Rectangle(1, 2, 3, 3));

        //R.displayOnDashboard("R");
        //E.displayOnDashboard("E");

        region = new Region(R, E);
        //region.displayOnDashboard("combo");

        FieldPositions.displayAll();

        // Calibrate the gyro sensor when the robot is powered on
        Gyro.getInstance().calibrate();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        SmartDashboard.putData(field);
        SmartDashboard.putBoolean("In Zone", region.contains(Point.fromPose2d(Drivetrain.getInstance().getPose())));

        SmartDashboard.putNumber("xbox", RobotContainer.xbox.rightAngle());
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {
        // SmartDashboard.putBoolean("In Region",
        // FieldPositions.LOADING_ZONE.containsPosition(Drivetrain.getInstance().getPose()));
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }
}
