package org.team498.C2023.commands.drivetrain;

import org.team498.C2023.Robot;
import org.team498.C2023.subsystems.Drivetrain;
import org.team498.lib.wpilib.ChassisSpeeds;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TimedDrive extends CommandBase {
    private ChassisSpeeds speeds;
    private final double time;
    private final Timer timer = new Timer();
    private final Drivetrain drivetrain = Drivetrain.getInstance();

    public TimedDrive(ChassisSpeeds speeds, double time) {
        this.speeds = speeds;
        this.time = time;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        if (Robot.alliance == Alliance.Red) {
            speeds = new ChassisSpeeds(-speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, -speeds.omegaRadiansPerSecond);
        }
    }

    @Override
    public void execute() {
        drivetrain.drive(speeds, new Translation2d());
    }

    @Override
    public boolean isFinished() {
        return timer.get() > time;
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        drivetrain.stop();
    }

}
