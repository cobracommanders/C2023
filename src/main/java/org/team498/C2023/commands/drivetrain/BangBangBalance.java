package org.team498.C2023.commands.drivetrain;

import org.team498.C2023.subsystems.Drivetrain;
import org.team498.lib.drivers.Gyro;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class BangBangBalance extends CommandBase {
    private Gyro gyro = Gyro.getInstance();
    private Drivetrain drivetrain = Drivetrain.getInstance();
    private boolean isTipped;
    private double deadzone;
    private double speed;
    public BangBangBalance() {
        deadzone = 8;
        speed = 2;
    }
    @Override
    public void initialize() {
        isTipped = false;
    }
    @Override
    public void execute() {
        double angle = gyro.getPitch();
        if (Math.abs(angle) > deadzone || !isTipped) {
            isTipped = true;
            drivetrain.drive(Math.copySign(speed, angle), 0, 0, true);
        } else {
            drivetrain.drive(0, 0, 0, true);
        }
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}
