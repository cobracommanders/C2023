package org.team498.C2023.commands.prototype;

import java.util.function.DoubleSupplier;

import org.team498.C2023.subsystems.Prototype;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class PrototypeTest extends CommandBase {
    private final Prototype prototype = Prototype.getInstance();
    private final DoubleSupplier speed;

    public PrototypeTest(DoubleSupplier speed) {
        this.speed = speed;

        addRequirements(prototype);
    }

    @Override
    public void execute() {
        prototype.setSpeed(speed.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        prototype.setSpeed(0);
    }
}
