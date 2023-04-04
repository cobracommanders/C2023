package org.team498.C2023.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import org.team498.C2023.State;
import org.team498.C2023.commands.robot.ReturnToIdle;
import org.team498.C2023.subsystems.Drivetrain;
import org.team498.C2023.subsystems.elevator.Elevator;
import org.team498.C2023.subsystems.elevatorwrist.ElevatorWrist;
import org.team498.C2023.subsystems.intakerollers.IntakeRoller;
import org.team498.C2023.subsystems.intakewrist.IntakeWrist;
import org.team498.C2023.subsystems.manipulator.Manipulator;
import org.team498.lib.drivers.Blinkin;
import org.team498.lib.drivers.Blinkin.BlinkinColor;

import java.util.function.BooleanSupplier;

import static edu.wpi.first.wpilibj2.command.Commands.*;


public class SystemsCheck {
    public Command getCommand() {
        return sequence(
                test("Elevator Encoder", () -> {}, () -> Elevator.getInstance().checkEncoderConnection(), 1),
                test("ElevatorWrist Encoder", () -> {}, () -> ElevatorWrist.getInstance().checkEncoderConnection(), 1),
                test("IntakeWrist Encoder", () -> {}, () -> IntakeWrist.getInstance().checkEncoderConnection(), 1),

                test("IntakeWrist Motors", () -> IntakeWrist.getInstance().setState(State.IntakeWrist.TEMPORARY_IDLE),
                     () -> IntakeWrist.getInstance().atSetpoint(), 2),
                test("ElevatorWrist Motor", () -> ElevatorWrist.getInstance().setState(State.ElevatorWrist.TRAVEL),
                     () -> ElevatorWrist.getInstance().atSetpoint(), 2),
                test("Elevator Motors", () -> Elevator.getInstance().setState(State.Elevator.TOP_CUBE), () -> Elevator.getInstance().atSetpoint(), 2),
                test("Manipulator Motor", () -> Manipulator.getInstance().setState(State.Manipulator.INTAKE_CUBE),
                     () -> Manipulator.getInstance().getCurrentDraw() > 0.2, 2),
                test("IntakeRoller Motors", () -> IntakeRoller.getInstance().setState(State.IntakeRollers.INTAKE), () -> {
                    var amps = IntakeRoller.getInstance().getCurrentDraws();
                    return amps[0] + amps[1] + amps[2] > 0.6;
                }, 2),
                new ReturnToIdle(),
                test("Drivetrain Motors", () -> Drivetrain.getInstance().drive(1, 0, 0, true), () ->
                             Math.abs(Drivetrain.getInstance().getCurrentSpeeds().vxMetersPerSecond - 1) < 0.05,
                     2),
                runOnce(() -> Drivetrain.getInstance().X())
                       );
    }

    private final Timer timer = new Timer();

    private CommandBase test(String system, Runnable test, BooleanSupplier isWorking, double timeout) {
        return sequence(
                runOnce(() -> {
                    timer.reset();
                    timer.start();
                    Blinkin.getInstance().setColor(BlinkinColor.SOLID_YELLOW); //BlinkinColor.SOLID_YELLOW
                    System.out.println("Starting test on " + system);
                }),
                race(
                        sequence(
                                waitSeconds(timeout),
                                print("Test failed in " + timeout + " seconds"),
                                runOnce(() -> Blinkin.getInstance().setColor(BlinkinColor.SOLID_RED))),
                        sequence(
                                waitUntil(isWorking),
                                print("Test successful"),
                                runOnce(() -> Blinkin.getInstance().setColor(BlinkinColor.SOLID_LIME))),
                        run(test)),
                runOnce(timer::stop),
                waitSeconds(0.5));
    }
}
