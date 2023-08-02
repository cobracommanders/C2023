package org.team498.lib.auto;

import org.team498.lib.auto.Auto.Action;
import org.team498.lib.auto.Auto.Position;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoBuilder {
    public static Auto[] autos = {};
    public static SendableChooser<Position> startPosition = new SendableChooser<>();
    public static SendableChooser<Action> firstAction = new SendableChooser<>();
    public static SendableChooser<Action> secondAction = new SendableChooser<>();
    public static SendableChooser<Action> thirdAction = new SendableChooser<>();
    public static SendableChooser<Position> endPosition = new SendableChooser<>();

    public static void displaySelectors() {
        SmartDashboard.putData(startPosition);
        SmartDashboard.putData(firstAction);
        SmartDashboard.putData(secondAction);
        SmartDashboard.putData(thirdAction);
        SmartDashboard.putData(endPosition);
    }

    public static void loadData(Auto... autos) {

    }

    public static Command configureCommand() {
        Command firstCommand = findCommand(startPosition.getSelected(), null, firstAction.getSelected());
        Command secondCommand = findCommand(null, null, secondAction.getSelected());
        Command thirdCommand = findCommand(null, endPosition.getSelected(), thirdAction.getSelected());
        return new SequentialCommandGroup(
            firstCommand, secondCommand, thirdCommand
        );
    }
    public static Command findCommand(Position start, Position end, Action action) {
        for (Auto auto : autos) {
            if (auto.compare(start, end, action))
            return (Command) auto;
        }
        return Commands.none();
    }
}
