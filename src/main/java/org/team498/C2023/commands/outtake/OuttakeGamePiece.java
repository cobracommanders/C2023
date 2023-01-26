package org.team498.C2023.commands.outtake;

import org.team498.C2023.RobotState.GamePiece;
import org.team498.C2023.subsystems.Outtake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class OuttakeGamePiece extends SequentialCommandGroup {
    public OuttakeGamePiece(GamePiece gamePiece) {
            super(
                Outtake.getInstance().setOuttake(gamePiece == GamePiece.CONE ? Outtake.State.SHOOT_CONE : Outtake.State.SHOOT_CUBE),
                new WaitCommand(1),
                Outtake.getInstance().setOuttake(Outtake.State.IDLE)
            );
    }
}
