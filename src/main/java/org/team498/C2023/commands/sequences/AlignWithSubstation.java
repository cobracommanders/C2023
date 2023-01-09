package org.team498.C2023.commands.sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team498.C2023.FieldPositions;
import org.team498.C2023.commands.drivetrain.DriveToPosition;

public class AlignWithSubstation extends SequentialCommandGroup {
    public AlignWithSubstation() {
        super(new DriveToPosition(FieldPositions.SUBSTATION_RIGHT_BLUE));
    }
}
