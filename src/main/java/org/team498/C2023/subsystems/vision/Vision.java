package org.team498.C2023.subsystems.vision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;
import org.team498.C2023.Constants;
import org.team498.C2023.FieldPositions;
import org.team498.C2023.subsystems.vision.VisionIO.VisionIOInputs;
import org.team498.lib.util.PoseUtil;

import java.util.Optional;

public class Vision extends SubsystemBase {
    private final VisionIO vision;
    private final VisionIOInputs inputs = new VisionIOInputs();

    private Vision() {
        vision = switch (Constants.mode) {
            case REAL, REPLAY ->
                new VisionIOSingleCamera();
            case SIM -> new VisionIO() {};
        };
    }

    @Override
    public void periodic() {
        vision.updateInputs(inputs);
        Logger.getInstance().processInputs("Vision", inputs);


        var tagPoses = new Pose2d[inputs.targets.length];

        for (int i = 0; i < inputs.targets.length; i++) {
            double[] tag = inputs.targets[i];
            tagPoses[i] = PoseUtil.toPose2d(FieldPositions.aprilTags.get((int) tag[0]));
        }

        Logger.getInstance().recordOutput("Vision/TrackedTargets", tagPoses);
    }

    public Optional<Pose2d> getEstimatedPose() {
        if (inputs.estimatedPose.equals(new Pose2d())) {
            return Optional.empty();
        }
        return Optional.of(inputs.estimatedPose);
    }


    private static Vision instance;

    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
        }
        return instance;
    }
}