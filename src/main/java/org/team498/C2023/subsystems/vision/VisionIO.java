package org.team498.C2023.subsystems.vision;

import java.util.List;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public interface VisionIO {
    class VisionIOInputs implements LoggableInputs, Cloneable {
        public Pose2d estimatedPose = new Pose2d();
        /**{target ID, target ambiguity}, */
        public double[][] targets = new double[][] {};

        @Override
        public void toLog(LogTable table) {
            double[] poseData = new double[3];
            poseData[0] = estimatedPose.getX();
            poseData[1] = estimatedPose.getY();
            poseData[2] = estimatedPose.getRotation().getDegrees();
            table.put("EstimatedPose", poseData);

            table.put("TargetQuantity", targets.length);
            for (int i = 0; i < targets.length; i++) {
                table.put("Target/" + Integer.toString(i), targets[i]);
            }
        }

        @Override
        public void fromLog(LogTable table) {
            double[] poseData = table.getDoubleArray("EstimatedPose", new double[] {});
            estimatedPose = new Pose2d(poseData[0], poseData[1], Rotation2d.fromDegrees(poseData[2]));

            int targetCount = (int) table.getInteger("TargetQuantity", 0);
            targets = new double[targetCount][];
            for (int i = 0; i < targetCount; i++) {
                targets[i] = table.getDoubleArray("Target/" + Integer.toString(i), new double[] {});
            }
        }

        @Override
        public VisionIOInputs clone() {
            VisionIOInputs copy = new VisionIOInputs();
            copy.estimatedPose = new Pose2d(this.estimatedPose.getX(), this.estimatedPose.getY(), Rotation2d.fromDegrees(this.estimatedPose.getRotation().getDegrees()));
            copy.targets = this.targets.clone();
            return copy;
        }

    }

    default void updateInputs(VisionIOInputs inputs) {
    }

}
