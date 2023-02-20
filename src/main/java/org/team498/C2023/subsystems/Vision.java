package org.team498.C2023.subsystems;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.team498.C2023.FieldPositions;

import java.util.Optional;

public class Vision extends SubsystemBase {
    private final PhotonCamera cam1 = new PhotonCamera("Arducam_OV9281_USB_Camera");
    private PhotonPipelineResult pipelineResult = new PhotonPipelineResult();
    public Pose2d pose = new Pose2d();
    public PhotonPipelineResult lastResult;    

    private Vision() {
        cam1.setPipelineIndex(0);
        cam1.setDriverMode(false);
    }

    @Override
    public void periodic() {
        // pipelineResult = cam1.getLatestResult();
        // if (pipelineResult.hasTargets()) {
        //     lastResult = pipelineResult;
        //     pose = getPose();
        // }
    }

    private Pose2d getPose() {
        Transform3d pose = pipelineResult.getBestTarget().getBestCameraToTarget();
        return new Pose2d(pose.getX(), pose.getY(), /*new Rotation2d(-pose.getRotation().toRotation2d().getRadians())*/ Rotation2d.fromDegrees(Drivetrain.getInstance().getYaw()));
    }

    public int getTargetedTag() {
        if (pipelineResult.hasTargets()) return pipelineResult.getBestTarget().getFiducialId();
        return -1;
    }

    public Optional<Pose2d> getRobotPose() {
        int tag = getTargetedTag();

        if (tag != -1 && tag <= 8) {
            Pose2d pose1 = getPose();
            pose1 = toFieldRelativeSpeeds(new Pose2d(pose.getX(), pose.getY(), Rotation2d.fromDegrees(Drivetrain.getInstance().getYaw())));//.transformBy(new Transform2d(new Translation2d(FieldPositions.aprilTags.get(tag).getX(), FieldPositions.aprilTags.get(tag).getY()), new Rotation2d())));
            Pose2d pose = new Pose2d(pose1.getX() + FieldPositions.aprilTags.get(tag).getX(), pose1.getY() + FieldPositions.aprilTags.get(tag).getY(), pose1.getRotation());

            if (tag > 4) {
                return Optional.of(pose.plus(new Transform2d(new Translation2d(this.pose.getX(), this.pose.getY()), Rotation2d.fromDegrees(180))));
            } else {
                return Optional.of(pose.plus(new Transform2d(new Translation2d(-this.pose.getX(), this.pose.getY()), Rotation2d.fromDegrees(0))));
            }
        }

        return Optional.empty();
    }
    public static Pose2d toFieldRelativeSpeeds(Pose2d pose) {
        return toFieldRelativeSpeeds(pose.getX(), pose.getY(), pose.getRotation().getDegrees());
    }

    public static Pose2d toFieldRelativeSpeeds(double vxMetersPerSecond, double vyMetersPerSecond, double robotAngle1) {
        Rotation2d robotAngle = Rotation2d.fromDegrees(robotAngle1);
        return new Pose2d(vxMetersPerSecond * robotAngle.getCos() - vyMetersPerSecond * robotAngle.getSin(), vxMetersPerSecond * robotAngle.getSin() + vyMetersPerSecond * robotAngle.getCos(), robotAngle);
    }

    public static Pose2d fromFieldRelativeSpeeds(double vxMetersPerSecond, double vyMetersPerSecond, double robotAngle) {
        double angleInRadians = Math.toRadians(robotAngle);
        double vxRobot = vxMetersPerSecond * Math.cos(angleInRadians) + vyMetersPerSecond * Math.sin(angleInRadians);
        double vyRobot = -vxMetersPerSecond * Math.sin(angleInRadians) + vyMetersPerSecond * Math.cos(angleInRadians);
        return new Pose2d(vxRobot, vyRobot, Rotation2d.fromDegrees(robotAngle));
    }


    private static Vision instance;

    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
        }
        return instance;
    }
}
