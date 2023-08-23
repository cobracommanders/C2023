/*
 * MIT License
 *
 * Copyright (c) 2022 PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

 package org.team498.C2023.subsystems;

 import edu.wpi.first.apriltag.AprilTagFieldLayout;
 import edu.wpi.first.apriltag.AprilTagFields;
 import edu.wpi.first.math.geometry.*;
 import edu.wpi.first.math.util.Units;
 import edu.wpi.first.wpilibj.DriverStation;
 import org.photonvision.EstimatedRobotPose;
 import org.photonvision.PhotonCamera;
 import org.photonvision.PhotonPoseEstimator;
 import org.photonvision.PhotonPoseEstimator.PoseStrategy;
 import org.photonvision.targeting.PhotonPipelineResult;
 import org.photonvision.targeting.PhotonTrackedTarget;
 import org.team498.C2023.FieldPositions;
 
 import java.io.IOException;
 import java.util.List;
 import java.util.Optional;
 import java.util.function.Supplier;
 
 public class Photonvision {
     private PhotonPoseEstimator photonPoseEstimator;
     private final PhotonCamera photonCamera;
 
     private final double acceptedTagRange = 3.75;
 
     private Photonvision() {
         photonCamera = new PhotonCamera("Arducam_OV9281_USB_Camera");
 
         try {
             // Attempt to load the AprilTagFieldLayout that will tell us where the tags are on the field.
             AprilTagFieldLayout fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
             // Create pose estimator
             photonPoseEstimator = new PhotonPoseEstimator(fieldLayout,
                                                           PoseStrategy.MULTI_TAG_PNP,
                                                           photonCamera,
                                                           new Transform3d(new Translation3d(Units.inchesToMeters(2.5),
                                                                                             -Units.inchesToMeters(5.6875),
                                                                                             Units.inchesToMeters(22)
                                                           ), new Rotation3d())
             );
             photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
         } catch (IOException e) {
             // The AprilTagFieldLayout failed to load. We won't be able to estimate poses if we don't know
             // where the tags are.
             DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
             photonPoseEstimator = null;
         }
     }
 
     /**
      * @return an EstimatedRobotPose with an estimated pose, the timestamp, and targets used to create the estimate
      */
     public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
         if (photonPoseEstimator == null) {
             // The field layout failed to load, so we cannot estimate poses.
             return Optional.empty();
         }
 
         PhotonPipelineResult result = photonCamera.getLatestResult();
 
         PhotonTrackedTarget target = null;
 
         if (result.hasTargets()) {
             target = result.getBestTarget();
         } else {
             return Optional.empty();
         }
 
         // Don't return a new position if the closest target is further than 3.75 meters away
         if (Math.abs(target.getBestCameraToTarget().getTranslation().getNorm()) >= acceptedTagRange) {
             return Optional.empty();
         }
 
         return photonPoseEstimator.update(result);
     }
 
     public double distanceToClosestTag() {
         PhotonPipelineResult result = photonCamera.getLatestResult();
         List<PhotonTrackedTarget> tags = result.targets;
         double minDistance = 0;
         for (PhotonTrackedTarget tag : tags) {
             if (Math.abs(tag.getBestCameraToTarget().getTranslation().getNorm()) <= minDistance) {
                 minDistance = tag.getBestCameraToTarget().getTranslation().getNorm();
             }
         }
         return minDistance;
     }
 
     public Supplier<Pose3d> nearestTagPose() {
         PhotonPipelineResult result = photonCamera.getLatestResult();
         List<PhotonTrackedTarget> tags = result.targets;
         double minDistance = 0;
         int closestTag = 0;
         for (PhotonTrackedTarget tag : tags) {
             if (Math.abs(tag.getBestCameraToTarget().getTranslation().getNorm()) <= minDistance) {
                 minDistance = tag.getBestCameraToTarget().getTranslation().getNorm();
                 closestTag = tag.getFiducialId();
             }
         }
         final int tag = closestTag;
         return () -> FieldPositions.aprilTags.get(tag);
     }
 
 
     private static Photonvision instance;
 
     public static Photonvision getInstance() {
         if (instance == null) {
             instance = new Photonvision();
         }
         return instance;
     }
 }