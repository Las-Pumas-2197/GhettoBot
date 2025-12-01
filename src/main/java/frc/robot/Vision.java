package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision {
  private final PhotonCamera leftCamera;
  private final PhotonCamera rightCamera;

  private final PhotonPoseEstimator leftPhotonPoseEstimator;
  private final PhotonPoseEstimator rightPhotonPoseEstimator;

  List<Optional<EstimatedRobotPose>> vision = new ArrayList<Optional<EstimatedRobotPose>>();

  // private Matrix<N3, N1> curStdDevs;

  public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
  public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

  public Vision() {

      leftCamera = new PhotonCamera("LeftCamera");
      rightCamera = new PhotonCamera("RightCamera");

      AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

      Transform3d leftCameraintrinsics = new Transform3d(
        Units.inchesToMeters(16.5),
        Units.inchesToMeters(8.3125),
        Units.inchesToMeters(6),
        new Rotation3d(
          0,
          0,
          0
        )
      );

      Transform3d rightCameraintrinsics = new Transform3d(
        Units.inchesToMeters(16.5),
        Units.inchesToMeters(-8.3125),
        Units.inchesToMeters(6),
        new Rotation3d(
          0,
          0,
          0
        )
      );

      leftPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, leftCameraintrinsics);
      rightPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, rightCameraintrinsics);

      leftPhotonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
      rightPhotonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  public Pair<Pose2d, Double> poseEstimate() {

    vision.clear();
    Optional<EstimatedRobotPose> leftVisionEst = Optional.empty();
    Optional<EstimatedRobotPose> rightVisionEst = Optional.empty();
    Pose2d pose = Pose2d.kZero;

    for (var change : leftCamera.getAllUnreadResults()) {
      leftVisionEst = leftPhotonPoseEstimator.update(change);
    }

    for (var change : rightCamera.getAllUnreadResults()) {
      rightVisionEst = rightPhotonPoseEstimator.update(change);
    }

    if (leftVisionEst.isPresent()) {vision.add(leftVisionEst);}
    if (rightVisionEst.isPresent()) {vision.add(rightVisionEst);}

    if (vision.size() != 0) {
      pose = vision.get(0).get().estimatedPose.toPose2d();
      if (vision.size() > 1) {
        pose = vision.get(0).get().estimatedPose.toPose2d();
        pose = pose.interpolate(vision.get(1).get().estimatedPose.toPose2d(), 0.5);
      }
    }

    if (!vision.isEmpty()) {
      return new Pair<Pose2d,Double>(pose, vision.get(0).get().timestampSeconds);
    } else {
      return null;
    }

  }

  public void periodic() {

   

    // leftVisionEst.ifPresent(
    //   est -> {
    //       leftEstConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, kSingleTagStdDevs);
    //   });

    // rightVisionEst.ifPresent(
    //   est -> {
    //       rightEstConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, kSingleTagStdDevs);
    //   });


  }

  // public void updatevision() {
    // Optional<EstimatedRobotPose> leftPoseVisionEst = Optional.empty();
    // for (var change : leftCamera.getAllUnreadResults()) {
    //   leftPoseVisionEst = leftPhotonPoseEstimator.update(change);
      // updateEstimationStdDevs(visionEst, change.getTargets());

    //   if (Robot.isSimulation()) {
    //       visionEst.ifPresentOrElse(
    //               est ->
    //                       getSimDebugField()
    //                               .getObject("VisionEstimation")
    //                               .setPose(est.estimatedPose.toPose2d()),
    //               () -> {
    //                   getSimDebugField().getObject("VisionEstimation").setPoses();
    //               });
    //   }

    //   visionEst.ifPresent(
    //           est -> {
    //               // Change our trust in the measurement based on the tags we can see
    //               var estStdDevs = getEstimationStdDevs();

    //               estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
    //           });
    // }
    
    // }

    // Optional<EstimatedRobotPose> rightPoseVisionEst = Optional.empty();
    // for (var change : leftCamera.getAllUnreadResults()) {
    //   rightPoseVisionEst = leftPhotonPoseEstimator.update(change);
    // }
  // }
  /**
   * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
   * deviations based on number of tags, estimation strategy, and distance from the tags.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   * @param targets All targets in this camera frame
   */
  // private void updateEstimationStdDevs(
  //         Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
  //     if (estimatedPose.isEmpty()) {
  //         // No pose input. Default to single-tag std devs
  //         curStdDevs = kSingleTagStdDevs;

  //     } else {
  //         // Pose present. Start running Heuristic
  //         var estStdDevs = kSingleTagStdDevs;
  //         int numTags = 0;
  //         double avgDist = 0;

  //         // Precalculation - see how many tags we found, and calculate an average-distance metric
  //         for (var tgt : targets) {
  //             var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
  //             if (tagPose.isEmpty()) continue;
  //             numTags++;
  //             avgDist +=
  //                     tagPose
  //                             .get()
  //                             .toPose2d()
  //                             .getTranslation()
  //                             .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
  //         }

  //         if (numTags == 0) {
  //             // No tags visible. Default to single-tag std devs
  //             curStdDevs = kSingleTagStdDevs;
  //         } else {
  //             // One or more tags visible, run the full heuristic.
  //             avgDist /= numTags;
  //             // Decrease std devs if multiple targets are visible
  //             if (numTags > 1) estStdDevs = kMultiTagStdDevs;
  //             // Increase std devs based on (average) distance
  //             if (numTags == 1 && avgDist > 4)
  //                 estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
  //             else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
  //             curStdDevs = estStdDevs;
  //         }
  //     }
  // }

  /**
   * Returns the latest standard deviations of the estimated pose from {@link
   * #getEstimatedGlobalPose()}, for use with {@link
   * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
   * only be used when there are targets visible.
   */
  // public Matrix<N3, N1> getEstimationStdDevs() {
  //     return kSingleTagStdDevs;
  // }

  @FunctionalInterface
  public static interface EstimateConsumer {
      public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
  }

}
