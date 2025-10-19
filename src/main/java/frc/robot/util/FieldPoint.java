package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import com.chaos131.util.FieldData;
import com.chaos131.vision.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldDimensions;
import frc.robot.Constants.RobotDimensions;
import frc.robot.subsystems.drive.Drive;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

/**
 * A class to help managing positions on the field (for either alliance color).
 */
public class FieldPoint {
  /** The pre-calculated red pose. */
  protected final Pose2d m_redPose;

  /** The pre-calculated blue pose. */
  protected final Pose2d m_bluePose;

  /** A potentially null name for the pose. */
  protected final String m_name;

  /** The corresponding alliance the original pose was built for. */
  protected final Alliance m_defaultAlliance;

  protected final double m_fieldLength = FieldDimensions.FieldLength;
  protected final double m_fieldWidth = FieldDimensions.FieldWidth;

  // List of named points on the field
  public static final FieldPoint processor = new FieldPoint("processor",
      new Pose2d(5.988, 0, Rotation2d.fromDegrees(90)));
  public static final FieldPoint leftSource = new FieldPoint("leftSource",
      new Pose2d(0.8512, 7.396, Rotation2d.fromDegrees(-60)));
  public static final FieldPoint rightSource = new FieldPoint("rightSource",
      new Pose2d(0.852, 0.6553, Rotation2d.fromDegrees(60)));
  public static final FieldPoint testPoint = new FieldPoint("testPoint",
      new Pose2d(10.0, 5.0, Rotation2d.fromDegrees(37)));
  public static final FieldPoint LollipopLeft = new FieldPoint("LollipopLeft",
      new Pose2d(1.204913, 5.854700, Rotation2d.fromDegrees(180)));
  public static final FieldPoint LollipopCenter = new FieldPoint("LollipopCenter",
      new Pose2d(1.204913, 4.02500, Rotation2d.fromDegrees(180)));
  public static final FieldPoint LollipopRight = new FieldPoint("LollipopRight",
      new Pose2d(1.204913, 2.197100, Rotation2d.fromDegrees(180)));
  public static final FieldPoint CenterBarge = new FieldPoint("CenterBarge", 
      new Pose2d(Meters.of(7.4).in(Meters), 4, Rotation2d.fromDegrees(180)));
      

  public static HashMap<Integer, AprilTag> aprilTagMap = FieldData.GetAprilTagMap("assets/frc2025.fmap");

  public static final FieldPoint ReefPose2 = new FieldPoint("reefPose2", aprilTagMap.get(22).pose2d);
  public static final FieldPoint ReefPose4 = new FieldPoint("reefPose4", aprilTagMap.get(17).pose2d);
  public static final FieldPoint ReefPose6 = new FieldPoint("reefPose6", aprilTagMap.get(18).pose2d);
  public static final FieldPoint ReefPose8 = new FieldPoint("reefPose8", aprilTagMap.get(19).pose2d);
  public static final FieldPoint ReefPose10 = new FieldPoint("reefPose10", aprilTagMap.get(20).pose2d);
  public static final FieldPoint ReefPose12 = new FieldPoint("reefPose12", aprilTagMap.get(21).pose2d);
  // public static final FieldPoint ReefCenter = new FieldPoint("reefCenter", ReefPose2.getBluePose().interpolate(ReefPose8.getBluePose().rotateBy(Rotation2d.fromDegrees(180)), 0.5));
  public static final FieldPoint ReefCenter = getMidPoint("reefCenter", ReefPose2, ReefPose8);

  /**
   * Creates a FieldPoint between two FieldPoints (ignores angles).
   */
  public static final FieldPoint getMidPoint(String newName, FieldPoint pointA, FieldPoint pointB) {
    var x = (pointA.getBluePose().getX() + pointB.getBluePose().getX()) / 2.0;
    var y = (pointA.getBluePose().getY() + pointB.getBluePose().getY()) / 2.0;
    return new FieldPoint(newName, new Pose2d(x, y, Rotation2d.kZero));
  }

  public static Map<String, FieldPoint> ReefClockPoses = Map.ofEntries(
      Map.entry("2L", getDrivePoseFromReefFace(ReefPose2.getBluePose(), true)),
      Map.entry("2R", getDrivePoseFromReefFace(ReefPose2.getBluePose(), false)),
      Map.entry("4L", getDrivePoseFromReefFace(ReefPose4.getBluePose(), false)),
      Map.entry("4R", getDrivePoseFromReefFace(ReefPose4.getBluePose(), true)),
      Map.entry("6L", getDrivePoseFromReefFace(ReefPose6.getBluePose(), false)),
      Map.entry("6R", getDrivePoseFromReefFace(ReefPose6.getBluePose(), true)),
      Map.entry("8L", getDrivePoseFromReefFace(ReefPose8.getBluePose(), false)),
      Map.entry("8R", getDrivePoseFromReefFace(ReefPose8.getBluePose(), true)),
      Map.entry("10L", getDrivePoseFromReefFace(ReefPose10.getBluePose(), true)),
      Map.entry("10R", getDrivePoseFromReefFace(ReefPose10.getBluePose(), false)),
      Map.entry("12L", getDrivePoseFromReefFace(ReefPose12.getBluePose(), true)),
      Map.entry("12R", getDrivePoseFromReefFace(ReefPose12.getBluePose(), false))
  );

  /**
   * Gets the april tabs for the blue reef.
   */
  public static ArrayList<AprilTag> blueReefAprilTags() {
    ArrayList<AprilTag> blueTagArrayList = new ArrayList<AprilTag>();
    blueTagArrayList.add(aprilTagMap.get(17));
    blueTagArrayList.add(aprilTagMap.get(18));
    blueTagArrayList.add(aprilTagMap.get(19));
    blueTagArrayList.add(aprilTagMap.get(20));
    blueTagArrayList.add(aprilTagMap.get(21));
    blueTagArrayList.add(aprilTagMap.get(22));
    return blueTagArrayList;
  }

  /**
   * Gets the april tags for the blue HP positions.
   */
  public static ArrayList<AprilTag> blueHpAprilTags() {
    ArrayList<AprilTag> blueTagArrayList = new ArrayList<AprilTag>();
    blueTagArrayList.add(aprilTagMap.get(12));
    blueTagArrayList.add(aprilTagMap.get(13));
    return blueTagArrayList;
  }

  /**
   * Gets all the drive positions we can consider for scoring on the reef (left
   * and right of each april tag).
   */
  public static ArrayList<FieldPoint> getReefDrivePoses() {
    ArrayList<FieldPoint> reefDrivePoses = new ArrayList<FieldPoint>();
    for (AprilTag aprilTag : blueReefAprilTags()) {
      reefDrivePoses.add(getDrivePoseFromReefFace(aprilTag.pose2d, true));
      reefDrivePoses.add(getDrivePoseFromReefFace(aprilTag.pose2d, false));
    }
    return reefDrivePoses;
  }

  private static FieldPoint getDrivePoseFromReefFace(Pose2d aprilTagPose, boolean isRight) {
    Pose2d pose = aprilTagPose.transformBy(
        new Transform2d(
            RobotDimensions.FrontBackLength.in(Meters) / 2 + RobotDimensions.RobotToReefCoralMargin.in(Meters),
            isRight ? FieldDimensions.ReefBranchRight.getY() : FieldDimensions.ReefBranchLeft.getY(),
            Rotation2d.fromDegrees(180)));
    return new FieldPoint("ReefDrivePose", pose);
  }

  /**
   * Gets all the drive positions we can consider for scoring on the reef (left
   * and right of each april tag).
   */
  public static Pose2d getNearestReefDrivePose(Drive swerveDrive, double stickBias) {
    ArrayList<FieldPoint> reefDrivePoses = new ArrayList<FieldPoint>();
    FieldPoint aprilTag = getNearestPoint(swerveDrive.getPose(), getReefAprilTagPoses());
    FieldPoint leftPose = getDrivePoseFromReefFace(aprilTag.getBluePose(), false);
    reefDrivePoses.add(leftPose);
    FieldPoint rightPose = getDrivePoseFromReefFace(aprilTag.getBluePose(), true);
    reefDrivePoses.add(rightPose);
    if (stickBias < -0.1) {
      return aprilTag.m_bluePose.getRotation().getMeasure().isNear(Degrees.of(180), Degrees.of(90)) ? leftPose.getCurrentAlliancePose() : rightPose.getCurrentAlliancePose();
    } else if (stickBias > 0.1) {
      return aprilTag.m_bluePose.getRotation().getMeasure().isNear(Degrees.of(180), Degrees.of(90)) ? rightPose.getCurrentAlliancePose() : leftPose.getCurrentAlliancePose();
    } else {
      return getNearestPoint(swerveDrive.getPose(), reefDrivePoses).getCurrentAlliancePose();
    }
  }

  /**
   * Gets all the drive positions we can consider for scoring on the reef (middle
   * april tag).
   */
  public static ArrayList<FieldPoint> getReefCenterDrivePose() {
    ArrayList<FieldPoint> reefCenterDrivePose = new ArrayList<FieldPoint>();
    for (AprilTag aprilTag : blueReefAprilTags()) {
      Pose2d centerPose = aprilTag.pose2d.transformBy(
          new Transform2d(
              RobotDimensions.FrontBackLength.in(Meters) / 2 + RobotDimensions.RobotToReefCoralMargin.in(Meters),
              FieldDimensions.ReefCenterBranch.getY(),
              Rotation2d.fromDegrees(180)));
      reefCenterDrivePose.add(new FieldPoint(aprilTag.id + " ReefCenter", centerPose));
    }
    return reefCenterDrivePose;
  }

  /** 
   * Returns an array list of all the Lollipop poses.
   */
  public static ArrayList<FieldPoint> getLollipopPoses() {
    ArrayList<FieldPoint> lollipopPoses = new ArrayList<FieldPoint>();
    lollipopPoses.add(LollipopCenter);
    lollipopPoses.add(LollipopLeft);
    lollipopPoses.add(LollipopRight);
    return lollipopPoses;
  }

  /**
   * Returns an array list of all the april tags on the reef is made.
   */
  public static ArrayList<FieldPoint> getReefAprilTagPoses() {
    ArrayList<FieldPoint> reefAprilTagPoses = new ArrayList<FieldPoint>();
    for (AprilTag aprilTag : blueReefAprilTags()) {
      reefAprilTagPoses.add(new FieldPoint(aprilTag.id + " ReefTag", aprilTag.pose2d));
    }
    return reefAprilTagPoses;
  }

  /**
   * Gets our positions for picking up coral from the HP stations.
   */
  public static ArrayList<FieldPoint> getHpDrivePoses() {
    ArrayList<FieldPoint> hpDrivePoses = new ArrayList<FieldPoint>();
    for (AprilTag aprilTag : blueHpAprilTags()) {
      Pose2d leftPose = aprilTag.pose2d.transformBy(
          new Transform2d(
              RobotDimensions.FrontBackLength.in(Meters) / 2 + 0.05,
              RobotDimensions.SideSideLength.in(Meters) / 2 + 0.05,
              Rotation2d.fromDegrees(180)));
      hpDrivePoses.add(new FieldPoint(aprilTag.id + " HPLeft", leftPose));
      Pose2d rightPose = aprilTag.pose2d.transformBy(
          new Transform2d(
              RobotDimensions.FrontBackLength.in(Meters) / 2 + 0.05,
              -RobotDimensions.SideSideLength.in(Meters) / 2 - 0.05,
              Rotation2d.fromDegrees(180)));
      hpDrivePoses.add(new FieldPoint(aprilTag.id + " HPRight", rightPose));
    }
    return hpDrivePoses;
  }

  /**
   * Finds the nearest point in an ArrayList of Field Points to the given robotPose.
   *
   * @param robotPose The robot's pose, could be anything but generally the robot pose
   * @param points ArrayList of the points
   * @return Whichever point was the closest, null if points is empty
   */
  public static FieldPoint getNearestPoint(Pose2d robotPose, ArrayList<FieldPoint> points) {
    FieldPoint nearest = null;
    double minimumDistance = 99;
    for (FieldPoint pt : points) {
      double distance = robotPose.relativeTo(pt.getCurrentAlliancePose()).getTranslation().getNorm();
      if (distance < minimumDistance) {
        minimumDistance = distance;
        nearest = pt;
      }
    }
    return nearest;
  }

  /**
   * Creates a new FieldPoint.
   *
   * @param name the name of the field point
   * @param pose the blue alliance pose on the field
   */
  public FieldPoint(String name, Pose2d pose) {
    m_name = name;
    m_bluePose = pose;
    m_defaultAlliance = Alliance.Blue;
    Translation2d poseTranslation = pose.getTranslation().minus(new Translation2d(m_fieldLength, m_fieldWidth).div(2));
    poseTranslation = poseTranslation.rotateBy(Rotation2d.fromDegrees(180));
    poseTranslation = poseTranslation.plus(new Translation2d(m_fieldLength, m_fieldWidth).div(2));
    m_redPose = new Pose2d(poseTranslation, pose.getRotation().plus(Rotation2d.fromDegrees(180)));
  }

  /**
   * Creates a new FieldPoint.
   *
   * @param name the name of the field point
   * @param pose the pose on the field
   * @param alliance the current alliance
   * 
   */
  public FieldPoint(String name, Pose2d pose, Alliance alliance) {
    m_name = name;
    m_defaultAlliance = Alliance.Blue;
    Translation2d poseTranslation = pose.getTranslation().minus(new Translation2d(m_fieldLength, m_fieldWidth).div(2));
    poseTranslation = poseTranslation.rotateBy(Rotation2d.fromDegrees(180));
    poseTranslation = poseTranslation.plus(new Translation2d(m_fieldLength, m_fieldWidth).div(2));
    m_bluePose = Alliance.Blue == alliance ? pose : new Pose2d(poseTranslation, pose.getRotation().plus(Rotation2d.fromDegrees(180)));
    m_redPose = Alliance.Blue == alliance ? new Pose2d(poseTranslation, pose.getRotation().plus(Rotation2d.fromDegrees(180))) : pose;
  }

  /**
   * Creates a new FieldPoint.
   *
   * @param name the name of the field point
   * @param swerveDrive current pose of the swerve drive
   * @param alliance the current alliance
   * 
   */
  public FieldPoint(String name, Drive swerveDrive, Alliance alliance) {
    this(name, swerveDrive.getPose(), alliance);
  }

  public Pose2d getBluePose() {
    return m_bluePose;
  }

  public Pose2d getRedPose() {
    return m_redPose;
  }


  public String getName() {
    return m_name;
  }

  /**
   * Gets the appropriate pose for the current alliance color. (Override this when
   * using unit
   * testing since the call to DriverStation will throw an exception)
   */
  protected Alliance getCurrentAlliance() {
    return DriverStation.getAlliance().orElse(m_defaultAlliance);
  }

  /**
   * Gets the pose for the robot's current alliance. (If the DS is not connected,
   * the default
   * alliance is used)
   */
  public Pose2d getCurrentAlliancePose() {
    return getCurrentAlliance() == Alliance.Blue ? m_bluePose : m_redPose;
  }

  /**
   * Get the distance between the robot and a specified point.
   */
  public Distance getDistance(Pose2d pose) {
    return Meters.of(getCurrentAlliancePose().getTranslation().getDistance(pose.getTranslation()));
  }
}
