package frc.robot.util;



import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Drive;

import java.util.ArrayList;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class PathUtil {

  // Create the constraints to use while pathfinding
  static PathConstraints constraints =
      new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

  /**
   * Drives to the FieldPoint on the field (respective of the current alliance).
   */
  public static Command driveToPoseCommand(FieldPoint targetPostion, Drive swerveDrive) {
    return new DeferredCommand(
        () -> AutoBuilder.pathfindToPose(targetPostion.getCurrentAlliancePose(), constraints, 0.0),
        Set.of(swerveDrive));
  }

  /**
   * Drives to the Pose on the field.
   */
  @Deprecated
  public static Command driveToPoseCommand(Pose2d targetPostion, Drive swerveDrive) {
    return new DeferredCommand(
        () -> AutoBuilder.pathfindToPose(targetPostion, constraints, 0.0),
        Set.of(swerveDrive));
  }

  

  /**
   * Finds the closest point along a line.
   */
  public static FieldPoint findClosestXpointOnLine(Drive swerveDrive, FieldPoint linePoint) {
    double x = linePoint.getBluePose().getX(); 
    double y = new FieldPoint("SwervePose", swerveDrive, DriverStation.getAlliance().get()).getBluePose().getY();
    Rotation2d rotation = linePoint.getBluePose().getRotation();
    return new FieldPoint("ClosestPointOnLine", new Pose2d(x, y, rotation));
  }

  /**
   * Drives to the closest FieldPoint on the field (respective of the current alliance).
   */
  // public static Command driveToClosestPointAutoCommand(
  //     ArrayList<FieldPoint> possibleTargets, Drive swerveDrive, double timeOutSeconds) {
  //   return new DeferredCommand(
  //       () -> {
  //         ArrayList<Pose2d> possiblePoses = new ArrayList<Pose2d>();
  //         for (int i = 0; i < possibleTargets.size(); i++) {
  //           possiblePoses.add(possibleTargets.get(i).getCurrentAlliancePose());
  //         }
  //         FieldPoint nearestPoint = FieldPoint.getNearestPoint(swerveDrive.getPose(), possibleTargets);
  //         Logger.recordOutput("Swerve/Nearest Point", nearestPoint.getCurrentAlliancePose());
  //         Command simpleDriveToPosition = new SimpleDriveToPositionV2(swerveDrive, nearestPoint).withTimeout(timeOutSeconds);
  //         return AutoBuilder.pathfindToPose(
  //             swerveDrive.getPose().nearest(possiblePoses), constraints, 0.0).andThen(simpleDriveToPosition);
  //       },
  //       Set.of(swerveDrive));
  // }

  /**
   * Drives to the target FieldPoint on the field (respective of the current alliance).
   */
  // public static Command driveToClosestPointAutoCommand(FieldPoint target, Drive swerveDrive, double timeOutSeconds) {
  //   return new DeferredCommand(
  //       () -> {
  //         Logger.recordOutput("Swerve/Nearest Point", target.getCurrentAlliancePose());
  //         Command simpleDriveToPosition = new SimpleDriveToPositionV2(swerveDrive, target).withTimeout(timeOutSeconds);
  //         return AutoBuilder.pathfindToPose(
  //           target.getCurrentAlliancePose(), constraints, 0.0).andThen(simpleDriveToPosition);
  //       },
  //       Set.of(swerveDrive));
  // }

  /**
   * Drives to the closest FieldPoint on the field (respective of the current alliance).
   */
  // public static Command driveToClosestPointTeleopCommandV2(
  //     ArrayList<FieldPoint> possibleTargets, Drive swerveDrive) {
  //   return new DeferredCommand(
  //       () -> {
  //         ArrayList<Pose2d> possiblePoses = new ArrayList<Pose2d>();
  //         for (int i = 0; i < possibleTargets.size(); i++) {
  //           possiblePoses.add(possibleTargets.get(i).getCurrentAlliancePose());
  //         }
  //         FieldPoint nearestPoint = FieldPoint.getNearestPoint(swerveDrive.getPose(), possibleTargets);
  //         Logger.recordOutput("Swerve/Nearest Point", nearestPoint.getCurrentAlliancePose());
  //         Command simpleDriveToPosition = new SimpleDriveToPositionV2(swerveDrive, nearestPoint);
  //         if (Robot.isSimulation()) {
  //           return AutoBuilder.pathfindToPose(
  //             swerveDrive.getPose().nearest(possiblePoses), constraints, 0.0)
  //             .andThen(simpleDriveToPosition)
  //             .andThen(new RunCommand(() -> swerveDrive.moveToTarget(0), swerveDrive));
  //         }
  //         return AutoBuilder.pathfindToPose(
  //             swerveDrive.getPose().nearest(possiblePoses), constraints, 0.0)
  //             .andThen(simpleDriveToPosition);
  //         // return simpleDriveToPosition;
  //       },
  //       Set.of(swerveDrive));
  // }
}

