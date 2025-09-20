// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  /** This contains all of our constants for CAN IDs and Can Bus Names. */
  public static class CanIdentifiers {
  }
  
  public static class SwerveConstants { 
  }

  public static class ArmConstants { 
  }

  public static class GripperConstants {
  }

  public static class QuestConstants {
  } 

  /** This contains constants for the field. */
  public static class FieldDimensions {
    // Value taken from Limelight fmap
    public static final double FieldLength = 17.5482504;
    // Value taken from Limelight fmap
    public static final double FieldWidth = 8.0519016;
    // Transform to the Driver Perspective Left Reef from the perspective of the April Tag
    public static final Transform2d ReefBranchLeft =
        new Transform2d(-0.0536, -0.1643, Rotation2d.fromDegrees(0));
    // Transform to the Driver Perspective Right Reef from the perspective of the April Tag
    public static final Transform2d ReefBranchRight =
        new Transform2d(-0.0536, 0.1643, Rotation2d.fromDegrees(0));
    // Trasform the Driver Perspective Center Reef from the perspective of the April Tag
    public static final Transform2d ReefCenterBranch =
        new Transform2d(-0.0536, 0, Rotation2d.fromDegrees(0));
    // Value taken from field cad
    public static final Distance TroughHeight = Meters.of(0.5175);
    // Value taken from field cad
    public static final Distance Reef1 = Meters.of(0.7763);
    // Value taken from field cad
    public static final Distance Reef2 = Meters.of(1.1794);
    // Value taken from field cad
    public static final Distance Reef3 = Meters.of(1.8287);
    // Value taken from field cad
    public static final Distance Barge = Meters.of(1.0);
    public static final Distance CoralWidth = Inches.of(4.5); 
    // Distance between center of robot + safety +center of reef 
    public static final Distance ReefScoringDistanceThreshold = Meters.of((RobotDimensions.FrontBackLength.in(Meters) / 2) + 0.912493).plus(Inches.of(13)); 
  }

  /** This contains constants for our robot dimensions. */
  public static class RobotDimensions {
    // Includes the Bumpers
    public static final Distance FrontBackLength = Meters.of(0.9157);
    // Includes the Bumpers
    public static final Distance SideSideLength = Meters.of(0.9157);
    // Buffer space to use between the end effector and an interaction point
    public static final Distance CoralPlacementMargin = Meters.of(0.03);
    // Robot length buffer
    public static final Distance RobotToReefMargin = Meters.of(0.015);
    public static final Distance RobotToReefCoralMargin = RobotToReefMargin.plus(FieldDimensions.CoralWidth); // silly :3
    public static final double WristToCoralIntakeAxle = 0.169627; // -0.209097 down, but who cares?
    public static final Angle AlgaeBarAngle = Degrees.of(117.160050);
    // Distance from the robot origin to the axle for the Base Pivot
    public static final Transform2d BasePivotOffset = new Transform2d(-0.1973, 0.1762, Rotation2d.kZero);

    /** Distance from the dynamic arm position to the wrist on the gripper mechanism.
     * If the arm were all the way down, then this would be the distance from the center of the axle of 
     * the Base Pivot to the center of the axle of the wrist pivot. 
     * This must be rotated by the angle of the Base Pivot at some point in the Forward Kinematics. */
    public static final Transform2d ArmToWristOffset = new Transform2d(0.1784, 0.4259, Rotation2d.kZero);

    // Distance from the wrist to the back of the coral, placed as if it was as in the CAD mockup (Mar/14)
    public static final Transform2d WristToCoralFront = new Transform2d(0.382352, -0.164647, Rotation2d.kZero);
    // Distance from the wrist to the front of the coral, placed as if it was as in the CAD mockup (Mar/14)
    public static final Transform2d WristToCoralBack = new Transform2d(0.080727, -0.164647, Rotation2d.kZero);
  }
}
