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

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import com.chaos131.util.DashboardNumber;
import com.chaos131.util.DashboardUnit;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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

  public static class GeneralConstants {
    public static final Pose2d DefaultStartingPose = new Pose2d(7.1, 2, Rotation2d.fromDegrees(180));

    public static final double SlowModeModifier = 0.5;
  }

  /** This contains all of our constants for CAN IDs and Can Bus Names. */
  public static class CanIdentifiers {
    // public static final String CTRECANBus = "CTRE bus";
    public static final String RioCANBus = "rio";
    public static final String CTRECANBus = "canivore";

    // Arm (20s)
    public static final int ArmMotorCANID = 20;
    public static final int ArmCANcoderCANID = 21;

    // Gripper (50s)
    public static final int GripperMotorCANID = 52;
  }

  public static class GripperConstants {
    public static final Current SupplyCurrentLimit = Amps.of(40); //TODO: double check the values soon...
    public static final Current StatorCurrentLimit = Amps.of(40); //TODO: double check the values soon...
    public static final DashboardUnit<CurrentUnit, Current> CoralGrippedCurrentLimit = new DashboardUnit<>("Gripper/CoralGrippedCurrentLimit", Amps.of(10.0)); //TODO check this
  }

  public static class SimArmConstants {
    public static final double kP = 10.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
  }

  public static class ArmConstants {
    public static final DashboardUnit<AngleUnit, Angle> AngleTolerance = new DashboardUnit<AngleUnit,Angle>("Arm/AngleTolerance", Degrees.of(1));

    public static final Angle canCoderOffsetAngle = Rotations.of(-0.115);

    // TODO: Check all of these
    public static final double kP = 150.0;
    public static final double kI = 0.0;
    public static final double kD = 20.0;
    public static final double kG = 0.18;
    public static final double kS = 0.4;
    public static final double kV = 0.0;
    public static final double kA = 0.0;

    public static final double MMCruiseVelocity = 100.0;
    public static final double MMAcceleration = 100.0;
    public static final double MMJerk = 100.0;

    public static final double SupplyCurrentLimit = 50.0;
    public static final double StatorCurrentLimit = 50.0;

    public static final double RotorToSensorRatio = 89.1453;
    public static final double SensorToMechanismRatio = 1.0;
    public static final double VoltageClosedLoopRampPeriod = 0.1;

    public static final Angle MinAngle = Degrees.of(10);
    public static final Angle MaxAngle = Degrees.of(215); // TODO: Actual: 216

    public static class ArmPoses {
      public static final DashboardUnit<AngleUnit, Angle> StartingPose = new DashboardUnit<>("Poses/Starting", Degrees.of(90)); //TODO Tune all of these
      public static final DashboardUnit<AngleUnit, Angle> FloorIntakePose = new DashboardUnit<>("Poses/FloorIntake", Degrees.of(213));
      public static final DashboardUnit<AngleUnit, Angle> HPIntakePose = new DashboardUnit<>("Poses/HPIntake", Degrees.of(115));
      public static final DashboardUnit<AngleUnit, Angle> ScoreLowPose = new DashboardUnit<>("Poses/ScoreLow", Degrees.of(12));
      public static final DashboardUnit<AngleUnit, Angle> ScoreHighPose = new DashboardUnit<>("Poses/ScoreHigh", Degrees.of(22));
      public static final DashboardUnit<AngleUnit, Angle> DeAlgaePose = new DashboardUnit<>("Poses/DeAlgae", Degrees.of(140));
    }
  }

  public static class QuestNavConstants {
    public static final double RobotToQuestXInches = 2;
    public static final double RobotToQuestYInches = 4;
    public static final Rotation2d RobotToQuestRotation = new Rotation2d();
  }

  /** This contains constants for our robot dimensions. */
  public static class RobotDimensions {
    // Includes the Bumpers
    public static final Distance FrontBackLength = Inches.of(25.5+(3.25*2)); // frames plus 3.25" bumpers
    // Includes the Bumpers
    public static final Distance SideSideLength = Inches.of(29.5+(3.25*2)); // frames plus 3.25" bumpers
    // Margin between the robot and something to interact with
    public static final Distance RobotToReefMargin = Inches.of(4.5); 
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
    public static final Distance CoralWidth = Inches.of(4.5); 
    // Distance between center of robot + safety +center of reef 
    public static final Distance ReefScoringDistanceThreshold = Meters.of((RobotDimensions.FrontBackLength.in(Meters) / 2) + 0.912493).plus(Inches.of(13)); 
  }
}