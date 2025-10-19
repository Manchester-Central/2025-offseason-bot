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
import static edu.wpi.first.units.Units.Rotations;

import com.chaos131.util.DashboardUnit;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
    public static final Current SupplyCurrentLimit = Amps.of(60); //TODO: double check the values soon...
    public static final Current StatorCurrentLimit = Amps.of(60); //TODO: double check the values soon...
  }

  public static class SimArmConstants {
    public static final double kP = 10.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
  }

  public static class ArmConstants {
    public static final Angle canCoderOffsetAngle = Rotations.of(-0.115);

    // TODO: Check all of these
    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kG = 0.0;
    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;

    public static final double MMCruiseVelocity = 10.0;
    public static final double MMAcceleration = 10.0;
    public static final double MMJerk = 10.0;

    public static final double SupplyCurrentLimit = 50.0;
    public static final double StatorCurrentLimit = 50.0;

    public static final double RotorToSensorRatio = 89.1453;
    public static final double SensorToMechanismRatio = 1.0;
    public static final double VoltageClosedLoopRampPeriod = 0.5;

    public static final Angle MinAngle = Degrees.of(10);
    public static final Angle MaxAngle = Degrees.of(210); // TODO: Actual: 216

    public static class ArmPoses {
      public static final DashboardUnit<AngleUnit, Angle> StartingPose = new DashboardUnit<>("Poses/Starting", Degrees.of(90)); //TODO Tune all of these
      public static final DashboardUnit<AngleUnit, Angle> FloorIntakePose = new DashboardUnit<>("Poses/FloorIntake", Degrees.of(216));
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
}