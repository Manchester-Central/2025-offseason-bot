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

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
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

  /** This contains all of our constants for CAN IDs and Can Bus Names. */
  public static class CanIdentifiers {
    // public static final String CTRECANBus = "CTRE bus";
    public static final String RioCANBus = "rio";

    // Swerve (30s & 40s)
    // public static final int FLSpeedCANID = 30;
    // public static final int FLAngleCANID = 31;
    // public static final int FLAbsoEncoCANID = 32;
    // public static final int FRSpeedCANID = 33;
    // public static final int FRAngleCANID = 34;
    // public static final int FRAbsoEncoCANID = 35;
    // public static final int BLSpeedCANID = 39;
    // public static final int BLAngleCANID = 40;
    // public static final int BLAbsoEncoCANID = 41;
    // public static final int BRSpeedCANID = 36;
    // public static final int BRAngleCANID = 37;
    // public static final int BRAbsoEncoCANID = 38;
    // public static final int GyroCANID = 45;

    // Base Pivot (20s)
    public static final int BasePivotMotorCANID = 20;
    public static final int BasePivotCANcoderCANID = 21;

    // Gripper (70s)
    public static final int GripperMotorCANID = 52; // TODO: set on robot
  }
  public static class GripperConstants {

    public static final Current SupplyCurrentLimit = Amps.of(60); //TODO: double check the values soon...
    public static final Current StatorCurrentLimit = Amps.of(60); //TODO: double check the values soon...
  }
  public static class ArmConstants {
    public static final Angle canCoderOffsetAngle = Degrees.of(0.0); //TODO: add real value to this...

    // TODO: Check all of these
    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kG = 0.0;
    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;

    public static final double MMCruiseVelocity = 0.0;
    public static final double MMAcceleration = 0.0;
    public static final double MMJerk = 0.0;

    public static final double SupplyCurrentLimit = 0.0;
    public static final double StatorCurrentLimit = 0.0;

    public static final double RotorToSensorRatio = 0.0;
    public static final double SensorToMechanismRatio = 0.0;
    public static final double VoltageClosedLoopRampPeriod = 0.0;

    public static final Angle MinAngle = Degrees.of(90); // Actual: 0
    public static final Angle MaxAngle = Degrees.of(180); // Actual: 210
  }
  public static class QuestNavConstants {
    public static final double RobotToQuestXInches = 2;
    public static final double RobotToQuestYInches = 4;
    public static final Rotation2d RobotToQuestRotation = new Rotation2d();
  }
}