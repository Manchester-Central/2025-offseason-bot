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

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants.ArmConstants.GripperConstants;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  /** This contains constants for our entire arm system. */
  public static class ArmConstants {
    /** Contains values for different known arm poses. */
    public static class ArmPoses {
      // Default Poses
      public static final ArmPose Stow = new ArmPose("Stow", 80.0, 0.25, -18.8);

      // Holding Poses
      //  public static final ArmPose HoldCoral = new ArmPose("HoldCoral", 85, 0.65, -70.9);
      public static final ArmPose HoldAlgae = new ArmPose("HoldAlgae", 83.5, 0.05, -38.3); // TODO tune this
      public static final ArmPose HoldCoralL1 = new ArmPose("HoldCoralL1", 85.0, 0.23, -35.5);


      // Coral Scoring Poses
      public static final ArmPose ScoreL1 = new ArmPose("ScoreL1", 61.5, 0.23, -35.5);

      // Coral Pickup Poses
      public static final ArmPose HpIntake = new ArmPose("HpIntake", 77, 0.54, -39.0); // Last updated 2/22/25
      public static final ArmPose FloorIntakeCoral = new ArmPose("FloorIntakeCoral", 17, 0.28, -13); //TODO tune this

    /** This contains constants for our Base Pivot. */
    public static class BasePivotConstants {
      public static final Angle MinAngle = Degrees.of(9); // TODO: go back to 20
      public static final Angle MaxAngle = Degrees.of(90);
      public static final Angle LowerSafetyAngle = Degrees.of(45);

      public static final double kP = 400.0;
      public static final double kI = 10.00;
      public static final double kD = 0.0;
      public static final double kG = 0.4;
      public static final double kS = 0.25;
      public static final double kV = 0.12;
      public static final double kA = 0.01;

      // Motion Magic
      public static final double MMCruiseVelocity = 0.3;
      public static final double MMAcceleration = 1;
      public static final double MMJerk = 100;

      public static final double MMCruiseVelocityHigh = 0.3;
      public static final double MMAccelerationHigh = 0.25;
      public static final double MMJerkHigh = 100;

      public static final LinearVelocity MMClimbCruiseVelocity = MetersPerSecond.of(0.15);

      public static final double SupplyCurrentLimit = 50;
      public static final double StatorCurrentLimit = 50; // TODO: up when climbing

      // Sensor Feedback
      public static final double RotorToSensorRatio = 302.4;
      public static final double SensorToMechanismRatio = 1.0;
      
      // Ramp Rates
      public static final double VoltageClosedLoopRampPeriod = 0.1;
      // Offset
      public static final double canCoderOffsetDegrees = -107.5;
    }
    
    /** This contains constants for our Gripper. */
    public static class GripperConstants {
      /** Creates quick DashboardNumbers for the gripper. */
      public static DashboardNumber gripperSpeed(double value, String name) { 
        return new DashboardNumber("GripperSpeeds/" + name, value, true, newValue -> {});
      }


  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
