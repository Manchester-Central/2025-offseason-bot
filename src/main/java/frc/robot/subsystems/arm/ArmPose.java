// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;

import com.chaos131.util.DashboardNumber;
import edu.wpi.first.units.measure.Angle;
import java.util.Optional;

/** A class for maintain goal points for certain arm poses. */
public class ArmPose {
  private String m_name;
  private Angle m_basePivotAngle;
  private double m_extensionMeters;
  private Angle m_gripperPivotAngle;
  private Optional<Angle> m_basePivotSafetyAngle = Optional.empty();

  /** Creates a new arm pose. */
  public ArmPose(String name, double basePivotDegrees, double extensionMeters, double gripperPivotDegrees) {
    this(name, Degrees.of(basePivotDegrees), extensionMeters, Degrees.of(gripperPivotDegrees));
  }

  /** Creates a new arm pose. */
  public ArmPose(String name, Angle basePivotAngle, double extensionMeters, Angle gripperPivotAngle) {
    m_name = name;
    m_basePivotAngle = basePivotAngle;
    m_extensionMeters = extensionMeters;
    m_gripperPivotAngle = gripperPivotAngle;

    new DashboardNumber(
      "Arm/Pose/" + m_name + "/Base Pivot Degrees", basePivotAngle.in(Degrees), true, newDegrees -> m_basePivotAngle = Degrees.of(newDegrees)
    );
    new DashboardNumber(
      "Arm/Pose/" + m_name + "/Extension Meters", extensionMeters, true, newMeters -> m_extensionMeters = newMeters
    );
    new DashboardNumber(
      "Arm/Pose/" + m_name + "/Gripper Pivot Degrees", gripperPivotAngle.in(Degrees), true, newDegrees -> m_gripperPivotAngle = Degrees.of(newDegrees)
    );
  }

  /**
   * add an angle that the base pivot will go to before the proper target.
   */
  public ArmPose withBasePivotSafety(Angle safetyRotation) {
    m_basePivotSafetyAngle = Optional.of(safetyRotation);
    return this;
  }
  
  /**
   * add an angle that the base pivot will go to before the proper target.
   */
  public ArmPose withBasePivotSafety(double safetyRotation) {
    return withBasePivotSafety(Degrees.of(safetyRotation));
  }



  public Angle getBasePivotAngle() {
    return m_basePivotAngle;
  }

  public double getExtensionMeters() {
    return m_extensionMeters;
  }

  public Angle getGripperPivotAngle() {
    return m_gripperPivotAngle;
  }

  public Optional<Angle> getBasePivotSafetyAngle() {
    return m_basePivotSafetyAngle;
  }
}
