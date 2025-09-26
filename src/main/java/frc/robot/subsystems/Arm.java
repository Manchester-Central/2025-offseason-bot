// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  public Arm() {}
  
  public class ArmValues {
      public Angle basePivotAngle;
      public Angle gripperPivotAngle;
      public double coralGripSpeed;
      public double climbSpeed;
      public double extenderLength;
      public boolean isBasePivotAtCloseAngle;
      public boolean isExtenderAtCloseLength;
      public boolean hasCoral;
      public boolean hasAlgae;
      public boolean hasCage;
      public boolean isGripperPivotAtCloseAngle;
    }

    public ArmValues getArmValues() {
      ArmValues values = new ArmValues();
      values.basePivotAngle = m_basePivot.getCurrentAngle();
      values.gripperPivotAngle = m_gripperPivot.getCurrentAngle();
      values.coralGripSpeed = m_gripper.getCoralGripSpeed();
      values.climbSpeed = m_climber.getClimbSpeed();
      values.extenderLength = m_extender.getCurrentLength();
      values.isBasePivotAtCloseAngle = m_basePivot.atClose();
      values.isExtenderAtCloseLength = m_extender.atClose();
      values.hasCoral = m_gripper.hasCoral();
      values.hasAlgae = m_gripper.hasAlgae();
      values.hasCage = m_climber.hasCage();
      values.isGripperPivotAtCloseAngle = m_gripperPivot.atClose();
      return values;
    }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
