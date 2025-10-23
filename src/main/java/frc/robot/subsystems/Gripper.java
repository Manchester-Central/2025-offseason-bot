// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import com.chaos131.util.DashboardNumber;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CanIdentifiers;
import frc.robot.Constants.GripperConstants;
import frc.robot.util.ChaosTalonFx;
import frc.robot.util.ChaosTalonFxTuner;

public class Gripper extends SubsystemBase {
  private ChaosTalonFx m_gripperMotor = new ChaosTalonFx(CanIdentifiers.GripperMotorCANID);
  private ChaosTalonFxTuner m_gripperTuner = new ChaosTalonFxTuner("Gripper", m_gripperMotor);

  private static boolean m_hasCoralGripped = false;

  private DashboardNumber m_supplyCurrentLimit = m_gripperTuner.tunable(
      "SupplyCurrentLimit", GripperConstants.SupplyCurrentLimit.in(Amps), (config, newValue) -> config.CurrentLimits.SupplyCurrentLimit = newValue);
  private DashboardNumber m_statorCurrentLimit = m_gripperTuner.tunable(
      "StatorCurrentLimit", GripperConstants.StatorCurrentLimit.in(Amps), (config, newValue) -> config.CurrentLimits.StatorCurrentLimit = newValue);
      
  private LoggedMechanismLigament2d m_ligament;

  /** Creates a new Gripper. */
  public Gripper(LoggedMechanismLigament2d ligament) {
    m_ligament = ligament;
    
    m_gripperMotor.Configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    m_gripperMotor.Configuration.CurrentLimits.StatorCurrentLimit = m_statorCurrentLimit.get();
    m_gripperMotor.Configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_gripperMotor.Configuration.CurrentLimits.SupplyCurrentLimit = m_supplyCurrentLimit.get();

    // m_gripperMotor.Configuration.CurrentLimits.SupplyCurrentLowerLimit = GripperConstants.CoralSupplyCurrentLowerLimit.in(Amps);
    // m_gripperMotor.Configuration.CurrentLimits.SupplyCurrentLowerTime = GripperConstants.CoralSupplyCurrentLowerTime.in(Seconds);
    //m_gripperMotor.Configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    m_gripperMotor.Configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_gripperMotor.applyConfig();
  }

  /**
   * Sets the speed [-1.0, 1.0] of the coral gripper.
   */
  public void setGripSpeed(double newSpeed) {
    m_gripperMotor.set(newSpeed);
  }

  public double getGripSpeed() {
    return m_gripperMotor.get();
  }

  /**
   * Checks if there is a coral at the sensor.
   */
  public boolean hasCoral() {
    return m_hasCoralGripped; 
  }

  /**                                                    //TODO: Add function and stuff...
   * Checks if there is a coral at the sensor.
   */
  // public boolean hasCoralNoDebounce() {
  // return hasCoralFrontNoDebounce() || hasCoralBackNoDebounce();
  // }

  @Override
  public void periodic() {
    boolean currentLimitReached = m_gripperMotor.getStatorCurrent().getValue().gt(GripperConstants.CoralGrippedCurrentLimit.get());
    m_hasCoralGripped = currentLimitReached;
    //TODO: 

    Logger.recordOutput("Gripper/HasCoral", hasCoral());
    // This method will be called once per scheduler run

    if (hasCoral()) {
      m_ligament.setColor(new Color8Bit(255, 255, 255));
      m_ligament.setLineWeight(8);
    } else {
      m_ligament.setColor(new Color8Bit(50, 50, 255));
      m_ligament.setLineWeight(3);
    }
  }
}
