// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import java.util.jar.Attributes.Name;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import com.chaos131.util.DashboardNumber;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIdentifiers;
import frc.robot.Constants.GripperConstants;
import frc.robot.util.ChaosTalonFxs;
import frc.robot.util.ChaosTalonFxsTuner;

public class Launcher extends SubsystemBase {
  private ChaosTalonFxs m_launcherMotorLeft = new ChaosTalonFxs(CanIdentifiers.LauncherMotorCANIDLeft);
  private ChaosTalonFxs m_launcherMotorRight = new ChaosTalonFxs(CanIdentifiers.LauncherMotorCANIDRight);
  private ChaosTalonFxsTuner m_launcherTunerLeft = new ChaosTalonFxsTuner("Launcher", m_launcherMotorLeft);
  private ChaosTalonFxsTuner m_launcherTunerRight = new ChaosTalonFxsTuner("Launcher", m_launcherMotorRight);

  private DashboardNumber m_leftLaunchSpeed = new DashboardNumber("leftLaunchSpeed", 0, true, null);
  private DashboardNumber m_rightLaunchSpeed = new DashboardNumber("rightLaunchSpeed", 0, true, null);

  //private static boolean m_hasCoralGripped = false;
 // private static boolean m_hasCoralGrippedSim = false;
  //private Debouncer coralDebouncer = new Debouncer(0.5,DebounceType.kBoth);
  private DashboardNumber m_supplyCurrentLimitLeft = m_launcherTunerLeft.tunable(
      "SupplyCurrentLimitLeft", GripperConstants.SupplyCurrentLimit.in(Amps), (config, newValue) -> config.CurrentLimits.SupplyCurrentLimit = newValue);
  private DashboardNumber m_statorCurrentLimitLeft = m_launcherTunerLeft.tunable(
      "StatorCurrentLimitLeft", GripperConstants.StatorCurrentLimit.in(Amps), (config, newValue) -> config.CurrentLimits.StatorCurrentLimit = newValue);
      
  private DashboardNumber m_supplyCurrentLimitRight = m_launcherTunerRight.tunable(
      "SupplyCurrentLimitRight", GripperConstants.SupplyCurrentLimit.in(Amps), (config, newValue) -> config.CurrentLimits.SupplyCurrentLimit = newValue);
  private DashboardNumber m_statorCurrentLimitRight = m_launcherTunerRight.tunable(
      "StatorCurrentLimitRight", GripperConstants.StatorCurrentLimit.in(Amps), (config, newValue) -> config.CurrentLimits.StatorCurrentLimit = newValue);
  
      private LoggedMechanismLigament2d m_ligament;

  /** Creates a new Gripper. */
  public Launcher(LoggedMechanismLigament2d ligament) {
    m_ligament = ligament;
    
    m_launcherMotorLeft.Configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    m_launcherMotorLeft.Configuration.CurrentLimits.StatorCurrentLimit = m_statorCurrentLimitLeft.get();
    m_launcherMotorLeft.Configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_launcherMotorLeft.Configuration.CurrentLimits.SupplyCurrentLimit = m_supplyCurrentLimitLeft.get();

    m_launcherMotorLeft.Configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    m_launcherMotorLeft.Configuration.CurrentLimits.StatorCurrentLimit = m_statorCurrentLimitRight.get();
    m_launcherMotorLeft.Configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_launcherMotorLeft.Configuration.CurrentLimits.SupplyCurrentLimit = m_supplyCurrentLimitRight.get();

    // m_gripperMotor.Configuration.CurrentLimits.SupplyCurrentLowerLimit = GripperConstants.CoralSupplyCurrentLowerLimit.in(Amps);
    // m_gripperMotor.Configuration.CurrentLimits.SupplyCurrentLowerTime = GripperConstants.CoralSupplyCurrentLowerTime.in(Seconds);
    m_launcherMotorLeft.Configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; //TODO: double check this :3
    m_launcherMotorRight.Configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; //TODO: this too :D


    m_launcherMotorLeft.Configuration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    m_launcherMotorRight.Configuration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    m_launcherMotorLeft.applyConfig();
    m_launcherMotorRight.applyConfig();
  }

  /**
   * Sets the speed [-1.0, 1.0] of the coral gripper.
   */
  public void setLaunchSpeedLeft(double newSpeed) {
    m_launcherMotorLeft.set(newSpeed);
  }

  public void setLaunchSpeedRight(double newSpeed) {
    m_launcherMotorRight.set(newSpeed);
  }

  public double getLaunchSpeedLeft() {
    return m_launcherMotorLeft.get();
  }

  public double getLaunchSpeedRight() {
    return m_launcherMotorRight.get();
  }

  public double getSelectedLaunchSpeedLeft() {
    return m_launcherMotorLeft.get();
  }

  public double getSelectedLaunchSpeedRight() {
    return m_launcherMotorRight.get();
  }


  /**
   * Checks if there is a coral at the sensor.
   */
  public boolean hasCoral() {
    // if (Robot.isSimulation()) {
    //   return m_hasCoralGrippedSim; 
    // }
    return true;
    //return m_hasCoralGripped; 
  }

  /**
   * Sets the state of coral in sim
   */
  public void setCoralSim(boolean hasCoral) {
    //m_hasCoralGrippedSim = hasCoral;
  }

  /**                                                    //TODO: Add function and stuff...
   * Checks if there is a coral at the sensor.
   */
  // public boolean hasCoralNoDebounce() {
  // return hasCoralFrontNoDebounce() || hasCoralBackNoDebounce();
  // }

  @Override
  public void periodic() {
   // boolean currentLimitReached = m_gripperMotor.getStatorCurrent().getValue().gt(GripperConstants.CoralGrippedCurrentLimit.get());
    //m_hasCoralGripped = coralDebouncer.calculate(currentLimitReached);
    //TODO: ???

    //Logger.recordOutput("Gripper/HasCoral", hasCoral());
    Logger.recordOutput("Launcher/LaunchSpeedLeft", getLaunchSpeedLeft());
    Logger.recordOutput("Launcher/LaunchSpeedRight", getLaunchSpeedRight());
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
