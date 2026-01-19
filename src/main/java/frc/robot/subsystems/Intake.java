// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import org.littletonrobotics.junction.Logger;

import com.chaos131.util.DashboardNumber;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIdentifiers;
import frc.robot.Constants.GripperConstants;
import frc.robot.util.ChaosTalonFxs;
import frc.robot.util.ChaosTalonFxsTuner;

public class Intake extends SubsystemBase {
  private ChaosTalonFxs m_intakeMotor = new ChaosTalonFxs(CanIdentifiers.IntakeMotorCANID);
  private ChaosTalonFxs m_kickerMotor = new ChaosTalonFxs(CanIdentifiers.KickerMotorCANID);

  private ChaosTalonFxsTuner m_intakeTuner = new ChaosTalonFxsTuner("Intake", m_intakeMotor);
  private ChaosTalonFxsTuner m_kickerTuner = new ChaosTalonFxsTuner("Kicker", m_kickerMotor);

  private DashboardNumber m_intakeSpeed = new DashboardNumber("IntakeSpeed", 0.2, true, (x) -> {});
  private DashboardNumber m_kickerSpeed = new DashboardNumber("KickerSpeed", 0.2, true, (x) -> {});

  private DashboardNumber m_supplyCurrentLimitIntake = m_intakeTuner.tunable(
      "SupplyCurrentLimitIntake", GripperConstants.SupplyCurrentLimit.in(Amps), (config, newValue) -> config.CurrentLimits.SupplyCurrentLimit = newValue);
  private DashboardNumber m_statorCurrentLimitIntake = m_intakeTuner.tunable(
      "StatorCurrentLimitIntake", GripperConstants.StatorCurrentLimit.in(Amps), (config, newValue) -> config.CurrentLimits.StatorCurrentLimit = newValue);

  private DashboardNumber m_supplyCurrentLimitKicker = m_kickerTuner.tunable(
      "SupplyCurrentLimitLeftKicker", GripperConstants.SupplyCurrentLimit.in(Amps), (config, newValue) -> config.CurrentLimits.SupplyCurrentLimit = newValue);
  private DashboardNumber m_statorCurrentLimitKicker = m_kickerTuner.tunable(
      "StatorCurrentLimitLeftKicker", GripperConstants.StatorCurrentLimit.in(Amps), (config, newValue) -> config.CurrentLimits.StatorCurrentLimit = newValue);
    
  /** Creates a new Intake. */
  public Intake() {
    m_intakeMotor.Configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    m_intakeMotor.Configuration.CurrentLimits.StatorCurrentLimit = m_statorCurrentLimitIntake.get();
    m_intakeMotor.Configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_intakeMotor.Configuration.CurrentLimits.SupplyCurrentLimit = m_supplyCurrentLimitIntake.get();

    m_kickerMotor.Configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    m_kickerMotor.Configuration.CurrentLimits.StatorCurrentLimit = m_statorCurrentLimitKicker.get();
    m_kickerMotor.Configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_kickerMotor.Configuration.CurrentLimits.SupplyCurrentLimit = m_supplyCurrentLimitKicker.get();
    
    m_intakeMotor.Configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; //TODO: double check this :3
    m_kickerMotor.Configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; //TODO: double check this

    m_intakeMotor.Configuration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    m_kickerMotor.Configuration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    m_intakeMotor.applyConfig();
    m_kickerMotor.applyConfig();
  }

  public void setIntakeSpeed(double intakeSpeed, double kickSpeed) {
    m_intakeMotor.set(intakeSpeed);
    m_kickerMotor.set(kickSpeed);
  }

  public double getIntakeSpeed() {
    return m_intakeMotor.get();
  }

  public double getKickerSpeed() {
    return m_kickerMotor.get();
  }

  public double getSelectedIntakeSpeed() {
    return m_intakeSpeed.get();
  }

  public double getSelectedKickerSpeed() {
    return m_kickerSpeed.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Intake/IntakeSpeed", getIntakeSpeed());
    Logger.recordOutput("Intake/KickerSpeed", getKickerSpeed());
  }
}
