// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.chaos131.util.DashboardNumber;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIdentifiers;
import frc.robot.Constants.GripperConstants;
import frc.robot.util.ChaosTalonFxs;
import frc.robot.util.ChaosTalonFxsTuner;

public class Intake extends SubsystemBase {
  private static final CurrentUnit Amps = null;
  private ChaosTalonFxs m_intakeMotor = new ChaosTalonFxs(CanIdentifiers.LauncherMotorCANIDRight);
  private ChaosTalonFxsTuner m_intakeTuner = new ChaosTalonFxsTuner("Launcher", m_intakeMotor);
   private DashboardNumber m_intakeSpeed = new DashboardNumber("IntakeSpeed", 0, true, null);

   private DashboardNumber m_supplyCurrentLimit = m_intakeTuner.tunable(
      "SupplyCurrentLimitLeft", GripperConstants.SupplyCurrentLimit.in(Amps), (config, newValue) -> config.CurrentLimits.SupplyCurrentLimit = newValue);
  private DashboardNumber m_statorCurrentLimit = m_intakeTuner.tunable(
      "StatorCurrentLimitLeft", GripperConstants.StatorCurrentLimit.in(Amps), (config, newValue) -> config.CurrentLimits.StatorCurrentLimit = newValue);

  /** Creates a new Intake. */
  public Intake() {
    m_intakeMotor.Configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    m_intakeMotor.Configuration.CurrentLimits.StatorCurrentLimit = m_statorCurrentLimit.get();
    m_intakeMotor.Configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_intakeMotor.Configuration.CurrentLimits.SupplyCurrentLimit = m_supplyCurrentLimit.get();

    m_intakeMotor.Configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; //TODO: double check this :3

    m_intakeMotor.Configuration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    m_intakeMotor.applyConfig();
  }

   public void setIntakeSpeed(double newSpeed) {
    m_intakeMotor.set(newSpeed);
  }

   public double getIntakeSpeed() {
    return m_intakeMotor.get();
  }

    public double getSelectedIntakeSpeed() {
    return m_intakeMotor.get();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
        Logger.recordOutput("Intake/IntakeSpeed", getIntakeSpeed());
  }
}
