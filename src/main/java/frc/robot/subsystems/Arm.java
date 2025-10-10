// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.chaos131.util.DashboardNumber;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CanIdentifiers;
import frc.robot.Robot;
import frc.robot.util.ChaosCanCoder;
import frc.robot.util.ChaosCanCoderTuner;
import frc.robot.util.ChaosTalonFx;
import frc.robot.util.ChaosTalonFxTuner;

public class Arm extends SubsystemBase {

  private Angle m_targetAngle = Degrees.of(120);
   private ChaosTalonFx m_motor = new ChaosTalonFx(CanIdentifiers.BasePivotMotorCANID);
   private ChaosCanCoder m_canCoder =
      new ChaosCanCoder(CanIdentifiers.BasePivotCANcoderCANID);
      private ChaosTalonFxTuner m_talonTuner = new ChaosTalonFxTuner("Base Pivot", m_motor);
      private ChaosCanCoderTuner m_canCoderTuner = new ChaosCanCoderTuner("Base Pivot", m_canCoder);

       private DashboardNumber m_canCoderOffsetDegrees = m_canCoderTuner.tunable("CANCoder Tuner",
      //Robot.isReal() ? BasePivotConstants.canCoderOffsetDegrees : SimBasePivotConstants.canCoderOffsetDegrees, (config, newValue) -> 
      ArmConstants.canCoderOffsetAngle.in(Degrees), (config, newValue) ->
      config.MagnetSensor.MagnetOffset = Degrees.of(newValue).in(Rotations));

      private DashboardNumber m_kp = m_talonTuner.tunable("kP", ArmConstants.kP, (config, newValue) -> config.Slot0.kP = newValue);
      private DashboardNumber m_ki = m_talonTuner.tunable("kI", ArmConstants.kI, (config, newValue) -> config.Slot0.kI = newValue);
      private DashboardNumber m_kd = m_talonTuner.tunable("kD", ArmConstants.kD, (config, newValue) -> config.Slot0.kD = newValue);
      private DashboardNumber m_kg = m_talonTuner.tunable("kG", ArmConstants.kG, (config, newValue) -> config.Slot0.kG = newValue);
      private DashboardNumber m_ks = m_talonTuner.tunable("kS", ArmConstants.kS, (config, newValue) -> config.Slot0.kS = newValue);
      private DashboardNumber m_kv = m_talonTuner.tunable("kV", ArmConstants.kV, (config, newValue) -> config.Slot0.kV = newValue);
      private DashboardNumber m_ka = m_talonTuner.tunable("kA", ArmConstants.kA, (config, newValue) -> config.Slot0.kA = newValue);

  /** Creates a new Arm. */
  public Arm() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
