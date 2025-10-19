// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.util.Color8Bit;
import static edu.wpi.first.units.Units.Rotations;

import org.littletonrobotics.junction.Logger;

import com.chaos131.util.DashboardNumber;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CanIdentifiers;
import frc.robot.Constants.SimArmConstants;
import frc.robot.Constants.ArmConstants.ArmPoses;
import frc.robot.util.ChaosCanCoder;
import frc.robot.util.ChaosCanCoderTuner;
import frc.robot.util.ChaosTalonFx;
import frc.robot.util.ChaosTalonFxTuner;

public class Arm extends SubsystemBase {

  private LoggedMechanismLigament2d m_ligament;
  
  private DCMotor m_DCMotor = DCMotor.getKrakenX60(1);
  private DCMotorSim m_motorSim = new DCMotorSim(
    LinearSystemId.createDCMotorSystem(m_DCMotor, 0.01, ArmConstants.RotorToSensorRatio),
    m_DCMotor,
    0.001,
    0.001);

  private Angle m_targetAngle = Degrees.of(120);
   private ChaosTalonFx m_motor = new ChaosTalonFx(CanIdentifiers.ArmMotorCANID);
   private ChaosCanCoder m_canCoder =
      new ChaosCanCoder(CanIdentifiers.ArmCANcoderCANID);
      private ChaosTalonFxTuner m_talonTuner = new ChaosTalonFxTuner("Arm", m_motor);
      private ChaosCanCoderTuner m_canCoderTuner = new ChaosCanCoderTuner("Arm", m_canCoder);

       private DashboardNumber m_canCoderOffsetDegrees = m_canCoderTuner.tunable("Offset_Degrees",
      //Robot.isReal() ? ArmConstants.canCoderOffsetDegrees : SimArmConstants.canCoderOffsetDegrees, (config, newValue) -> 
      ArmConstants.canCoderOffsetAngle.in(Degrees), (config, newValue) ->
      config.MagnetSensor.MagnetOffset = Degrees.of(newValue).in(Rotations));

      private DashboardNumber m_kp = m_talonTuner.tunable("kP", Robot.isSimulation() ? SimArmConstants.kP : ArmConstants.kP, (config, newValue) -> config.Slot0.kP = newValue);
      private DashboardNumber m_ki = m_talonTuner.tunable("kI", Robot.isSimulation() ? SimArmConstants.kI : ArmConstants.kI, (config, newValue) -> config.Slot0.kI = newValue);
      private DashboardNumber m_kd = m_talonTuner.tunable("kD", Robot.isSimulation() ? SimArmConstants.kD : ArmConstants.kD, (config, newValue) -> config.Slot0.kD = newValue);
      private DashboardNumber m_kg = m_talonTuner.tunable("kG", ArmConstants.kG, (config, newValue) -> config.Slot0.kG = newValue);
      private DashboardNumber m_ks = m_talonTuner.tunable("kS", ArmConstants.kS, (config, newValue) -> config.Slot0.kS = newValue);
      private DashboardNumber m_kv = m_talonTuner.tunable("kV", ArmConstants.kV, (config, newValue) -> config.Slot0.kV = newValue);
      private DashboardNumber m_ka = m_talonTuner.tunable("kA", ArmConstants.kA, (config, newValue) -> config.Slot0.kA = newValue);
      private DashboardNumber m_mmCruiseVelocity = m_talonTuner.tunable(
      "MM_CruiseVelocity", ArmConstants.MMCruiseVelocity, (config, newValue) -> config.MotionMagic.MotionMagicCruiseVelocity = newValue);
  private DashboardNumber m_mmAcceleration = m_talonTuner.tunable("MM_Acceleration", ArmConstants.MMAcceleration, (config, newValue) -> config.MotionMagic.MotionMagicAcceleration = newValue);
  private DashboardNumber m_mmJerk = m_talonTuner.tunable("MM_Jerk", ArmConstants.MMJerk, (config, newValue) -> config.MotionMagic.MotionMagicJerk = newValue);
  private DashboardNumber m_supplyCurrentLimit = m_talonTuner.tunable(
      "SupplyCurrentLimit", ArmConstants.SupplyCurrentLimit, (config, newValue) -> config.CurrentLimits.SupplyCurrentLimit = newValue);
  private DashboardNumber m_statorCurrentLimit = m_talonTuner.tunable(
      "StatorCurrentLimit", ArmConstants.StatorCurrentLimit, (config, newValue) -> config.CurrentLimits.StatorCurrentLimit = newValue);
  // Sensor Feedback
  private DashboardNumber m_rotorToSensorRatio = m_talonTuner.tunable("RotorToSensorRatio", ArmConstants.RotorToSensorRatio,
      (config, newValue) -> config.Feedback.RotorToSensorRatio = newValue);
  private DashboardNumber m_sensorToMechRatio = m_talonTuner.tunable("SensorToMechanismRatio", ArmConstants.SensorToMechanismRatio,
      (config, newValue) -> config.Feedback.SensorToMechanismRatio = newValue);

  // Ramp rates
  private DashboardNumber m_rampPeriod = m_talonTuner.tunable("VoltageClosedLoopRampPeriod", ArmConstants.VoltageClosedLoopRampPeriod,
      (config, newValue) -> config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = newValue);

  /** Creates a new Arm. */
  public Arm(LoggedMechanismLigament2d ligament) {
    m_ligament = ligament;
    m_ligament.setColor(new Color8Bit(150, 150, 150));
    m_ligament.setLineWeight(3);

    m_canCoder.Configuration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.75;
    m_canCoder.Configuration.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    m_canCoder.Configuration.MagnetSensor.MagnetOffset = Degrees.of(m_canCoderOffsetDegrees.get()).in(Rotations); // TODO: Check this value
    m_canCoder.applyConfig();

    m_motor.Configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_motor.Configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // TODO: Check Value
    m_motor.Configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_motor.Configuration.CurrentLimits.SupplyCurrentLimit = m_supplyCurrentLimit.get();
    m_motor.Configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    m_motor.Configuration.CurrentLimits.StatorCurrentLimit = m_statorCurrentLimit.get();
    m_motor.Configuration.Feedback.FeedbackRemoteSensorID = CanIdentifiers.ArmCANcoderCANID;
    m_motor.Configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder; // TODO: Check Value
    m_motor.Configuration.Feedback.RotorToSensorRatio = m_rotorToSensorRatio.get();
    m_motor.Configuration.ClosedLoopRamps.VoltageClosedLoopRampPeriod = m_rampPeriod.get();
    m_motor.Configuration.Feedback.SensorToMechanismRatio = m_sensorToMechRatio.get();
    m_motor.Configuration.MotionMagic.MotionMagicCruiseVelocity = m_mmCruiseVelocity.get();
    m_motor.Configuration.MotionMagic.MotionMagicAcceleration = m_mmAcceleration.get();
    m_motor.Configuration.MotionMagic.MotionMagicJerk = m_mmJerk.get();

    var slot0 = new Slot0Configs();
    slot0.kP = m_kp.get();
    slot0.kI = m_ki.get();
    slot0.kD = m_kd.get();
    slot0.kG = m_kg.get();
    slot0.kS = m_ks.get();
    slot0.kV = m_kv.get();
    slot0.kA = m_ka.get();
    slot0.GravityType = GravityTypeValue.Arm_Cosine;
    m_motor.Configuration.Slot0 = slot0;

    m_motor.applyConfig();

    m_motor.attachMotorSim(m_motorSim, ArmConstants.RotorToSensorRatio, ChassisReference.Clockwise_Positive, true);
    m_motor.attachCanCoderSim(m_canCoder);
    m_motor.setSimAngle(ArmPoses.StartingPose.get());
  }

  /**
   * Sets the direct speed [-1.0, 1.0] of the motors.
   */
  public void setSpeed(double speed) {
    if (getCurrentAngle().in(Degrees) > ArmConstants.MaxAngle.in(Degrees)) {
      speed = Math.min(speed, 0.0);
    } else if (getCurrentAngle().in(Degrees) < ArmConstants.MinAngle.in(Degrees)) {
      speed = Math.max(speed, 0.0);
    }
    m_motor.set(speed);
  }

  /**
   * Sets the target angle and tries to drive there.
   */
  public void setTargetAngle(Angle newAngle) {
    if (newAngle.in(Degrees) > ArmConstants.MaxAngle.in(Degrees)) {
      newAngle = ArmConstants.MaxAngle;
    } else if (newAngle.in(Degrees) < ArmConstants.MinAngle.in(Degrees)) {
      newAngle = ArmConstants.MinAngle;
    }

    m_targetAngle = newAngle;
    
    m_motor.moveToPositionMotionMagic(newAngle.in(Rotations)); // Rotation to match the cancoder units
  }

  public Angle getCurrentAngle() {
    return Rotations.of(
        m_canCoder.getAbsolutePosition().getValueAsDouble());
  }

  /**
   * Checks if the current angle is at the goal angle.
   */
  public boolean atTarget() {
    return Math.abs(getCurrentAngle().minus(m_targetAngle).in(Degrees)) < 0.5;
  }

  @Override
  public void simulationPeriodic() {
    m_motor.simUpdate();
  }

  /**
   * Set extender motor to Coast. :3
   */ 
  public void setMotorCoast() {
    m_motor.Configuration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    m_motor.applyConfig();
  }

  /**
   * Set extender motor to Brake. :3
   */ 
  public void setMotorBrake() {
    m_motor.Configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_motor.applyConfig();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_ligament.setAngle(getCurrentAngle().in(Degrees) - 90);

    super.periodic();
    Logger.recordOutput("Arm/Setpoint", m_targetAngle);
    Logger.recordOutput("Arm/CurrentAngle", getCurrentAngle().in(Degrees));
    Logger.recordOutput("Arm/AtTarget", atTarget());
    Logger.recordOutput("Arm/AngleError", getCurrentAngle().minus(m_targetAngle));
    Logger.recordOutput("Arm/Voltage", m_motor.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput("Arm/StatorCurrent", m_motor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Arm/SupplyCurrent", m_motor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("Arm/MotorAngle", Rotations.of(m_motor.getPosition().getValueAsDouble()).in(Degrees));
  }
}
