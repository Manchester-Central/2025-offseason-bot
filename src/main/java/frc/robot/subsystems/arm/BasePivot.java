// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import com.chaos131.util.DashboardNumber;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.ArmConstants.BasePivotConstants;
import frc.robot.Constants.ArmConstants.ExtenderConstants;
import frc.robot.Constants.CanIdentifiers;
import frc.robot.Robot;
import frc.robot.SimConstants.SimBasePivotConstants;
import frc.robot.subsystems.arm.Arm.ArmValues;
import frc.robot.utils.ChaosCanCoder;
import frc.robot.utils.ChaosCanCoderTuner;
import frc.robot.utils.ChaosTalonFx;
import frc.robot.utils.ChaosTalonFxTuner;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class BasePivot extends AbstractArmPart {
  private double m_gearRatio = BasePivotConstants.RotorToSensorRatio;
  private double m_jkgMetersSquared = 1.0;
  private Angle m_targetAngle = Degrees.of(120);
  private DCMotor m_dcMotor = DCMotor.getKrakenX60(1);
  private DCMotorSim m_motorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(m_dcMotor, m_jkgMetersSquared, m_gearRatio),
          m_dcMotor,
          0.001,
          0.001);
  private ChaosTalonFx m_motor = new ChaosTalonFx(CanIdentifiers.BasePivotMotorCANID);
  private ChaosCanCoder m_canCoder =
      new ChaosCanCoder(CanIdentifiers.BasePivotCANcoderCANID);
  private ChaosTalonFxTuner m_talonTuner = new ChaosTalonFxTuner("Base Pivot", m_motor);
  private ChaosCanCoderTuner m_canCoderTuner = new ChaosCanCoderTuner("Base Pivot", m_canCoder);

  private DashboardNumber m_canCoderOffsetDegrees = m_canCoderTuner.tunable("CANCoder Tuner",
      Robot.isReal() ? BasePivotConstants.canCoderOffsetDegrees : SimBasePivotConstants.canCoderOffsetDegrees, (config, newValue) -> 
      config.MagnetSensor.MagnetOffset = Degrees.of(newValue).in(Rotations));

  private DashboardNumber m_kp = m_talonTuner.tunable("kP", Robot.isReal() ? BasePivotConstants.kP : SimBasePivotConstants.kP, (config, newValue) -> config.Slot0.kP = newValue);
  private DashboardNumber m_ki = m_talonTuner.tunable("kI", Robot.isReal() ? BasePivotConstants.kI : SimBasePivotConstants.kI, (config, newValue) -> config.Slot0.kI = newValue);
  private DashboardNumber m_kd = m_talonTuner.tunable("kD", Robot.isReal() ? BasePivotConstants.kD : SimBasePivotConstants.kD, (config, newValue) -> config.Slot0.kD = newValue);
  private DashboardNumber m_kg = m_talonTuner.tunable("kG", BasePivotConstants.kG, (config, newValue) -> config.Slot0.kG = newValue);
  private DashboardNumber m_ks = m_talonTuner.tunable("kS", BasePivotConstants.kS, (config, newValue) -> config.Slot0.kS = newValue);
  private DashboardNumber m_kv = m_talonTuner.tunable("kV", BasePivotConstants.kV, (config, newValue) -> config.Slot0.kV = newValue);
  private DashboardNumber m_ka = m_talonTuner.tunable("kA", BasePivotConstants.kA, (config, newValue) -> config.Slot0.kA = newValue);
  private DashboardNumber m_mmCruiseVelocity = m_talonTuner.tunable(
      "MM_CruiseVelocity", BasePivotConstants.MMCruiseVelocity, (config, newValue) -> config.MotionMagic.MotionMagicCruiseVelocity = newValue);
  private DashboardNumber m_mmAcceleration = m_talonTuner.tunable("MM_Acceleration", BasePivotConstants.MMAcceleration, (config, newValue) -> config.MotionMagic.MotionMagicAcceleration = newValue);
  private DashboardNumber m_mmJerk = m_talonTuner.tunable("MM_Jerk", BasePivotConstants.MMJerk, (config, newValue) -> config.MotionMagic.MotionMagicJerk = newValue);
  private DashboardNumber m_mmCruiseVelocityHigh = m_talonTuner.tunable(
      "MM_CruiseVelocityHigh", BasePivotConstants.MMCruiseVelocityHigh, (config, newValue) -> {});
  private DashboardNumber m_mmAccelerationHigh = m_talonTuner.tunable("MM_AccelerationHigh", BasePivotConstants.MMAccelerationHigh, (config, newValue) -> {});
  private DashboardNumber m_mmJerkHigh = m_talonTuner.tunable("MM_JerkHigh", BasePivotConstants.MMJerkHigh, (config, newValue) -> {});
  private DashboardNumber m_supplyCurrentLimit = m_talonTuner.tunable(
      "SupplyCurrentLimit", BasePivotConstants.SupplyCurrentLimit, (config, newValue) -> config.CurrentLimits.SupplyCurrentLimit = newValue);
  private DashboardNumber m_statorCurrentLimit = m_talonTuner.tunable(
      "StatorCurrentLimit", BasePivotConstants.StatorCurrentLimit, (config, newValue) -> config.CurrentLimits.StatorCurrentLimit = newValue);
  // Sensor Feedback
  private DashboardNumber m_rotorToSensorRatio = m_talonTuner.tunable("RotorToSensorRatio", BasePivotConstants.RotorToSensorRatio,
      (config, newValue) -> config.Feedback.RotorToSensorRatio = newValue);
  private DashboardNumber m_sensorToMechRatio = m_talonTuner.tunable("SensorToMechanismRatio", BasePivotConstants.SensorToMechanismRatio,
      (config, newValue) -> config.Feedback.SensorToMechanismRatio = newValue);

  // Ramp rates
  private DashboardNumber m_rampPeriod = m_talonTuner.tunable("VoltageClosedLoopRampPeriod", BasePivotConstants.VoltageClosedLoopRampPeriod,
      (config, newValue) -> config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = newValue);

  /**
   * Creates a new BasePivot.
   *
   * @param armValuesSupplier the supplier of arm values
   */
  public BasePivot(Supplier<ArmValues> armValuesSupplier) {
    super(armValuesSupplier);

    m_canCoder.Configuration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    m_canCoder.Configuration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    m_canCoder.Configuration.MagnetSensor.MagnetOffset = Degrees.of(m_canCoderOffsetDegrees.get()).in(Rotations);
    m_canCoder.applyConfig();

    m_motor.Configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_motor.Configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    m_motor.Configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_motor.Configuration.CurrentLimits.SupplyCurrentLimit = m_supplyCurrentLimit.get();
    m_motor.Configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    m_motor.Configuration.CurrentLimits.StatorCurrentLimit = m_statorCurrentLimit.get();
    m_motor.Configuration.Feedback.FeedbackRemoteSensorID = CanIdentifiers.BasePivotCANcoderCANID;
    m_motor.Configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
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

    m_motor.attachMotorSim(m_motorSim, m_gearRatio, ChassisReference.Clockwise_Positive, true);
    m_motor.attachCanCoderSim(m_canCoder);
    
    // m_motor.setPosition(getCurrentAngle().getRotations());
  }

  /**
   * Sets the direct speed [-1.0, 1.0] of the motors.
   */
  public void setSpeed(double speed) {
    if (getCurrentAngle().in(Degrees) > BasePivotConstants.MaxAngle.in(Degrees)) {
      speed = Math.min(speed, 0.0);
    } else if (getCurrentAngle().in(Degrees) < BasePivotConstants.MinAngle.in(Degrees)) {
      speed = Math.max(speed, 0.0);
    }
    m_motor.set(speed);
  }

  /**
   * Sets the target angle and tries to drive there.
   */
  public void setTargetAngle(Angle newAngle) {
    setTargetAngle(newAngle, Optional.empty());
  }
  
  /**
  * Sets the target angle and tries to drive there.
  */
  public void setTargetAngle(Angle newAngle, LinearVelocity maxVelocity) {
    setTargetAngle(newAngle, Optional.of(maxVelocity));
  }

  /**
   * Sets the target angle and tries to drive there.
   */
  public void setTargetAngle(Angle newAngle, Optional<LinearVelocity> maxVelocity) {
    if (newAngle.in(Degrees) > BasePivotConstants.MaxAngle.in(Degrees)) {
      newAngle = BasePivotConstants.MaxAngle;
    } else if (newAngle.in(Degrees) < BasePivotConstants.MinAngle.in(Degrees)) {
      newAngle = BasePivotConstants.MinAngle;
    }

    boolean isExtenderAndGripperAtSafetyPose = getArmValues().isExtenderAtCloseLength && getArmValues().isGripperPivotAtCloseAngle;

    if (newAngle.in(Degrees) < BasePivotConstants.LowerSafetyAngle.in(Degrees) && !isExtenderAndGripperAtSafetyPose) {
      newAngle = getCurrentAngle().lt(BasePivotConstants.LowerSafetyAngle) ? getCurrentAngle() : BasePivotConstants.LowerSafetyAngle;
    }

    m_targetAngle = newAngle;
    if (maxVelocity.isPresent()) {
      m_motor.moveToPositionMotionMagic(newAngle.in(Rotations), maxVelocity.get().in(MetersPerSecond), m_mmAcceleration.get(), m_mmJerk.get());
    } else if (getArmValues().extenderLength > ExtenderConstants.BasePivotHighThresholdMeter) {
      m_motor.moveToPositionMotionMagic(newAngle.in(Rotations), m_mmCruiseVelocityHigh.get(), m_mmAccelerationHigh.get(), m_mmJerkHigh.get());
    } else {
      m_motor.moveToPositionMotionMagic(newAngle.in(Rotations)); // Rotation to match the cancoder units
    }
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

  /**
   * Checks if the current angle is close to goal angle.
   */
  public boolean atClose() {
    return Math.abs(getCurrentAngle().minus(m_targetAngle).in(Degrees)) < 1;
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
    super.periodic();
    Logger.recordOutput("BasePivot/Setpoint", m_targetAngle);
    Logger.recordOutput("BasePivot/CurrentAngle", getCurrentAngle().in(Degrees));
    Logger.recordOutput("BasePivot/AtTarget", atTarget());
    Logger.recordOutput("BasePivot/AngleError", getCurrentAngle().minus(m_targetAngle));
    Logger.recordOutput("BasePivot/Voltage", m_motor.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput("BasePivot/StatorCurrent", m_motor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("BasePivot/SupplyCurrent", m_motor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("BasePivot/MotorAngle", Rotations.of(m_motor.getPosition().getValueAsDouble()).in(Degrees));
  }
}
