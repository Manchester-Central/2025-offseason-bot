// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import com.chaos131.util.DashboardNumber;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.ArmConstants.GripperPivotConstants;
import frc.robot.Constants.CanIdentifiers;
import frc.robot.Robot;
import frc.robot.SimConstants.SimGripperPivotConstants;
import frc.robot.subsystems.arm.Arm.ArmValues;
import frc.robot.utils.ChaosCanCoder;
import frc.robot.utils.ChaosCanCoderTuner;
import frc.robot.utils.ChaosTalonFx;
import frc.robot.utils.ChaosTalonFxTuner;
import frc.robot.utils.SafetyUtil.GripperPivotSafety;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class GripperPivot extends AbstractArmPart {
  private double m_simGearRatio = GripperPivotConstants.RotorToSensorRatio;
  private double m_jkgMetersSquared = 0.1;
  private Angle m_targetAngle = Degrees.of(0);
  private DCMotor m_dcMotor = DCMotor.getKrakenX60(1);
  private DCMotorSim m_motorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(m_dcMotor, m_jkgMetersSquared, m_simGearRatio),
          m_dcMotor,
          0.001,
          0.001);
  private ChaosTalonFx m_motor =
      new ChaosTalonFx(CanIdentifiers.GripperPivotMotorCANID);
  private ChaosCanCoder m_canCoder =
      new ChaosCanCoder(CanIdentifiers.GripperPivotCANCoderCANID);

  private ChaosTalonFxTuner m_tuner = new ChaosTalonFxTuner("GripperPivot", m_motor);
  private ChaosCanCoderTuner m_canCoderTuner = new ChaosCanCoderTuner("Gripper Pivot", m_canCoder);

  private DashboardNumber m_canCoderOffsetDegrees = m_canCoderTuner.tunable("CANCoder Tuner",
      Robot.isReal() ? GripperPivotConstants.canCoderOffsetDegrees : SimGripperPivotConstants.canCoderOffsetDegrees, (config, newValue) -> 
      config.MagnetSensor.MagnetOffset = Degrees.of(newValue).in(Rotations));

  // Motion Magic Slot 0 Configs
  private DashboardNumber m_kp = m_tuner.tunable("kP", Robot.isReal() ? GripperPivotConstants.kP : SimGripperPivotConstants.kP, (config, newValue) -> config.Slot0.kP = newValue);
  private DashboardNumber m_ki = m_tuner.tunable("kI", Robot.isReal() ? GripperPivotConstants.kI : SimGripperPivotConstants.kI, (config, newValue) -> config.Slot0.kI = newValue);
  private DashboardNumber m_kd = m_tuner.tunable("kD", Robot.isReal() ? GripperPivotConstants.kD : SimGripperPivotConstants.kD, (config, newValue) -> config.Slot0.kD = newValue);
  private DashboardNumber m_kg = m_tuner.tunable("kG", GripperPivotConstants.kG, (config, newValue) -> config.Slot0.kG = newValue);
  private DashboardNumber m_ks = m_tuner.tunable("kS", GripperPivotConstants.kS, (config, newValue) -> config.Slot0.kS = newValue);
  private DashboardNumber m_kv = m_tuner.tunable("kV", GripperPivotConstants.kV, (config, newValue) -> config.Slot0.kV = newValue);
  private DashboardNumber m_ka = m_tuner.tunable("kA", GripperPivotConstants.kA, (config, newValue) -> config.Slot0.kA = newValue);
  private DashboardNumber m_dynamicKg = m_tuner.tunable("dynamicKg", GripperPivotConstants.dynamicKg, (config, newValue) -> {});

  // Motion Magic Constraints
  private DashboardNumber m_mmCruiseVelocity = m_tuner.tunable(
      "MM_CruiseVelocity", GripperPivotConstants.MMCruiseVelocity, (config, newValue) -> config.MotionMagic.MotionMagicCruiseVelocity = newValue);
  private DashboardNumber m_mmAcceleration = m_tuner.tunable(
      "MM_Acceleration", GripperPivotConstants.MMAcceleration, (config, newValue) -> config.MotionMagic.MotionMagicAcceleration = newValue);
  private DashboardNumber m_mmJerk = m_tuner.tunable(
      "MM_Jerk", GripperPivotConstants.MMJerk, (config, newValue) -> config.MotionMagic.MotionMagicJerk = newValue);

  // Current limits
  private DashboardNumber m_supplyCurrentLimit = m_tuner.tunable(
      "SupplyCurrentLimit", GripperPivotConstants.SupplyCurrentLimit, (config, newValue) -> config.CurrentLimits.SupplyCurrentLimit = newValue);
  private DashboardNumber m_statorCurrentLimit = m_tuner.tunable(
      "StatorCurrentLimit", GripperPivotConstants.StatorCurrentLimit, (config, newValue) -> config.CurrentLimits.StatorCurrentLimit = newValue);

  // Sensor Feedback
  private DashboardNumber m_rotorToSensorRatio = m_tuner.tunable("RotorToSensorRatio", GripperPivotConstants.RotorToSensorRatio,
      (config, newValue) -> config.Feedback.RotorToSensorRatio = newValue);
  private DashboardNumber m_sensorToMechRatio = m_tuner.tunable("SensorToMechanismRatio", GripperPivotConstants.SensorToMechanismRatio,
      (config, newValue) -> config.Feedback.SensorToMechanismRatio = newValue);

  // Ramp rates
  private DashboardNumber m_rampPeriod = m_tuner.tunable("VoltageClosedLoopRampPeriod", GripperPivotConstants.VoltageClosedLoopRampPeriod,
      (config, newValue) -> config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = newValue);

  /**
   * Creates a new GripperPivot.
   *
   * @param armValuesSupplier the supplier of arm values
   */
  public GripperPivot(Supplier<ArmValues> armValuesSupplier) {
    super(armValuesSupplier);
    m_canCoder.Configuration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    m_canCoder.Configuration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    m_canCoder.Configuration.MagnetSensor.MagnetOffset = Degrees.of(m_canCoderOffsetDegrees.get()).in(Rotations);
    m_canCoder.applyConfig();

    m_motor.Configuration.ClosedLoopGeneral.ContinuousWrap = false;
    m_motor.Configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_motor.Configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    m_motor.Configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_motor.Configuration.CurrentLimits.SupplyCurrentLimit = m_supplyCurrentLimit.get();
    m_motor.Configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    m_motor.Configuration.CurrentLimits.StatorCurrentLimit = m_statorCurrentLimit.get();
    m_motor.Configuration.Feedback.FeedbackRemoteSensorID = CanIdentifiers.GripperPivotCANCoderCANID;
    m_motor.Configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    // m_motor.Configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    m_motor.Configuration.Feedback.RotorToSensorRatio = m_rotorToSensorRatio.get();
    m_motor.Configuration.Feedback.SensorToMechanismRatio = m_sensorToMechRatio.get();
    m_motor.Configuration.ClosedLoopRamps.VoltageClosedLoopRampPeriod = m_rampPeriod.get();
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
    m_motor.Configuration.Slot0 = slot0;

    m_motor.applyConfig();

    m_motor.attachMotorSim(m_motorSim, m_simGearRatio, ChassisReference.CounterClockwise_Positive, true);
    m_motor.attachCanCoderSim(m_canCoder);
  }

  /**
   * Sets the target angle and tries to drive there.
   */
  public void setTargetAngle(Angle newAngle) {
    GripperPivotSafety currentSafety = GripperPivotSafety.getGripperPivotSafety(Meters.of(getArmValues().extenderLength), GripperPivotConstants.Safeties);
    Angle currentMin = currentSafety.getMinAngle();
    Angle currentMax = currentSafety.getMaxAngle();

    if (newAngle.in(Degrees) > currentMax.in(Degrees)) {
      newAngle = Degrees.of(currentMax.in(Degrees));
    } else if (newAngle.in(Degrees) < currentMin.in(Degrees)) {
      newAngle = Degrees.of(currentMin.in(Degrees));
    }

    // if (!getArmValues().isBasePivotAtSafeAngle || !getArmValues().isExtenderAtSafeLength) {
    //   newAngle = ArmPoses.Stow.getGripperPivotAngle();
    // }
    m_targetAngle = newAngle;
    m_motor.moveToPositionMotionMagic(newAngle.in(Rotations), m_dynamicKg.get(), getCurrentGravityAngle());
  }
  
  /**
   * Disables the FusedCANCoder, this was used when the GripperPivot was breaking.
   */
  public void disableFuseCanCoder() {
    m_motor.Configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
    m_motor.applyConfig();
  }

  /**
   * Sets the direct speed [-1.0, 1.0] of the motors.
   */
  public void setSpeed(double speed) {
    // Rotation2d lowLevelMin = getArmValues().extenderLength < ExtenderConstants.BaseThresholdMeter ? GripperPivotConstants.MinAngleBase : GripperPivotConstants.MinAngleLow;
    // Rotation2d currentMax = getArmValues().extenderLength > ExtenderConstants.HighThresholdMeter ? GripperPivotConstants.MaxAngleHigh : GripperPivotConstants.MaxAngleStandard;
    // Rotation2d currentMin = getArmValues().extenderLength < ExtenderConstants.LowThresholdMeter ? lowLevelMin : GripperPivotConstants.MinAngleStandard;
    GripperPivotSafety currentSafety = GripperPivotSafety.getGripperPivotSafety(Meters.of(getArmValues().extenderLength), GripperPivotConstants.Safeties);
    Angle currentMin = currentSafety.getMinAngle();
    Angle currentMax = currentSafety.getMaxAngle();

    if (getCurrentAngle().in(Degrees) > currentMax.in(Degrees)) {
      speed = Math.min(speed, 0.0);
    } else if (getCurrentAngle().in(Degrees) < currentMin.in(Degrees)) {
      speed = Math.max(speed, 0.0);
    }
    m_motor.set(speed);
  }

  public double getSpeed() {
    return m_motor.getVelocity().getValueAsDouble();
  }

  public Angle getCurrentAngle() {
    return Rotations.of(
        m_canCoder.getAbsolutePosition().getValueAsDouble());
  }

  /** Gets the current angle of the gripper in parallel to the ground (when gravity effects it most). */
  public Angle getCurrentGravityAngle() {
    var basePivotAngle = getArmValues().basePivotAngle;
    var gripperPivotAngle = getCurrentAngle();
    return basePivotAngle.plus(gripperPivotAngle);
  }

  /**
   * Checks if the current angle is at the goal angle.
   */
  public boolean atTarget() {
    return Math.abs(getCurrentAngle().minus(m_targetAngle).in(Degrees)) < 1.0;
  }

  /**
   * Checks if the current angle close to the goal angle.
   */
  public boolean atClose() {
    return Math.abs(getCurrentAngle().minus(m_targetAngle).in(Degrees)) < 10.0;
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
    // TODO Auto-generated method stub
    super.periodic();
    Logger.recordOutput("GripperPivot/Setpoint", m_targetAngle);
    Logger.recordOutput("GripperPivot/CurrentAngle", getCurrentAngle().in(Degrees));
    Logger.recordOutput("GripperPivot/CurrentGravityAngle", getCurrentGravityAngle().in(Degrees));
    Logger.recordOutput("GripperPivot/AtTarget", atTarget());
    Logger.recordOutput("GripperPivot/AngleError", getCurrentAngle().minus(m_targetAngle));
    Logger.recordOutput("GripperPivot/Voltage", m_motor.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput("GripperPivot/StatorCurrent", m_motor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("GripperPivot/SupplyCurrent", m_motor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("GripperPivot/MotorAngle", Rotations.of(m_motor.getPosition().getValueAsDouble()).in(Degrees));
    Logger.recordOutput("GripperPivot/Erro", Rotations.of(m_motor.getPosition().getValueAsDouble()).minus(getCurrentAngle()).in(Degrees));  
    Logger.recordOutput("GripperPivot/MotorVelocityRPS", m_motor.getVelocity().getValueAsDouble());
  }
}
