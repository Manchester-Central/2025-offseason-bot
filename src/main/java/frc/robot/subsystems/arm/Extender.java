// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Meters;

import com.chaos131.util.DashboardNumber;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.ArmConstants.ExtenderConstants;
import frc.robot.Constants.ArmConstants.GripperPivotConstants;
import frc.robot.Constants.CanIdentifiers;
import frc.robot.Constants.IoPortsConstants;
import frc.robot.Robot;
import frc.robot.SimConstants.SimExtenderConstants;
import frc.robot.subsystems.arm.Arm.ArmValues;
import frc.robot.utils.ChaosTalonFx;
import frc.robot.utils.ChaosTalonFxTuner;
import frc.robot.utils.SafetyUtil.GripperPivotSafety;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Extender extends AbstractArmPart {
  private double m_targetLength = 1;
  private double m_gearRatio = ExtenderConstants.SensorToMechanismRatio;
  private double m_jkgMetersSquared = 0.1;
  private boolean m_hasReachedMinimum = false;
  private DCMotor m_dcMotor = DCMotor.getKrakenX60(1);
  private DCMotorSim m_motorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(m_dcMotor, m_jkgMetersSquared, m_gearRatio),
          m_dcMotor,
          0.001,
          0.001);
  private ChaosTalonFx m_motor1 = new ChaosTalonFx(CanIdentifiers.ExtenderMotorCANID);
  private ChaosTalonFxTuner m_tuner = new ChaosTalonFxTuner("Extender", m_motor1);
  private DigitalInput m_minimumSensor = new DigitalInput(IoPortsConstants.ExtenderMinimumChannelID);

  // Motion Magic Slot 0 Configs
  private DashboardNumber m_kp = m_tuner.tunable("kP", Robot.isReal() ? ExtenderConstants.kP : SimExtenderConstants.kP, (config, newValue) -> config.Slot0.kP = newValue);
  private DashboardNumber m_ki = m_tuner.tunable("kI", Robot.isReal() ? ExtenderConstants.kI : SimExtenderConstants.kI, (config, newValue) -> config.Slot0.kI = newValue);
  private DashboardNumber m_kd = m_tuner.tunable("kD", Robot.isReal() ? ExtenderConstants.kD : SimExtenderConstants.kD, (config, newValue) -> config.Slot0.kD = newValue);
  private DashboardNumber m_kg = m_tuner.tunable("kG", ExtenderConstants.kG, (config, newValue) -> config.Slot0.kG = newValue);
  private DashboardNumber m_ks = m_tuner.tunable("kS", ExtenderConstants.kS, (config, newValue) -> config.Slot0.kS = newValue);
  private DashboardNumber m_kv = m_tuner.tunable("kV", ExtenderConstants.kV, (config, newValue) -> config.Slot0.kV = newValue);
  private DashboardNumber m_ka = m_tuner.tunable("kA", ExtenderConstants.kA, (config, newValue) -> config.Slot0.kA = newValue);

  // Motion Magic Constraints
  private DashboardNumber m_mmCruiseVelocity = m_tuner.tunable("MM_CruiseVelocity", ExtenderConstants.MMCruiseVelocity, (config, newValue) -> config.MotionMagic.MotionMagicCruiseVelocity = newValue);
  private DashboardNumber m_mmAcceleration = m_tuner.tunable("MM_Acceleration", ExtenderConstants.MMAcceleration, (config, newValue) -> config.MotionMagic.MotionMagicAcceleration = newValue);
  private DashboardNumber m_mmJerk = m_tuner.tunable("MM_Jerk", ExtenderConstants.MMJerk, (config, newValue) -> config.MotionMagic.MotionMagicJerk = newValue);

  private DashboardNumber m_mmUpCruiseVelocity = m_tuner.tunable("MM_Up_CruiseVelocity", ExtenderConstants.MMUpCruiseVelocity, (config, newValue) -> {});
  private DashboardNumber m_mmUpAcceleration = m_tuner.tunable("MM_Up_Acceleration", ExtenderConstants.MMUpAcceleration, (config, newValue) -> {});
  private DashboardNumber m_mmUpJerk = m_tuner.tunable("MM_Up_Jerk", ExtenderConstants.MMUpJerk, (config, newValue) -> {});
  // Current limits
  private DashboardNumber m_supplyCurrentLimit = m_tuner.tunable("SupplyCurrentLimit", ExtenderConstants.SupplyCurrentLimit, (config, newValue) -> config.CurrentLimits.SupplyCurrentLimit = newValue);
  private DashboardNumber m_statorCurrentLimit = m_tuner.tunable("StatorCurrentLimit", ExtenderConstants.StatorCurrentLimit, (config, newValue) -> config.CurrentLimits.StatorCurrentLimit = newValue);

  // Sensor Feedback
  private DashboardNumber m_rotorToSensorRatio = m_tuner.tunable("RotorToSensorRatio", ExtenderConstants.RotorToSensorRatio,
      (config, newValue) -> config.Feedback.RotorToSensorRatio = newValue);
  private DashboardNumber m_sensorToMechRatio = m_tuner.tunable("SensorToMechanismRatio", ExtenderConstants.SensorToMechanismRatio,
      (config, newValue) -> config.Feedback.SensorToMechanismRatio = newValue);

  // Ramp rates
  private DashboardNumber m_rampPeriod = m_tuner.tunable("VoltageClosedLoopRampPeriod", ExtenderConstants.VoltageClosedLoopRampPeriod,
      (config, newValue) -> config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = newValue);


  /**
   * Creates a new Extender.
   *
   * @param armValuesSupplier the supplier of arm values
   */
  public Extender(Supplier<ArmValues> armValuesSupplier) {
    super(armValuesSupplier);

    m_motor1.Configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_motor1.Configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    m_motor1.Configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_motor1.Configuration.CurrentLimits.SupplyCurrentLimit = m_supplyCurrentLimit.get();
    m_motor1.Configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    m_motor1.Configuration.CurrentLimits.StatorCurrentLimit = m_statorCurrentLimit.get();
    m_motor1.Configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    m_motor1.Configuration.Feedback.RotorToSensorRatio = m_rotorToSensorRatio.get();
    m_motor1.Configuration.Feedback.SensorToMechanismRatio = m_sensorToMechRatio.get();
    m_motor1.Configuration.ClosedLoopRamps.VoltageClosedLoopRampPeriod = m_rampPeriod.get();
    m_motor1.Configuration.MotionMagic.MotionMagicCruiseVelocity = m_mmCruiseVelocity.get();
    m_motor1.Configuration.MotionMagic.MotionMagicAcceleration = m_mmAcceleration.get();
    m_motor1.Configuration.MotionMagic.MotionMagicJerk = m_mmJerk.get();

    var slot0 = new Slot0Configs();
    slot0.kP = m_kp.get();
    slot0.kI = m_ki.get();
    slot0.kD = m_kd.get();
    slot0.kG = m_kg.get();
    slot0.kS = m_ks.get();
    slot0.kV = m_kv.get();
    slot0.kA = m_ka.get();
    slot0.GravityType = GravityTypeValue.Elevator_Static;
    m_motor1.Configuration.Slot0 = slot0;

    m_motor1.applyConfig();

    m_motor1.attachMotorSim(m_motorSim, m_gearRatio, ChassisReference.CounterClockwise_Positive, true);
  }

  /**
   * Set extender motor to Coast. :3
   */ 
  public void setMotorCoast() {
    m_motor1.Configuration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    m_motor1.applyConfig();
  }

  /**
   * Set extender motor to Brake. :3
   */ 
  public void setMotorBrake() {
    m_motor1.Configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_motor1.applyConfig();
  }

  /**
   * Sets the target length for extension and tries to drive there.
   */
  public void setTargetLength(double newLength) {
    if (hasReachedMinimum()) {
      if (newLength > ExtenderConstants.MaxLengthMeter) {
        newLength = ExtenderConstants.MaxLengthMeter;
      } else if (newLength < ExtenderConstants.MinLengthMeter) {
        newLength = ExtenderConstants.MinLengthMeter;
      }

      // if (!getArmValues().isBasePivotAtSafeAngle) {
      //   newLength = ArmPoses.Stow.getExtensionMeters();
      // }

      if (newLength < getCurrentLength()) {
        GripperPivotSafety currentSafety = GripperPivotSafety.getGripperPivotSafety(Meters.of(getCurrentLength()), GripperPivotConstants.Safeties);
        GripperPivotSafety targetSafety = GripperPivotSafety.getGripperPivotSafety(Meters.of(newLength), GripperPivotConstants.Safeties);
        Angle currentGpAngle = getArmValues().gripperPivotAngle; // )
        if (currentGpAngle.gt(targetSafety.getMaxAngle()) || currentGpAngle.lt(targetSafety.getMinAngle())) {
          newLength = currentSafety.getDistanceLow().in(Meters);
        }
      }



      m_targetLength = newLength;
      if (newLength > getCurrentLength()) {
        m_motor1.moveToPositionMotionMagic(newLength, m_mmUpCruiseVelocity.get(), m_mmUpAcceleration.get(), m_mmUpJerk.get());
      } else {
        m_motor1.moveToPositionMotionMagic(newLength, m_mmCruiseVelocity.get(), m_mmAcceleration.get(), m_mmJerk.get());
      }
    }
  }

  /**
   * Sets the direct speed [-1.0, 1.0] of the system.
   */
  public void setSpeed(double speed) {
    if (getCurrentLength() > ExtenderConstants.MaxLengthMeter || !hasReachedMinimum()) {
      speed = Math.min(speed, 0.0);
    } else if (getCurrentLength() < ExtenderConstants.MinLengthMeter) {
      speed = Math.max(speed, 0.0);
    }
    m_motor1.set(speed);
  }

  public double getCurrentLength() {
    return m_motor1.getPosition().getValueAsDouble();
  }

  /**
   * Checks if the extender length is at the target length.
   */
  public boolean atTarget() {
    return Math.abs(getCurrentLength() - m_targetLength) < 0.01;
  }

  /**
   * Checks if the extender length is close to target length.
   */
  public boolean atClose() {
    return Math.abs(getCurrentLength() - m_targetLength) < 0.03;
  }
  
  /**
   * Checks if the extender is at the minimum value of the arm.
   */
  public boolean isAtMinimum() {
    return !m_minimumSensor.get();
  }

  /**
   * Checks if the extender has reached the minimum value at some point.
   */
  public boolean hasReachedMinimum() {
    if (Robot.isSimulation()) {
      return true;
    }
    return m_hasReachedMinimum;
  }

  @Override
  public void simulationPeriodic() {
    m_motor1.simUpdate();
    // m_motor2.simUpdate();
  }

  @Override
  public void periodic() {
    super.periodic();

    if ((isAtMinimum() || !ExtenderConstants.HasMagnetSensor) && !m_hasReachedMinimum) {
      m_hasReachedMinimum = true;
      m_motor1.setPosition(ExtenderConstants.MinLengthMeter);
    }

    Logger.recordOutput("Extender/Setpoint", m_targetLength);
    Logger.recordOutput("Extender/CurrentLength", getCurrentLength());
    Logger.recordOutput("Extender/AtTarget", atTarget());
    Logger.recordOutput("Extender/LengthError", getCurrentLength() - m_targetLength);
    Logger.recordOutput("Extender/Voltage", m_motor1.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput("Extender/StatorCurrent", m_motor1.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Extender/SupplyCurrent", m_motor1.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("Extender/IsAtMinimum", isAtMinimum());
    Logger.recordOutput("Extender/hasReachedMin", hasReachedMinimum());
  }
}
