// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;

import com.chaos131.util.DashboardNumber;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.ArmConstants.GripperConstants;
import frc.robot.Constants.CanIdentifiers;
import frc.robot.Constants.IoPortsConstants;
import frc.robot.Robot;
import frc.robot.subsystems.arm.Arm.ArmValues;
import frc.robot.utils.ChaosTalonFx;
import frc.robot.utils.ChaosTalonFxTuner;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Gripper extends AbstractArmPart {
  public static boolean hasCoralGrippedSim = false;

  private static boolean m_hasCoralGrippedFront = false;

  private static boolean m_hasCoralGrippedBack = false;

  private static boolean m_hasCoralGrippedBoth = false;

  public static boolean hasAlgaeGrippedSim = false;

  private static boolean m_hasAlgaeGripped = false;


  private ChaosTalonFx m_coralMotor = new ChaosTalonFx(CanIdentifiers.GripperCoralMotorCANID);

  private DigitalInput m_coralSensorFront = new DigitalInput(IoPortsConstants.CoralChannelIDFront);

  private DigitalInput m_coralSensorBack = new DigitalInput(IoPortsConstants.CoralChannelIDBack);

  private Debouncer m_coralSensorDebouncerFront = new Debouncer(GripperConstants.CoralFrontDebounceSeconds, DebounceType.kBoth);

  private Debouncer m_coralSensorDebouncerBack = new Debouncer(GripperConstants.CoralBackDebounceSeconds, DebounceType.kFalling);

  private ChaosTalonFxTuner m_coralTuner = new ChaosTalonFxTuner("CoralGripper", m_coralMotor);

  private ChaosTalonFx m_algaeMotor = new ChaosTalonFx(CanIdentifiers.GripperAlgaeMotorCANID);

  private Debouncer m_algaeSensorDebouncer = new Debouncer(GripperConstants.AlgaeDropDebounceSeconds, DebounceType.kBoth);

  private ChaosTalonFxTuner m_algaeTuner = new ChaosTalonFxTuner("AlgaeGripper", m_algaeMotor);


  // Current limits
  private DashboardNumber m_algaeSupplyCurrentLimit = m_algaeTuner.tunable(
      "SupplyCurrentLimit", GripperConstants.AlgaeSupplyCurrentLimit.in(Amps), (config, newValue) -> config.CurrentLimits.SupplyCurrentLimit = newValue);
  private DashboardNumber m_algaeStatorCurrentLimit = m_algaeTuner.tunable(
      "StatorCurrentLimit", GripperConstants.AlgaeStatorCurrentLimit.in(Amps), (config, newValue) -> config.CurrentLimits.StatorCurrentLimit = newValue);

  private DashboardNumber m_coralSupplyCurrentLimit = m_coralTuner.tunable(
      "SupplyCurrentLimit", GripperConstants.CoralSupplyCurrentLimit.in(Amps), (config, newValue) -> config.CurrentLimits.SupplyCurrentLimit = newValue);
  private DashboardNumber m_coralStatorCurrentLimit = m_coralTuner.tunable(
      "StatorCurrentLimit", GripperConstants.CoralStatorCurrentLimit.in(Amps), (config, newValue) -> config.CurrentLimits.StatorCurrentLimit = newValue);
      

  /**
   * Creates a new Gripper.
   *
   * @param armValuesSupplier the supplier of arm values
   */
  public Gripper(Supplier<ArmValues> armValuesSupplier) {
    super(armValuesSupplier);
    m_algaeMotor.Configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    m_algaeMotor.Configuration.CurrentLimits.StatorCurrentLimit = m_algaeStatorCurrentLimit.get();
    m_algaeMotor.Configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_algaeMotor.Configuration.CurrentLimits.SupplyCurrentLimit = m_algaeSupplyCurrentLimit.get();
    m_algaeMotor.applyConfig();

    m_coralMotor.Configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    m_coralMotor.Configuration.CurrentLimits.StatorCurrentLimit = m_coralStatorCurrentLimit.get();
    m_coralMotor.Configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_coralMotor.Configuration.CurrentLimits.SupplyCurrentLimit = m_coralSupplyCurrentLimit.get();
    m_coralMotor.Configuration.CurrentLimits.SupplyCurrentLowerLimit = GripperConstants.CoralSupplyCurrentLowerLimit.in(Amps);
    m_coralMotor.Configuration.CurrentLimits.SupplyCurrentLowerTime = GripperConstants.CoralSupplyCurrentLowerTime.in(Seconds);
    m_coralMotor.Configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    m_coralMotor.Configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_coralMotor.applyConfig();
  }

  /**
   * Sets the speed [-1.0, 1.0] of the coral gripper.
   */
  public void setCoralGripSpeed(double newSpeed) {
    m_coralMotor.set(newSpeed);
  }

  public double getCoralGripSpeed() {
    return m_coralMotor.get();
  }

  /**
   * Sets the speed [-1.0, 1.0] of the algae gripper.
   */
  public void setAlgaeGripSpeed(double newSpeed) {
    m_algaeMotor.set(newSpeed);
  }

  public double getAlgaeGripSpeed() {
    return m_algaeMotor.get();
  }

  /**
   * Checks if there is a coral at the sensor.
   */
  public boolean hasCoral() {
    return m_hasCoralGrippedBoth;
  }

  /**
   * Checks if there is a coral at the sensor.
   */
  public boolean hasCoralBothNoDebounce() {
    return hasCoralFrontNoDebounce() || hasCoralBackNoDebounce();
  }

  /**
   * Checks if we have Algae.
   */
  public boolean hasAlgae() {
    return m_hasAlgaeGripped;
  }

  /**
   * checks if front sensor is triggered with no debounce.
   */
  public boolean hasCoralFrontNoDebounce() {
    return !m_coralSensorFront.get();
  }

  /**
   * checks if back sensor is triggeredd with no debounce.
   */
  public boolean hasCoralBackNoDebounce() {
    return !m_coralSensorBack.get();
  }
  
  /**
   * checks if front sensor is triggered.
   */
  public boolean hasCoralFront() {
    return m_hasCoralGrippedFront;
  }

  /**
   * checks if back sensor is triggered.
   */
  public boolean hasCoralBack() {
    return m_hasCoralGrippedBack;
  }

  @Override
  public void periodic() {
    super.periodic();
    m_hasCoralGrippedFront = m_coralSensorDebouncerFront.calculate(Robot.isSimulation() ? hasCoralGrippedSim : hasCoralFrontNoDebounce());
    m_hasCoralGrippedBack = m_coralSensorDebouncerBack.calculate(Robot.isSimulation() ? hasCoralGrippedSim : hasCoralBackNoDebounce());
    m_hasCoralGrippedBoth = m_hasCoralGrippedFront && m_hasCoralGrippedBack;
    boolean algaeCurrentLimitReached = m_algaeMotor.getStatorCurrent().getValue().gt(Amps.of(m_algaeStatorCurrentLimit.get() - 10.0));
    m_hasAlgaeGripped = m_algaeSensorDebouncer.calculate(Robot.isSimulation() ? hasAlgaeGrippedSim : algaeCurrentLimitReached);
    Logger.recordOutput("Gripper/HasCoral", hasCoral());
    Logger.recordOutput("Gripper/HasAlgae", hasAlgae());
    Logger.recordOutput("Gripper/CoralSensorFront", hasCoralFront());
    Logger.recordOutput("Gripper/CoralSensorBack", hasCoralBack());
  }
}
