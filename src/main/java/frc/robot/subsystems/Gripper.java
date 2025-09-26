// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.subsystems.arm.Arm.ArmValues;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gripper extends SubsystemBase {
  /** Creates a new Gripper. */
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
}