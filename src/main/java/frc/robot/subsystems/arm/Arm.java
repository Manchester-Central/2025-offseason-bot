// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.chaos131.gamepads.Gamepad;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants.ArmPoses;
import frc.robot.Constants.ArmConstants.BasePivotConstants;
import frc.robot.Constants.ArmConstants.ExtenderConstants;
import frc.robot.Constants.ArmConstants.GripperConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Robot;
import frc.robot.subsystems.shared.StateBasedSubsystem;
import frc.robot.subsystems.shared.SubsystemState;

/** Add your docs here. */
public class Arm extends StateBasedSubsystem<Arm.ArmState> {
  /**
   * Possible values from the arm that can be used in arm parts and other areas
   * of the code
   * (without having to know about the base parts).
   */
  public class ArmValues {
    public Angle basePivotAngle;
    public Angle gripperPivotAngle;
    public double coralGripSpeed;
    public double climbSpeed;
    public double extenderLength;
    public boolean isBasePivotAtCloseAngle;
    public boolean isExtenderAtCloseLength;
    public boolean hasCoral;
    public boolean hasAlgae;
    public boolean hasCage;
    public boolean isGripperPivotAtCloseAngle;
  }

  /**
   * Gets the latest arm values.
   */
  public ArmValues getArmValues() {
    ArmValues values = new ArmValues();
    values.basePivotAngle = m_basePivot.getCurrentAngle();
    values.gripperPivotAngle = m_gripperPivot.getCurrentAngle();
    values.coralGripSpeed = m_gripper.getCoralGripSpeed();
    values.climbSpeed = m_climber.getClimbSpeed();
    values.extenderLength = m_extender.getCurrentLength();
    values.isBasePivotAtCloseAngle = m_basePivot.atClose();
    values.isExtenderAtCloseLength = m_extender.atClose();
    values.hasCoral = m_gripper.hasCoral();
    values.hasAlgae = m_gripper.hasAlgae();
    values.hasCage = m_climber.hasCage();
    values.isGripperPivotAtCloseAngle = m_gripperPivot.atClose();
    return values;
  }

  public BasePivot m_basePivot = new BasePivot(this::getArmValues); // TODO: UNDO public
  public Extender m_extender = new Extender(this::getArmValues); // TODO: UNDO public
  public Gripper m_gripper = new Gripper(this::getArmValues);
  public GripperPivot m_gripperPivot = new GripperPivot(this::getArmValues);
  private Climber m_climber = new Climber(this::getArmValues);
  private Gamepad m_operator;
  private Gamepad m_driver;
  private Trigger m_scoringTrigger;
  private SelectedAlgaeState m_selectedAlgaeState = SelectedAlgaeState.PROCESSOR;
  private SelectedCoralState m_selectedCoralState = SelectedCoralState.L4;
  private boolean m_isClimbMode = false;

  /**
   * The possible states of the Arm's state machine.
   */
  public enum ArmState implements SubsystemState {
    MANUAL,
    START,
    STOW,
    INTAKE_CORAL_FROM_FLOOR,
    INTAKE_ALGAE_FROM_FLOOR,
    INTAKE_FROM_HP, // Probably won't implement -Josh // nevermind -Josh
    PREP_L1,
    PREP_L2,
    PREP_L3,
    PREP_L4, // Might need many prep states
    PREP_PROCESSOR,
    PREP_BARGE,
    SCORE_L1,
    SCORE_L2,
    SCORE_L3,
    SCORE_L4,
    ALGAE_HIGH,
    ALGAE_LOW,
    HOLD_CORAL,
    HOLD_ALGAE,
    PREP_CLIMB,
    CLOSE_CLIMB,
    POST_CLIMB,
    ESCAPE_CLIMB,
    DOT,
    SCORE_SAFETY;
  }

  /** Creates a new Arm. */
  public Arm(Gamepad operator, Gamepad driver) {
    super(ArmState.START);
    m_operator = operator;
    m_driver = driver;
    m_scoringTrigger = operator.rightBumper().or(driver.rightTrigger());
  }

  private boolean isPoseReady() {
    return m_extender.atTarget() && m_basePivot.atTarget() && m_gripperPivot.atTarget();
  }

  private boolean isPoseClose() {
    return m_extender.atClose() && m_basePivot.atClose() && m_gripperPivot.atClose();
  }

  @Override
  protected void runStateMachine() {
    switch (getCurrentState()) {
      case MANUAL:
        manualState();
        break;
      case START:
        startState();
        break;
      default:
      case STOW:
        stowState();
        break;
      case INTAKE_CORAL_FROM_FLOOR:
        intakeCoralFromFloorState();
        break;
      case INTAKE_ALGAE_FROM_FLOOR:
        intakeAlgaeFromFloorState();
        break;
      case INTAKE_FROM_HP:
        intakeFromHpState();
        break;
      case PREP_L1:
        prepL1State();
        break;
      case PREP_L2:
        prepL2State();
        break;
      case PREP_L3:
        prepL3State();
        break;
      case PREP_L4:
        prepL4State();
        break;
      case PREP_PROCESSOR:
        prepProcessorState();
        break;
      case PREP_BARGE:
        prepBargeState();
        break;
      case SCORE_L1:
        scoreL1State();
        break;
      case SCORE_L2:
        scoreL2State();
        break;
      case SCORE_L3:
        scoreL3State();
        break;
      case SCORE_L4:
        scoreL4State();
        break;
      case ALGAE_HIGH:
        algaeHighState();
        break;
      case ALGAE_LOW:
        algaeLowState();
        break;
      case HOLD_CORAL:
        holdCoralState();
        break;
      case HOLD_ALGAE:
        holdAlgaeState();
        break;
      case PREP_CLIMB:
        prepClimb();
        break;
      case CLOSE_CLIMB:
        closeClimb();
        break;
      case POST_CLIMB:
        postClimb();
        break;
      case ESCAPE_CLIMB:
        escapeClimb();
        break;
      case DOT:
        dot();
        break;
      case SCORE_SAFETY:
        scoreSafety();
        break;
    }
  }

  private void startState() {
    if (ExtenderConstants.HasMagnetSensor && !m_extender.hasReachedMinimum()) {
      m_extender.setSpeed(-0.05); // Mr. Negative - Matt Bisson
      m_gripperPivot.setTargetAngle(ArmPoses.Stow.getGripperPivotAngle());
      return;
    }

    if (Robot.isSimulation()) {
      // changeState(ArmState.STOW);
      changeState(ArmState.STOW);
    } else {
      // changeState(ArmState.MANUAL);
      changeState(ArmState.STOW);
    }
  }

  private void manualState() {
    /*
     * m_basePivot.setSpeed(m_operator.getLeftY() * 0.131);
     * m_gripper.setCoralGripSpeed(0.0);
     * m_gripperPivot.setSpeed(0.0);
     * m_extender.setSpeed(m_operator.getRightY() * 0.5);
*/
    m_basePivot.setSpeed(m_operator.getRightY() * 0.131);

    m_extender.setSpeed(m_operator.getLeftY() * 0.5);

    // m_gripperPivot.setSpeed(m_operator.getLeftY() * 0.2);

    if (m_operator.leftBumper().getAsBoolean()) {
      m_gripperPivot.setSpeed(0.2);
    } else if (m_operator.leftTrigger().getAsBoolean()) {
      m_gripperPivot.setSpeed(-0.2);
    } else {
      m_gripperPivot.setSpeed(0);
    }

    if (m_operator.rightBumper().getAsBoolean()) {
      m_gripper.setCoralGripSpeed(0.2);
    } else if (m_operator.rightTrigger().getAsBoolean()) {
      m_gripper.setCoralGripSpeed(-0.2);
    } else {
      m_gripper.setCoralGripSpeed(0);
    }

  }

  private void stowState() {
    if (m_gripper.hasCoral()) {
      changeState(ArmState.HOLD_CORAL);
      return;
    }
    if (m_gripper.hasAlgae()) {
      changeState(ArmState.HOLD_ALGAE);
      return;
    }
    m_basePivot.setTargetAngle(ArmPoses.Stow.getBasePivotAngle());
    m_extender.setTargetLength(ArmPoses.Stow.getExtensionMeters());
    m_gripperPivot.setTargetAngle(ArmPoses.Stow.getGripperPivotAngle());

    if (m_scoringTrigger.getAsBoolean()) {
      m_gripper.setCoralGripSpeed(GripperConstants.OutakeCoralSpeed.get());
      m_gripper.setAlgaeGripSpeed(GripperConstants.OutakeCoralOnAlgaeMotorSpeed.get());
    } else {
      m_gripper.setCoralGripSpeed(0.0);
      m_gripper.setAlgaeGripSpeed(GripperConstants.HoldAlgaeSpeed.get());
    }
    m_climber.setClimbSpeed(0);
  }

  private void intakeCoralFromFloorState() {
    if (m_gripper.hasAlgae() && isStateStarting()) {
      m_gripper.setCoralGripSpeed(0.0);
      changeState(ArmState.HOLD_ALGAE);
      return;
    }

    if (m_scoringTrigger.getAsBoolean()) {
      m_gripper.setCoralGripSpeed(GripperConstants.OutakeCoralSpeed.get()); 
      m_gripper.setAlgaeGripSpeed(GripperConstants.OutakeCoralOnAlgaeMotorSpeed.get());
    } else if (!m_gripper.hasCoralFront()) {
      m_gripper.setCoralGripSpeed(GripperConstants.IntakeCoralSpeed.get()); 
      m_gripper.setAlgaeGripSpeed(GripperConstants.IntakeCoralOnAlgaeMotorSpeed.get());
    } else if (m_gripper.hasCoralFront() && !m_gripper.hasCoralBack()) {
      double currentElapsedSeconds = getElapsedStateSeconds();
      double currentElapsedSpitAlignSeconds = currentElapsedSeconds % GripperConstants.IntakeCoralPeriod.get();
      if (currentElapsedSpitAlignSeconds > GripperConstants.IntakeCoralSpitAlignSecondsThreshold.get()) {
        m_gripper.setCoralGripSpeed(GripperConstants.IntakeCoralSpitAlignSpeed.get());
        m_gripper.setAlgaeGripSpeed(GripperConstants.IntakeCoralOnAlgaeSpitAlignSpeed.get());
      } else {
        m_gripper.setCoralGripSpeed(GripperConstants.IntakeCoralSlow.get());
        m_gripper.setAlgaeGripSpeed(GripperConstants.IntakeCoralOnAlgaeSlowMotorSpeed.get());
      }
    } else {
      m_gripper.setCoralGripSpeed(0.0);
      m_gripper.setAlgaeGripSpeed(0.0);
      changeState(ArmState.HOLD_CORAL);
      return;
    }

    m_basePivot.setTargetAngle(ArmPoses.FloorIntakeCoral.getBasePivotAngle());
    m_extender.setTargetLength(ArmPoses.FloorIntakeCoral.getExtensionMeters());
    m_gripperPivot.setTargetAngle(ArmPoses.FloorIntakeCoral.getGripperPivotAngle());

    if (Robot.isSimulation() && getElapsedStateSeconds() > 2.0) {
      Gripper.hasCoralGrippedSim = true;
    }
    m_climber.setClimbSpeed(0);
  }

  private void intakeAlgaeFromFloorState() {
    if (m_scoringTrigger.getAsBoolean()) {
      m_gripper.setAlgaeGripSpeed(GripperConstants.OutakeAlgaeSpeed.get()); 
    } else if (!m_gripper.hasAlgae() && !m_gripper.hasCoral()) {
      m_gripper.setAlgaeGripSpeed(GripperConstants.IntakeAlgaeSpeed.get()); 
    } else {
      m_gripper.setAlgaeGripSpeed(GripperConstants.HoldAlgaeSpeed.get());
      // changeState(ArmState.HOLD_ALGAE);
      return;
    }

    m_basePivot.setTargetAngle(ArmPoses.FloorIntakeAlgae.getBasePivotAngle());
    m_extender.setTargetLength(ArmPoses.FloorIntakeAlgae.getExtensionMeters());
    m_gripperPivot.setTargetAngle(ArmPoses.FloorIntakeAlgae.getGripperPivotAngle());
    m_gripper.setCoralGripSpeed(0.0);

    if (Robot.isSimulation() && getElapsedStateSeconds() > 2.0) {
      Gripper.hasAlgaeGrippedSim = true;
    }
    m_climber.setClimbSpeed(0);
  }

  private void intakeFromHpState() {
    if (m_gripper.hasAlgae()) {
      m_gripper.setCoralGripSpeed(0.0);
      changeState(ArmState.HOLD_ALGAE);
      return;
    }

    if (!m_gripper.hasCoralFrontNoDebounce()) {
      m_gripper.setCoralGripSpeed(GripperConstants.HpIntakeCoralSpeed.get()); 
      m_gripper.setAlgaeGripSpeed(GripperConstants.HpIntakeCoralOnAlgaeMotorSpeed.get());
    } else if (m_gripper.hasCoralFrontNoDebounce() && !m_gripper.hasCoralBackNoDebounce()) {
      m_gripper.setCoralGripSpeed(GripperConstants.HpIntakeCoralSlowSpeed.get());
      m_gripper.setAlgaeGripSpeed(GripperConstants.HpIntakeCoralOnAlgaeSlowMotorSpeed.get());
    } else if (m_driver.a().getAsBoolean()) {
      m_gripper.setCoralGripSpeed(0.0);
      m_gripper.setAlgaeGripSpeed(0.0);
    } else {
      m_gripper.setCoralGripSpeed(0.0);
      m_gripper.setAlgaeGripSpeed(0.0);
      changeState(ArmState.HOLD_CORAL);
      return;
    }

    m_basePivot.setTargetAngle(ArmPoses.HpIntake.getBasePivotAngle());
    m_extender.setTargetLength(ArmPoses.HpIntake.getExtensionMeters());
    m_gripperPivot.setTargetAngle(ArmPoses.HpIntake.getGripperPivotAngle());

    if (Robot.isSimulation() && getElapsedStateSeconds() > 2.0) {
      Gripper.hasCoralGrippedSim = true;
    }
    m_climber.setClimbSpeed(0);
  }
  
  //👻👽😺
  private void prepL1State() {
    scoreHelperCoral(ArmPoses.ScoreL1, true, GripperConstants.OutakeCoralL1.get(), GripperConstants.OutakeCoralOnAlgaeMotorL1.get());
  }

  private void prepL2State() {
    scoreHelperCoral(ArmPoses.ScoreL2, true, GripperConstants.OutakeCoralSpeed.get(), GripperConstants.OutakeCoralOnAlgaeMotorSpeed.get());
  }

  private void prepL3State() {
    scoreHelperCoral(ArmPoses.ScoreL3, true, GripperConstants.OutakeCoralSpeed.get(), GripperConstants.OutakeCoralOnAlgaeMotorSpeed.get());
  }

  private void prepL4State() {
    scoreHelperCoral(ArmPoses.ScoreL4, true, GripperConstants.OutakeInvertedCoralSpeed.get(), GripperConstants.OutakeInvertedCoralOnAlgaeMotorSpeed.get());
  }

  private void prepProcessorState() {
    m_basePivot.setTargetAngle(ArmPoses.ScoreProcessor.getBasePivotAngle());
    m_extender.setTargetLength(ArmPoses.ScoreProcessor.getExtensionMeters());
    m_gripperPivot.setTargetAngle(ArmPoses.ScoreProcessor.getGripperPivotAngle());
    if (m_scoringTrigger.getAsBoolean()) {
      scoreAlgaeHelper();
    } else {
      m_gripper.setAlgaeGripSpeed(GripperConstants.HoldAlgaeSpeed.get());
      m_gripper.setCoralGripSpeed(0.0);
    }
    m_climber.setClimbSpeed(0);
  }

  private void prepBargeState() {
    m_basePivot.setTargetAngle(ArmPoses.ScoreBarge.getBasePivotAngle());
    m_extender.setTargetLength(ArmPoses.ScoreBarge.getExtensionMeters());
    m_gripperPivot.setTargetAngle(ArmPoses.ScoreBarge.getGripperPivotAngle());
    if (m_scoringTrigger.getAsBoolean()) {
      scoreAlgaeHelper();
    } else {
      m_gripper.setAlgaeGripSpeed(GripperConstants.HoldAlgaeSpeed.get());
      m_gripper.setCoralGripSpeed(0.0);
    }
    m_climber.setClimbSpeed(0);
  }

  private void scoreL1State() {
    scoreHelperCoral(ArmPoses.ScoreL1, false, GripperConstants.OutakeCoralL1.get(), GripperConstants.OutakeCoralOnAlgaeMotorL1.get());
  }

  private void scoreL2State() {
    scoreHelperCoral(ArmPoses.ScoreL2, false, GripperConstants.OutakeCoralSpeed.get(), GripperConstants.OutakeCoralOnAlgaeMotorSpeed.get());
  }

  private void scoreL3State() {
    scoreHelperCoral(ArmPoses.ScoreL3, false, GripperConstants.OutakeCoralSpeed.get(), GripperConstants.OutakeCoralOnAlgaeMotorSpeed.get());
  }

  private void scoreL4State() {
    scoreHelperCoral(ArmPoses.ScoreL4, false, GripperConstants.OutakeInvertedCoralSpeed.get(), GripperConstants.OutakeInvertedCoralOnAlgaeMotorSpeed.get());
  }

  private void algaeHighState() {
    if (!getArmValues().hasCoral) {
      m_basePivot.setTargetAngle(ArmPoses.AlgaeHigh.getBasePivotAngle());
      m_extender.setTargetLength(ArmPoses.AlgaeHigh.getExtensionMeters());
      m_gripperPivot.setTargetAngle(ArmPoses.AlgaeHigh.getGripperPivotAngle());
      m_gripper.setAlgaeGripSpeed(GripperConstants.IntakeAlgaeSpeed.get());
      m_gripper.setCoralGripSpeed(0.0);
      m_climber.setClimbSpeed(0);
    } else {
      changeState(ArmState.HOLD_CORAL);
      return;
    }
  }

  private void algaeLowState() {
    if (!getArmValues().hasCoral) {
      m_basePivot.setTargetAngle(ArmPoses.AlgaeLow.getBasePivotAngle());
      m_extender.setTargetLength(ArmPoses.AlgaeLow.getExtensionMeters());
      m_gripperPivot.setTargetAngle(ArmPoses.AlgaeLow.getGripperPivotAngle());
      m_gripper.setAlgaeGripSpeed(GripperConstants.IntakeAlgaeSpeed.get());
      m_gripper.setCoralGripSpeed(0.0);
      m_climber.setClimbSpeed(0);
    } else {
      changeState(ArmState.HOLD_CORAL);
      return;
    }
  }

  private void holdCoralState() {
    ArmPose holdPose = m_selectedCoralState.HoldPose;
    m_basePivot.setTargetAngle(holdPose.getBasePivotAngle());
    m_extender.setTargetLength(holdPose.getExtensionMeters());
    m_gripperPivot.setTargetAngle(holdPose.getGripperPivotAngle());
    m_climber.setClimbSpeed(0);
    if (m_scoringTrigger.getAsBoolean()) {
      m_gripper.setCoralGripSpeed(GripperConstants.OutakeCoralSpeed.get());
      m_gripper.setAlgaeGripSpeed(GripperConstants.OutakeCoralOnAlgaeMotorSpeed.get());
    } else {
      m_gripper.setCoralGripSpeed(0.0);
      m_gripper.setAlgaeGripSpeed(0.0);
    }
    if (!m_gripper.hasCoral()) {
      changeState(ArmState.STOW);
      return;
    }
  }

  private void holdAlgaeState() {
    m_basePivot.setTargetAngle(ArmPoses.HoldAlgae.getBasePivotAngle());
    m_extender.setTargetLength(ArmPoses.HoldAlgae.getExtensionMeters());
    m_gripperPivot.setTargetAngle(ArmPoses.HoldAlgae.getGripperPivotAngle());
    m_climber.setClimbSpeed(0);
    if (m_scoringTrigger.getAsBoolean()) {
      m_gripper.setAlgaeGripSpeed(GripperConstants.OutakeAlgaeSpeed.get());
    } else {
      m_gripper.setAlgaeGripSpeed(GripperConstants.HoldAlgaeSpeed.get());
    }
    m_gripper.setCoralGripSpeed(0.0);
    if (!m_gripper.hasAlgae()) {
      changeState(ArmState.STOW);
      return;
    }
  }

  private void scoreHelperCoral(ArmPose armPose, boolean isPrep, double outakeCoralSpeed, double outakeCoralOnAlgaeSpeed) {
    if (!m_gripper.hasCoral() && DriverStation.isAutonomous()) {
      changeState(ArmState.SCORE_SAFETY);
      return;
    }

    m_basePivot.setTargetAngle(armPose.getBasePivotAngle());
    m_extender.setTargetLength(armPose.getExtensionMeters());
    m_gripperPivot.setTargetAngle(armPose.getGripperPivotAngle());

    boolean isControllerScoring = m_scoringTrigger.getAsBoolean();
    boolean isAutonomousScoringTimedOut = DriverStation.isAutonomousEnabled() && isPoseClose() && m_stateTimer.hasElapsed(2);
    if ((!isPrep && isPoseReady()) || isControllerScoring || isAutonomousScoringTimedOut) {
      m_gripper.setCoralGripSpeed(outakeCoralSpeed);
      m_gripper.setAlgaeGripSpeed(outakeCoralOnAlgaeSpeed);
      if (Robot.isSimulation() && getElapsedStateSeconds() > 2.0) {
        Gripper.hasCoralGrippedSim = false;
      }
    } else {
      m_gripper.setCoralGripSpeed(0);
      m_gripper.setAlgaeGripSpeed(0.0);
    }
    m_climber.setClimbSpeed(0);
  }

  private void scoreAlgaeHelper() {
    m_climber.setClimbSpeed(0);
    m_gripper.setAlgaeGripSpeed(GripperConstants.OutakeAlgaeSpeed.get());
    m_gripper.setCoralGripSpeed(0.0);
    if (Robot.isSimulation() && getElapsedStateSeconds() > 2.0) {
      Gripper.hasAlgaeGrippedSim = false;
    }
  }

  private void prepClimb() {
    m_isClimbMode = true;
    m_basePivot.setTargetAngle(ArmPoses.ClimbPrep.getBasePivotAngle());
    m_gripperPivot.setTargetAngle(ArmPoses.ClimbPrep.getGripperPivotAngle());
    m_extender.setTargetLength(ArmPoses.ClimbPrep.getExtensionMeters());

    if (!getArmValues().hasCage) {
      m_climber.setClimbSpeed(ClimberConstants.IntakeCageSpeed.get());
    } else {
      m_climber.setClimbSpeed(0);
    }

    if (Robot.isSimulation() && getElapsedStateSeconds() > 2.0) {
      Climber.hasCageGrippedSim = true;
    }
    m_gripper.setAlgaeGripSpeed(0);
    m_gripper.setCoralGripSpeed(0);
  }

  private void closeClimb() {
    if (m_isClimbMode) {
      m_basePivot.setTargetAngle(ArmPoses.CloseClimb.getBasePivotAngle(), BasePivotConstants.MMClimbCruiseVelocity);
      m_gripperPivot.setTargetAngle(ArmPoses.CloseClimb.getGripperPivotAngle());
      m_extender.setTargetLength(ArmPoses.CloseClimb.getExtensionMeters());
      m_climber.setClimbSpeed(0);
      m_gripper.setAlgaeGripSpeed(0);
      m_gripper.setCoralGripSpeed(0);
    } else {
      changeState(ArmState.STOW);
    }
  }

  private void postClimb() {
    if (m_isClimbMode) {
      m_basePivot.setTargetAngle(ArmPoses.Climb.getBasePivotAngle(), BasePivotConstants.MMClimbCruiseVelocity);
      m_gripperPivot.setTargetAngle(ArmPoses.Climb.getGripperPivotAngle());
      m_extender.setTargetLength(ArmPoses.Climb.getExtensionMeters());
      m_climber.setClimbSpeed(0);
      m_gripper.setAlgaeGripSpeed(0);
      m_gripper.setCoralGripSpeed(0);
    } else {
      changeState(ArmState.STOW);
    }
  }

  private void escapeClimb() {
    m_isClimbMode = false;
    changeState(ArmState.STOW);
  }

  private void dot() {
    m_basePivot.setTargetAngle(ArmPoses.Dot.getBasePivotAngle());
    m_extender.setTargetLength(ArmPoses.Dot.getExtensionMeters());
    m_gripperPivot.setTargetAngle(ArmPoses.Dot.getGripperPivotAngle());
    m_gripper.setAlgaeGripSpeed(0);
    m_gripper.setCoralGripSpeed(0);
  }

  private void scoreSafety() {

    if (m_selectedCoralState == SelectedCoralState.L4 && m_extender.getCurrentLength() > ArmPoses.HoldCoralL4.getExtensionMeters() + 0.05) {

      m_basePivot.setTargetAngle(ArmPoses.HoldCoralL4.getBasePivotAngle());
      m_extender.setTargetLength(ArmPoses.HoldCoralL4.getExtensionMeters());
      m_gripperPivot.setTargetAngle(ArmPoses.HoldCoralL4.getGripperPivotAngle());
      return;

    } else {
      changeState(ArmState.STOW);
    }
  }

  /**
   * Set Motor to clean up. :3
   */
  public void setMotorCleanUp() {
    m_extender.setMotorCoast();
    // m_basePivot.setMotorCoast();
    // m_gripperPivot.setMotorCoast();
  }

  /**
   * Set Motor to start up. :3
   */
  public void setMotorStartUp() {
    m_extender.setMotorBrake();
    // m_basePivot.setMotorBrake();
    // m_gripperPivot.setMotorBrake();
  }

  @Override
  public void periodic() {
    super.periodic();
    if (DriverStation.isTeleopEnabled()) {
      switch (getCurrentState()) {
        case INTAKE_CORAL_FROM_FLOOR:
        case INTAKE_ALGAE_FROM_FLOOR:
        case ALGAE_HIGH:
        case ALGAE_LOW:
        case INTAKE_FROM_HP:
          m_driver.getHID().setRumble(RumbleType.kBothRumble, GeneralConstants.RumbleIntensity);
          break;
      
        default:
          m_driver.getHID().setRumble(RumbleType.kBothRumble, 0);
          break;
      }
    } else {
      m_driver.getHID().setRumble(RumbleType.kBothRumble, 0);
    }

    if (getCurrentState() == ArmState.PREP_CLIMB && !getArmValues().hasCage && DriverStation.isTeleopEnabled()) {
      m_operator.getHID().setRumble(RumbleType.kBothRumble, GeneralConstants.RumbleIntensity);
    } else {
      m_operator.getHID().setRumble(RumbleType.kBothRumble, 0);
    }

    Logger.recordOutput("CurrentState", getCurrentState().name());
  }

  public SelectedAlgaeState getSelectedAlgaeState() {
    return m_selectedAlgaeState;
  }

  public SelectedCoralState getSelectedCoralState() {
    return m_selectedCoralState;
  }

  public void setSelectedAlgaeState(SelectedAlgaeState selectedAlgaeState) {
    this.m_selectedAlgaeState = selectedAlgaeState;
  }

  public void setSelectedCoralState(SelectedCoralState selectedCoralState) {
    this.m_selectedCoralState = selectedCoralState;
  }

}
// RIP m_oldArm & m_oldGripper 2025-2025
