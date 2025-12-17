// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.chaos131.gamepads.Gamepad;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.GripperConstants;
import frc.robot.Constants.Mode;
import frc.robot.Constants.RobotDimensions;
import frc.robot.Constants.ArmConstants.ArmPoses;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.HPIntakeCommand;
import frc.robot.commands.ScoreCommand;
import frc.robot.commands.ScorePrepCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Quest;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveSim;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.multisim.AdditionalSimRobot;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.littletonrobotics.junction.Logger;
import frc.robot.util.FieldPoint;
import frc.robot.util.PathUtil;
import frc.robot.util.DriveDirection;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private int m_id;
  // private boolean m_isAutoAlign = true;
  private double m_currentSpeedModifier = 1.0;
  private Mode m_currentMode = Constants.StartingMode;
  public Mode getCurrentMode() {
    return m_currentMode;
  }
  public void setCurrentMode(Mode mode) {
    m_currentMode = mode;
  }

  // Subsystems
  // private final Vision m_vision; // If we do use a limelight
  private final DriveIO m_swerveDrive;
  private final Gripper m_gripper;
  private final Arm m_arm;
  private IntakeSimulation m_intakeSim;

  // Controller
  private final Gamepad m_driver;

  // Mechanism2d Simulation Support
  private final LoggedMechanism2d m_mechanism2d;
  private final LoggedMechanismRoot2d m_mechanismRoot2d;
  private final LoggedMechanismLigament2d m_originToPivot;
  private final LoggedMechanismLigament2d m_armLigament;
  private final LoggedMechanismLigament2d m_gripperLigament;

  // Dashboard inputs
  private LoggedDashboardChooser<Command> m_autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(int controllerId) {
    m_id = controllerId;

    m_mechanism2d = new LoggedMechanism2d(0, 0);
    m_mechanismRoot2d = m_mechanism2d.getRoot("Robot"+controllerId+"ArmRoot", 0, 0);
    m_originToPivot = m_mechanismRoot2d.append(new LoggedMechanismLigament2d("Robot"+controllerId+"Supports", 0.4, 90));
    m_armLigament = m_originToPivot.append(new LoggedMechanismLigament2d("Robot"+controllerId+"Arm", 0.6, -90));
    m_gripperLigament = m_armLigament.append(new LoggedMechanismLigament2d("Robot"+controllerId+"Gripper", 0.2, 0));

    m_driver = new Gamepad(controllerId, 4.0, 4.0);
    m_arm = new Arm(m_armLigament, controllerId);
    m_gripper = new Gripper(m_gripperLigament);

    @SuppressWarnings("unused")
    FieldPoint _dummy = FieldPoint.ReefPose10;
    switch (m_currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        var drive_obj = 
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight),
              () -> GeneralConstants.DefaultStartingPose);
        Quest m_quest = new Quest(drive_obj);
        drive_obj.setQuest(m_quest);
        m_swerveDrive = drive_obj;
        m_autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        var drive = new DriveSim(() -> AdditionalSimRobot.ROBOTS_STARTING_POSITIONS[m_id]);
        m_intakeSim = IntakeSimulation.OverTheBumperIntake(
            // Specify the type of game pieces that the intake can collect
            "Coral",
            // Specify the drivetrain to which this intake is attached
            drive.swerveDriveSim,
            // Width of the intake
            RobotDimensions.SideSideLength,
            // The extension length of the intake beyond the robot's frame (when activated)
            Meters.of(0.2),
            // The intake is mounted on the back side of the chassis
            IntakeSimulation.IntakeSide.BACK,
            // The intake can hold up to 1 piece
            1);
        m_swerveDrive = drive;
        break;

      default:
        // Replayed robot, disable IO implementations
        m_swerveDrive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
              () -> GeneralConstants.DefaultStartingPose);
        break;
    }

    // Set up auto routines
    NamedCommands.registerCommand("HPIntake", new HPIntakeCommand(m_arm, m_gripper));
    NamedCommands.registerCommand("ScorePrep", new ScorePrepCommand(m_arm, m_gripper));
    NamedCommands.registerCommand("Score", new ScoreCommand(m_arm, m_gripper));
    NamedCommands.registerCommand("ReefAutoAlign", PathUtil.driveToClosestPointTeleopCommandV2(FieldPoint.getReefDrivePoses(), m_swerveDrive));

    // Set up SysId routines
    // m_autoChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(m_swerveDrive));
    // m_autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(m_swerveDrive));
    // m_autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     m_swerveDrive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // m_autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     m_swerveDrive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // m_autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", m_swerveDrive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // m_autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", m_swerveDrive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  public Gamepad getDriverGamepad() {
    return this.m_driver;
  }

  private Command getDefaultSwerveCommand() {
    return DriveCommands.joystickDrive(
      m_swerveDrive,
      () -> this.m_id < 3 ?  m_driver.getSlewLeftY() : -m_driver.getSlewLeftY(), // Multisim fix
      () -> this.m_id < 3 ? -m_driver.getSlewLeftX() :  m_driver.getSlewLeftX(),
      () -> -m_driver.getSlewRightX(),
      () -> m_currentSpeedModifier);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    m_swerveDrive.setDefaultCommand(getDefaultSwerveCommand());

    // Manual Arm Control
    m_arm.setDefaultCommand(new RunCommand(() -> {
      m_arm.setTargetAngle(m_gripper.hasCoral() ? ArmPoses.CoralGrippedPose.get() : ArmPoses.StowPose.get());
    }, m_arm));
    // m_arm.setDefaultCommand(new RunCommand(() -> m_arm.setSpeed(m_operator.getLeftY() * 0.5), m_arm));

    // Gripper Default
    m_gripper.setDefaultCommand(new RunCommand(() -> m_gripper.setGripSpeed(GripperConstants.PassiveIntakeSpeed.get()), m_gripper));

    // Lock to 0° when A button is held
    m_driver
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                m_swerveDrive,
                () -> m_driver.getSlewLeftY(),
                () -> -m_driver.getSlewLeftX(),
                () -> DriveDirection.Away.getAllianceAngle(),
                () -> m_currentSpeedModifier));

    // Switch to X pattern when X button is pressed
    m_driver.x().onTrue(Commands.runOnce(m_swerveDrive::stopWithX, m_swerveDrive));

    // Reset gyro to 0° when B button is pressed
    m_driver
        .povUp()
        .onTrue(
            Commands.runOnce(
                    () ->
                        m_swerveDrive.setPose(
                            new Pose2d(getRobotPose().getTranslation(), new Rotation2d())),
                    m_swerveDrive)
                .ignoringDisable(true));

    m_driver.leftBumper().whileTrue(new RunCommand(() -> m_arm.setTargetAngle(ArmPoses.HPIntakePose.get()), m_arm)
      .alongWith(new RunCommand(() -> m_gripper.setGripSpeed(GripperConstants.ActiveIntakeSpeed.get()), m_gripper)));
    m_driver.leftTrigger().whileTrue(new RunCommand(() -> m_arm.setTargetAngle(ArmPoses.FloorIntakePose.get()), m_arm)
      .alongWith(new RunCommand(() -> m_gripper.setGripSpeed(GripperConstants.ActiveIntakeSpeed.get()), m_gripper)));

    // m_driver.rightBumper().whileTrue(new RunCommand(() -> m_arm.setTargetAngle(ArmPoses.ScoreLowPose.get()), m_arm)
    // .alongWith (PathUtil.driveToClosestPointTeleopCommandV2(FieldPoint.getReefDrivePoses(), m_swerveDrive)));

    m_driver.rightBumper().whileTrue(new RunCommand(() -> {
      m_arm.setTargetAngle(ArmPoses.ScoreLowPose.get());
    }, m_arm));
    // m_driver.rightTrigger().whileTrue(new RunCommand(() -> m_gripper.setGripSpeed(GripperConstants.OuttakeSpeed.get()), m_gripper));
    m_driver.a().whileTrue(new RunCommand(() -> {
      m_gripper.setGripSpeed(GripperConstants.OuttakeSpeed.get());
      if (m_intakeSim != null && m_intakeSim.getGamePiecesAmount() > 0) {
        m_intakeSim.obtainGamePieceFromIntake();
        m_gripper.setCoralSim(false);
        var driveSimulation = ((DriveSim)m_swerveDrive).swerveDriveSim;
        SimulatedArena.getInstance().addGamePieceProjectile(new ReefscapeCoralOnFly(
            // Obtain robot position from drive simulation
            driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
            // The scoring mechanism is installed at (0.46, 0) (meters) on the robot
            new Translation2d(0.35, 0),
            // Obtain robot speed from drive simulation
            driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
            // Obtain robot facing from drive simulation
            driveSimulation.getSimulatedDriveTrainPose().getRotation(),
            // The height at which the coral is ejected
            Meters.of(1.28),
            // The initial speed of the coral
            MetersPerSecond.of(0),
            // The coral is ejected at a 35-degree slope
            Degrees.of(-35)));
      }
    }, m_gripper));

    // Disabled because PathPlanner can't handle multiple robots doing this at once right now
    // m_driver.y().whileTrue(PathUtil.driveToClosestPointTeleopCommandV2(FieldPoint.getReefDrivePoses(), m_swerveDrive));
    // m_driver.a().whileTrue(PathUtil.driveToClosestPointTeleopCommandV2(FieldPoint.getHpDrivePoses(), m_swerveDrive));

    m_driver.leftStick().onTrue(new InstantCommand(() -> m_currentSpeedModifier = m_currentSpeedModifier < 1.0 ? 1.0 : GeneralConstants.SlowModeModifier));
    m_driver.rightStick().onTrue(new InstantCommand(() -> m_currentSpeedModifier = m_currentSpeedModifier < 1.0 ? 1.0 : GeneralConstants.SlowModeModifier));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if (m_autoChooser == null) {
      return Commands.none();
    }

    return m_autoChooser.get();
  }

  // Returns a very mutable reference to a very sensitive subsystem.
  public DriveIO getDriveSystem() {
    return m_swerveDrive;
  }

  public Pose2d getRobotPose() {
    return m_swerveDrive.getPose();
  }

  public void setRobotPose(Pose2d setupPose) {
    m_swerveDrive.setPose(setupPose);
  }

  public void resetRobotPose() {
    m_swerveDrive.resetPose();
  }

  // NOTE: NOT CALLED AUTOMATICALLY
  public void periodic() {
    if (getCurrentMode() == Mode.DISABLED) {
      m_currentSpeedModifier = 0;
    } else {
      m_currentSpeedModifier = 1.0;
    }
    if (m_intakeSim != null) {
      // Controls two things:
      // 1) That the collision box exists
      // 2) Automatically picks up pieces when they enter that box
      // This means the intake should always be on when the arm is in position,
      // and off when it's anywhere else
      if (m_arm.atAngle(ArmPoses.FloorIntakePose.get())) {
        m_intakeSim.startIntake();
      } else {
        m_intakeSim.stopIntake();
      }

      // If we're capable of picking up a piece, tell the gripper we have it but don't take it from the intake yet
      // The shooting mechanism will take it from the intake when that happens.
      if (!m_gripper.hasCoral() && m_intakeSim.getGamePiecesAmount() > 0) {
        m_gripper.setCoralSim(true);
      }
      FieldSimLogging();
    }
  }

  public void FieldSimLogging() {
    // Basics
    Logger.recordOutput("FieldSimulation/Robot"+this.m_id+"/Pose", m_swerveDrive.getPose());
    Logger.recordOutput("FieldSimulation/Robot"+this.m_id+"/Arm", m_mechanism2d);

    // Robot specific parts
    Pose3d[] parts = {
      new Pose3d(new Translation3d(-0.083, 0, 0.406),
                 new Rotation3d(Degrees.of(0), m_arm.getCurrentAngle().times(-1), Degrees.of(0))),
    };
    // Note, the arm's angle is multiplied by -1 because RollPitchYaw uses slightly different directions
    // than many think are intuitive, +pitch is actually down because of the left facing +y direction
    Logger.recordOutput("FieldSimulation/Robot"+this.m_id+"/Mech3d", parts);

    // Coral held by the robot (shown as game piece), must be in field relative coordinate frame
    Pose3d[] held_coral_position = {};
    double arm_angle = m_arm.getCurrentAngle().in(Radians);
    if (m_gripper.hasCoral()) {
      held_coral_position = new Pose3d[]{
        // Insert Mech2d forward kinematics to get orientation, for now plot it at the origin
        new Pose3d(m_swerveDrive.getPose()).transformBy(new Transform3d(new Translation3d(Meters.of(Math.cos(arm_angle)*0.6), Meters.of(0), Meters.of(0.2+Math.sin(arm_angle)*0.6)),
                                                                        new Rotation3d(Degrees.of(0), Degrees.of(0), m_swerveDrive.getPose().getRotation().getMeasure().plus(Degrees.of(90)))))
      };
    }
    Logger.recordOutput("FieldSimulation/Robot"+this.m_id+"/heldCoralPosition", held_coral_position);
  }
}
