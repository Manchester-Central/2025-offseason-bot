package frc.robot.subsystems.multisim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.RobotContainer;
import frc.robot.Constants.GeneralConstants;
import frc.robot.generated.TunerConstants;

import java.util.function.Supplier;
import org.ironmaple.utils.FieldMirroringUtils;

import com.chaos131.gamepads.Gamepad;

public class AdditionalSimRobot extends SubsystemBase {
  /* If an opponent robot is not on the field, it is placed in a off field position to reduce startup load. */
  public static final Pose2d[] WAITING_POSITIONS = new Pose2d[] {
    new Pose2d(0, 0, new Rotation2d()),
    new Pose2d(-2.0, 0, new Rotation2d()),
    new Pose2d(-3.5, 0, new Rotation2d()),
    new Pose2d(-5.0, 0, new Rotation2d()),
    new Pose2d(-6.5, 0, new Rotation2d()),
    new Pose2d(-8.0, 0, new Rotation2d())
  };

  public static final Pose2d[] ROBOTS_STARTING_POSITIONS = new Pose2d[] {
    GeneralConstants.DefaultStartingPose,
    // 2 allies
    new Pose2d(7.1, 4, Rotation2d.fromDegrees(180)),
    new Pose2d(7.1, 6, Rotation2d.fromDegrees(180)),
    // then 3 enemies
    new Pose2d(9.8, 2, new Rotation2d()),
    new Pose2d(9.8, 4, new Rotation2d()),
    new Pose2d(9.8, 6, new Rotation2d())
  };

  // you can create as many opponent robots as you need... and I need a full field.
  public static final AdditionalSimRobot[] instances = new AdditionalSimRobot[5];
  public static void setupAdditionalRobotSims() {
    int idx = 0;
    try {
      // Robot ID 0 is the main robot, so start counting at 1
      for (idx = 0; idx < instances.length; idx++) {
        System.out.println("Loading opponent robot "+idx+1+" simulation paths");
        instances[idx] = new AdditionalSimRobot(idx+1);
      }
    } catch (Exception e) {
      DriverStation.reportError("Failed to load opponent robot "+idx+1+" simulation paths, error: " + e.getMessage(), false);
    }
  }

  /**
   * Object specific member variables and functions
   */
  public final RobotContainer m_RobotContainer;
  private final Pose2d waitingPose;
  // private final Pose2d startingPose;
  private final int id;

  private AdditionalSimRobot(int id) {
    this.id = id;
    this.m_RobotContainer = new RobotContainer(id);
    // this.m_driver = new Gamepad(id);
    this.waitingPose = WAITING_POSITIONS[this.id];
    // this.startingPose = ROBOTS_STARTING_POSITIONS[this.id];
    // this.m_mechanismRoot2d = m_mechanism2d.getRoot("ArmRoot", -0.1, 0.4);
    // this.m_armLigament = m_mechanismRoot2d.append(new LoggedMechanismLigament2d("Arm", 0.6, 0));
    // this.m_arm = new Arm(m_armLigament);
    // this.m_gripper = new Gripper(null);
    // this.m_heldCoral = new Pose3d[0];
    // this.m_heldAlgae = new Pose3d[0];

    buildJoystickController(this.m_RobotContainer.getDriverGamepad());
    // Commented out because we won't be supporting auto-cycling opponent robots for now.
    // instances[idx].buildBehaviorChooser(
    //         PathPlannerPath.fromPathFile("opponent robot cycle path "+idx),
    //         Commands.none(),
    //         PathPlannerPath.fromPathFile("opponent robot cycle path "+idx+" backwards"),
    //         Commands.none(),
    //         new XboxController(id));


    // Setup control scheme
    // m_arm.setDefaultCommand(new RunCommand(() -> m_arm.setTargetAngle(m_gripper.hasCoral() ? ArmPoses.CoralGrippedPose.get() : ArmPoses.StowPose.get()), m_arm));
    // m_gripper.setDefaultCommand(new RunCommand(() -> m_gripper.setGripSpeed(GripperConstants.PassiveIntakeSpeed.get()), m_gripper));

    // // Enable slowmode based on stick press
    // m_driver.leftStick().onTrue(new InstantCommand(() -> m_currentSpeedModifier = m_currentSpeedModifier < 1.0 ? 1.0 : GeneralConstants.SlowModeModifier));
    // m_driver.rightStick().onTrue(new InstantCommand(() -> m_currentSpeedModifier = m_currentSpeedModifier < 1.0 ? 1.0 : GeneralConstants.SlowModeModifier));
    // // Lock to 0° when A button is held
    // m_driver
    //     .a()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //           this,
    //             () -> m_driver.getLeftY(),
    //             () -> -m_driver.getLeftX(),
    //             () -> DriveDirection.Away.getAllianceAngle(),
    //             () -> m_currentSpeedModifier));
    // // Switch to X pattern when X button is pressed
    // m_driver.x().onTrue(Commands.runOnce(this::setXMode, this));
    // // LB sets arm to HP intake, runs intake
    // m_driver.leftBumper().whileTrue(new RunCommand(() -> m_arm.setTargetAngle(ArmPoses.HPIntakePose.get()), m_arm)
    //   .alongWith(new RunCommand(() -> m_gripper.setGripSpeed(GripperConstants.ActiveIntakeSpeed.get()), m_gripper)));
    // // LT sets arm to floor intake, runs intake
    // m_driver.leftTrigger().whileTrue(new RunCommand(() -> m_arm.setTargetAngle(ArmPoses.FloorIntakePose.get()), m_arm)
    //   .alongWith(new RunCommand(() -> m_gripper.setGripSpeed(GripperConstants.ActiveIntakeSpeed.get()), m_gripper)));
    // // RB sets arm to scoring position
    // m_driver.rightBumper().whileTrue(new RunCommand(() -> m_arm.setTargetAngle(ArmPoses.ScoreLowPose.get()), m_arm));
    // // RT tells arm to score
    // m_driver.rightTrigger().whileTrue(new RunCommand(() -> m_gripper.setGripSpeed(GripperConstants.OuttakeSpeed.get()), m_gripper));
    // // Y autodrives to Reef position
    // m_driver.y().whileTrue(PathUtil.driveToClosestPointTeleopCommandV2(FieldPoint.getReefDrivePoses(), this));
    // // A autodrives to HP intake position
    // m_driver.a().whileTrue(PathUtil.driveToClosestPointTeleopCommandV2(FieldPoint.getHpDrivePoses(), this));
  }

  @Override
  public void periodic() {
    //
  }

  @Override
  public void simulationPeriodic() {
    m_RobotContainer.periodic();
  }

  // Collects the swerve pod locations from the auto generated TunerConstants
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
        new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
        new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
        new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
        new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
  }

  /**
   * Sets up a joystick controller for this opponent robot
   * @param joystick the xbox controller, already defined with a driver station slot ID
   */
  private void buildJoystickController(Gamepad joystick) {
    SendableChooser<Command> behaviorChooser = new SendableChooser<>();
    final Supplier<Command> disable =
        () -> Commands.runOnce(() -> m_RobotContainer.setRobotPose(waitingPose), this)
                      .andThen(Commands.runOnce(
                        () -> m_RobotContainer.getDriveSystem().runVelocity(new ChassisSpeeds())))
                      .ignoringDisable(true);
    // List of options
    behaviorChooser.setDefaultOption("Disable", disable.get());
    behaviorChooser.addOption("Joystick Drive", joystickDrive(joystick));

    // Setup functionality
    behaviorChooser.onChange((Command::schedule));
    RobotModeTriggers.teleop()
                     .onTrue(Commands.runOnce(
                        () -> behaviorChooser.getSelected().schedule()));
    RobotModeTriggers.disabled().onTrue(disable.get());
    SmartDashboard.putData("FieldSimulation/Robot"+this.id+"/Behavior", behaviorChooser);
  }

  // /**
  //  * DEPRECATED: We won't be supporting auto-cycling opponent robots for now.
  //  *
  //  * Build the behavior chooser of this opponent robot and send it to the dashboard,
  //  * cycle commands will repeat from the start when they expire.
  //  * @param segment0
  //  * @param toRunAtEndOfSegment0
  //  * @param segment1
  //  * @param toRunAtEndOfSegment1
  //  * @param joystick
  //  */
  // @Deprecated
  // public void buildBehaviorChooser(
  //       PathPlannerPath segment0,
  //       Command toRunAtEndOfSegment0,
  //       PathPlannerPath segment1,
  //       Command toRunAtEndOfSegment1,
  //       Gamepad joystick) {
  //   SendableChooser<Command> behaviorChooser = new SendableChooser<>();
  //   final Supplier<Command> disable =
  //       () -> Commands.runOnce(() -> swerveDriveSim.setSimulationWorldPose(waitingPose), this)
  //                     .andThen(Commands.runOnce(
  //                       () -> swerveDriveSim.setRobotSpeeds(new ChassisSpeeds())))
  //                         // () -> swerveDriveSim.runChassisSpeeds(new ChassisSpeeds(),
  //                         //                                       new Translation2d(),
  //                         //                                       false, false)))
  //                     .ignoringDisable(true);

  //   // Option to disable the robot
  //   behaviorChooser.setDefaultOption("Disable", disable.get());

  //   // Option to auto-cycle the robot
  //   behaviorChooser.addOption(
  //       "Auto Cycle", getAutoCycleCommand(segment0, toRunAtEndOfSegment0,
  //                                              segment1, toRunAtEndOfSegment1));

  //   // Option to manually control the robot with a joystick
  //   behaviorChooser.addOption("Joystick Drive", joystickDrive(joystick));

  //   // Schedule the command when another behavior is selected
  //   behaviorChooser.onChange((Command::schedule));

  //   // Schedule the selected command when teleop starts
  //   RobotModeTriggers.teleop()
  //                    .onTrue(Commands.runOnce(
  //                       () -> behaviorChooser.getSelected().schedule()));

  //   // Disable the robot when the user robot is disabled
  //   RobotModeTriggers.disabled().onTrue(disable.get());

  //   SmartDashboard.putData("AIRobotBehaviors/Opponent Robot " + id + " Behavior", behaviorChooser);
  // }

  // /**
  //  * DEPRECATED: We won't be supporting auto-cycling opponent robots for now.
  //  *
  //  * Get the command to auto-cycle the robot relatively */
  // @Deprecated
  // private Command getAutoCycleCommand(
  //       PathPlannerPath segment0,
  //       Command toRunAtEndOfSegment0,
  //       PathPlannerPath segment1,
  //       Command toRunAtEndOfSegment1) {
  //   final SequentialCommandGroup cycle = new SequentialCommandGroup();
  //   final Pose2d startingPose = new Pose2d(
  //           segment0.getStartingDifferentialPose().getTranslation(),
  //           segment0.getIdealStartingState().rotation());

  //   cycle.addCommands(simRobotFollowPath(segment0)
  //           .andThen(toRunAtEndOfSegment0)
  //           .withTimeout(10));

  //   cycle.addCommands(simRobotFollowPath(segment1)
  //           .andThen(toRunAtEndOfSegment1)
  //           .withTimeout(10));

  //   return cycle.repeatedly()
  //           .beforeStarting(Commands.runOnce(() -> swerveDriveSim.setSimulationWorldPose(
  //                   FieldMirroringUtils.toCurrentAlliancePose(startingPose))));
  // }

  // /**
  //  * DEPRECATED: We won't be supporting auto-cycling opponent robots for now.
  //  *
  //  * Sets up a sequential command group to follow a path, and then execute a command.
  //  * Takes an arbitrary number of pairs, and repeats the cycle when done.
  //  *
  //  * @param cmd list of pairs of actions
  //  * @return the compiled sequential command
  //  */
  // @Deprecated
  // public Command createPathSequence(@SuppressWarnings("unchecked") Pair<PathPlannerPath, Command>... cmd) {
  //   if (cmd.length == 0) throw new IllegalArgumentException();
  //   final SequentialCommandGroup cycle = new SequentialCommandGroup();

  //   for (var p : cmd) {
  //     cycle.addCommands(simRobotFollowPath(p.getFirst())
  //           .andThen(p.getSecond())
  //           .withTimeout(10));
  //   }

  //   return cycle.repeatedly()
  //           .beforeStarting(Commands.runOnce(() -> swerveDriveSim.setSimulationWorldPose(
  //           FieldMirroringUtils.toCurrentAlliancePose(ROBOTS_STARTING_POSITIONS[id]))));
  // }

  // /**
  //  * Follow path command for additional robots during simulated autonomous
  //  * @param path 
  //  * @return the command segment to be run
  //  */
  // private Command simRobotFollowPath(PathPlannerPath path) {
  //   return new FollowPathCommand(
  //     path, // Specify the path
  //     // Provide actual robot pose in simulation, bypassing odometry error
  //     () -> swerveDriveSim.getSimulatedDriveTrainPose(),
  //     // Provide actual robot speed in simulation, bypassing encoder measurement error
  //     () -> swerveDriveSim.getDriveTrainSimulatedChassisSpeedsRobotRelative(),
  //     // Chassis speeds output
  //     // (speeds, feedforwards) -> 
  //     //   swerveDriveSim.runChassisSpeeds(speeds, new Translation2d(), false, false),
  //     (speeds, _feedforwards) -> swerveDriveSim.setRobotSpeeds(speeds),
  //     driveController, // Specify PID controller
  //     PP_CONFIG,       // Specify robot configuration
  //     // Flip path based on alliance side
  //     () -> DriverStation.getAlliance()
  //         .orElse(DriverStation.Alliance.Blue)
  //         .equals(DriverStation.Alliance.Red),
  //     this // AIRobotInSimulation is a subsystem; this command should use it as a requirement
  //   );
  // }

  /**
   * Generates a player driver command for an additional robot
   *
   * @param joystick object associated with a specific controller ID
   * @return cmd to be run during teleop
   */
  private Command joystickDrive(Gamepad joystick) {
    // Obtain chassis speeds from joystick input
    final Supplier<ChassisSpeeds> joystickSpeeds = () -> new ChassisSpeeds(
      this.id < 3
      ? -joystick.getSlewLeftY() * m_RobotContainer.getDriveSystem().getMaxLinearSpeedMetersPerSec()
      : joystick.getSlewLeftY() * m_RobotContainer.getDriveSystem().getMaxLinearSpeedMetersPerSec(),
      -joystick.getSlewLeftX() * m_RobotContainer.getDriveSystem().getMaxLinearSpeedMetersPerSec(),
      -joystick.getSlewRightX() * m_RobotContainer.getDriveSystem().getMaxAngularSpeedRadPerSec());

    // Obtain driverstation facing for opponent driver station
    // final Supplier<Rotation2d> opponentDriverStationFacing = () ->
    //     FieldMirroringUtils.getCurrentAllianceDriverStationFacing()
    //                        .plus(Rotation2d.fromDegrees(180));
    // Overly fancy trick from IronMaple that amounts to "face away" and then
    // "add 180 degrees so you're actually facing me"

    return Commands.run(() -> {
        // Calculate field-centric speed from driverstation-centric speed
        final ChassisSpeeds fieldCentricSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                joystickSpeeds.get(),
                this.id < 3
                ? FieldMirroringUtils.getCurrentAllianceDriverStationFacing()
                                     .plus(Rotation2d.fromDegrees(180))
                : FieldMirroringUtils.getCurrentAllianceDriverStationFacing());
        // Run the field-centric speed
        m_RobotContainer.getDriveSystem().runVelocity(fieldCentricSpeeds);
      }, this)
      // Before the command starts, reset the robot to a position inside the field
      .beforeStarting(() -> m_RobotContainer.setRobotPose(
              FieldMirroringUtils.toCurrentAlliancePose(ROBOTS_STARTING_POSITIONS[id])));
  }


  // Helper functions to expose robot state to other utilities and functions.
  public Pose2d getPose() {
    return m_RobotContainer.getRobotPose();
  }

  public void resetRobotPose() {
    m_RobotContainer.setRobotPose(waitingPose);
    m_RobotContainer.getDriveSystem().runVelocity(new ChassisSpeeds());
  }
  
  public Rotation2d getRotation() {
    return m_RobotContainer.getRobotPose().getRotation();
  }

  public void runVelocity(ChassisSpeeds speeds) {
    m_RobotContainer.getDriveSystem().runVelocity(speeds);
  }

  public void setXMode() {
    // We can approximate the increased resistance with additional weight, possibly.
    // swerveDriveSim.setMass();
    m_RobotContainer.getDriveSystem().stopWithX();
  }
}
