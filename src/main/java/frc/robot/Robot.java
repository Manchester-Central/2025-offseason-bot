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

import com.chaos131.util.DashboardNumber;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Audio;
import frc.robot.subsystems.multisim.AdditionalSimRobot;
import frc.robot.util.LocalADStarAK;
import frc.robot.Constants.Mode;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.Arena2025Reefscape;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;

  private Timer m_timer = new Timer();

  enum game_state {
    ACTIVE,
    END_GAME,
    GAME_OVER,
    DISABLED
  };

  private game_state m_game_state = game_state.DISABLED;
  private Pose3d m_winner_animation_point = null;

  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.StartingMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
      
      default:
        throw new RuntimeException("Unknown StartingMode: " + Constants.StartingMode);
    }

    // Start AdvantageKit logger
    Logger.start();

    // Supplemental Logging
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Check for valid swerve config
    var modules =
        new SwerveModuleConstants[] {
          TunerConstants.FrontLeft,
          TunerConstants.FrontRight,
          TunerConstants.BackLeft,
          TunerConstants.BackRight
        };
    for (var constants : modules) {
      if (constants.DriveMotorType != DriveMotorArrangement.TalonFX_Integrated
          || constants.SteerMotorType != SteerMotorArrangement.TalonFX_Integrated) {
        throw new RuntimeException(
            "You are using an unsupported swerve configuration, which this template does not support without manual customization. The 2025 release of Phoenix supports some swerve configurations which were not available during 2025 beta testing, preventing any development and support from the AdvantageKit developers.");
      }
    }

    // Instantiate our primary RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = new RobotContainer(0);
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {

    DashboardNumber.checkAll();
    
    // Optionally switch the thread to high priority to improve loop
    // timing (see the template project documentation for details)
    // Threads.setCurrentThreadPriority(true, 99);

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Return to non-RT thread priority (do not modify the first argument)
    // Threads.setCurrentThreadPriority(false, 10);

    Logger.recordOutput("FieldSimulation/GameState", m_game_state.toString());

    // Logs the mech2d but in a way that AdvScope can use the 3d parts
    // robotContainer.logMech3d();
    robotContainer.periodic();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    m_timer.restart();
    m_game_state = game_state.DISABLED;
    // Audio.getInstance().playAudio(Audio.GAME_OVER);
    Audio.getInstance().stopAudio(Audio.START);
    Audio.getInstance().stopAudio(Audio.CRASH);
    Audio.getInstance().stopAudio(Audio.ENDGAME);
    Audio.getInstance().stopAudio(Audio.GAME_OVER);
    Audio.getInstance().stopAudio(Audio.WATER_SOUNDS);
    Audio.getInstance().stopAudio(Audio.WATER_ENDGAME);

    // Remove all game pieces from the arena to clear space for the robots
    SimulatedArena.getInstance().clearGamePieces();
    // Reset the primary driver
    robotContainer.resetRobotPose();
    // Reset the additional simulated robots
    for (AdditionalSimRobot simRobot : AdditionalSimRobot.instances) {
      simRobot.resetRobotPose();
    }
    // Reset game pieces last, in case they collide with a robot still at the spawn points
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_timer.restart();
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    m_timer.restart();
    m_game_state = game_state.ACTIVE;
    // Start audio for the match
    Audio.getInstance().playAudioFromStart(Audio.START, 1.0);
    Audio.getInstance().playAudioFromStart(Audio.WATER_SOUNDS, 0.5);

    robotContainer.setCurrentMode(Constants.StartingMode);
    for (AdditionalSimRobot simRobot : AdditionalSimRobot.instances) {
      simRobot.m_RobotContainer.setCurrentMode(Constants.StartingMode);
    }

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    var end_game_time = 60.0*2+9.0;
    var end_game_duration = 21.0;
    // Following is only done for simulation purposes
    if (robotContainer.getCurrentMode() == Constants.Mode.SIM) {
      // After 2min, play endgame audio
      if (m_game_state == game_state.ACTIVE && m_timer.get() >= end_game_time) {
        Audio.getInstance().playAudioFromStart(Audio.ENDGAME, 0.5);
        m_game_state = game_state.END_GAME;
      } else if (m_game_state == game_state.END_GAME) {
        var diff = m_timer.get()-end_game_time;
        var alpha = diff / end_game_duration;
        // 1-alpha means to start at max, and decrease over time
        Audio.getInstance().changeVolume(Audio.WATER_SOUNDS, 0.5*(1.0-alpha));
        Audio.getInstance().changeVolume(Audio.ENDGAME, 0.5*(1+alpha));

        if (alpha >= 1.0) {
          robotContainer.setCurrentMode(Mode.DISABLED);
          for (AdditionalSimRobot simRobot : AdditionalSimRobot.instances) {
            simRobot.m_RobotContainer.setCurrentMode(Mode.DISABLED);
          }
          m_game_state = game_state.GAME_OVER;
          Alliance game_winner = getWinner();
          Logger.recordOutput("FieldSimulation/game_winner", game_winner == null ? "Tie" : game_winner.toString());
          setWinnerAnimationPoint(game_winner);
        }
      }
    }

    if (m_game_state == game_state.GAME_OVER) {
      var num_algae = SimulatedArena.getInstance().getGamePiecesByType("Algae").size();
      // Audio.getInstance().playAudioFromStart(Audio.GAME_OVER, 1.0);
      if (m_timer.get() - (end_game_time + end_game_duration) > (num_algae-6)/1.5) {
        spawnWinnerBall();
      }
      if (m_timer.get() > end_game_time+end_game_duration+10) {
        // if we're past the game timer plus 10 seconds, move into disabled mode finally
        m_game_state = game_state.DISABLED;
      }
    }
  }

  private Alliance getWinner() {
    var arena = (Arena2025Reefscape)SimulatedArena.getInstance();
    if (arena.getScore(Alliance.Blue) > arena.getScore(Alliance.Red)) {
      return Alliance.Blue;
    } else if (arena.getScore(Alliance.Red) > arena.getScore(Alliance.Blue)) {
      return Alliance.Red;
    }
    return null;
  }

  private void setWinnerAnimationPoint(Alliance winningAlliance) {
    if (winningAlliance == Alliance.Blue) {
      m_winner_animation_point = new Pose3d(4.84505, 4.0259, 2.0, new Rotation3d());
    } else if (winningAlliance == Alliance.Red) {
      m_winner_animation_point =  new Pose3d(13.058902, 4.0259, 2.0, new Rotation3d());
    } else {
      m_winner_animation_point = null;
    }
  }

  private void spawnWinnerBall() {
    if (m_winner_animation_point == null) {
      System.out.println("Can't spawn ball without spawn location...");
      return;
    }

    // var arena = (Arena2025Reefscape)SimulatedArena.getInstance();
    System.out.println("Spawning ball in random direction.");
    SimulatedArena.getInstance().addGamePieceProjectile(new ReefscapeAlgaeOnFly(
            // Obtain robot position from drive simulation
            m_winner_animation_point.getTranslation().toTranslation2d(),
            // The scoring mechanism is installed at (0.46, 0) (meters) on the robot
            new Translation2d(0.1, 0.0),
            // Obtain robot speed from drive simulation
            new ChassisSpeeds(),
            // Obtain robot facing from drive simulation
            new Rotation2d(Math.random()*Math.PI*2),
            // The height at which the algae is ejected
            Meters.of(2.0),
            // The initial speed of the algae
            MetersPerSecond.of(Math.random()*3+2.0),
            // The algae is ejected at a 35-degree slope
            Degrees.of(Math.random()*Math.PI/2)));
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    m_timer.restart();
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    // Move robots into position.
    AdditionalSimRobot.setupAdditionalRobotSims();
    for (AdditionalSimRobot simRobot : AdditionalSimRobot.instances) {
      simRobot.resetRobotPose();
    }
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    SimulatedArena.getInstance().simulationPeriodic();
    // Updates the primary robot, AdditionalSimRobots will update themselves

    for (AdditionalSimRobot simRobot : AdditionalSimRobot.instances) {
      simRobot.periodic();
    }
    // Game Pieces on the ground
    Logger.recordOutput("FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput("FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
    // MapleSim hasn't figured out how to handle this correctly.
    // It should really be a ReefscapeArena that extends the SimulatedArena.
    // We would decide on the map at the Robot main/init level... but whatever, ya know?
    Logger.recordOutput("FieldSimulation/ScoredBlue", ((Arena2025Reefscape)SimulatedArena.getInstance()).getBranches(Alliance.Blue));
    Logger.recordOutput("FieldSimulation/ScoredRed", ((Arena2025Reefscape)SimulatedArena.getInstance()).getBranches(Alliance.Red));
    Logger.recordOutput("FieldSimulation/BluePoints", ((Arena2025Reefscape)SimulatedArena.getInstance()).getScore(Alliance.Blue));
    Logger.recordOutput("FieldSimulation/RedPoints", ((Arena2025Reefscape)SimulatedArena.getInstance()).getScore(Alliance.Red));
  }
}
