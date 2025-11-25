package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;

public abstract class DriveIO extends SubsystemBase {
  public final static double ROBOT_MASS_KG = 74.088;
  public final static double ROBOT_MOI = 6.883;
  public final static double WHEEL_COF = 1.2;

  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

  public final static RobotConfig PP_CONFIG =
    new RobotConfig(
        ROBOT_MASS_KG,
        ROBOT_MOI,
        new ModuleConfig(
            TunerConstants.FrontLeft.WheelRadius,
            TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
            WHEEL_COF,
            DCMotor.getKrakenX60Foc(1)
                .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
            TunerConstants.FrontLeft.SlipCurrent,
            1),
        DriveIO.getModuleTranslations());
  public final static PPHolonomicDriveController pp_drive_controller = new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0));

  public DriveIO() {
    super();
  }


  // Resets the pose to the original starting spot
  abstract public void resetPose();

  // Tells the robot to drive in a field relative manner
  abstract public void runVelocity(ChassisSpeeds speeds);
  // public void runVelocity(ChassisSpeeds speeds, DriveFeedforwards dff) {
  //   runVelocity(speeds);
  // }

  abstract public ChassisSpeeds getChassisSpeeds();

  // Sets the motion to 0 (some lingering movement may still occur)
  abstract public void stop();

  // Sets the wheels in an X formation to resist being pushed
  abstract public void stopWithX();

  // Returns the current pose of the robot
  abstract public Pose2d getPose();

  // Same as above, but just returns the rotation for readability
  abstract public Rotation2d getRotation();

  // Replaces the current pose with the given pose
  abstract public void setPose(Pose2d pose);

  // Adds a pose to the pose estimator, typically with a vision system
  abstract public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs);

  abstract public double getMaxLinearSpeedMetersPerSec();

  abstract public double getMaxAngularSpeedRadPerSec();

    /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
  }
}
