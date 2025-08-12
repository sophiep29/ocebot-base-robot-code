// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.studica.frc.AHRS;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.*;

@Logged
public class Drivetrain extends SubsystemBase {
  // Create MAXSwerveModules
  private final SwerveModule frontLeft =
      new SwerveModule(
          CANMappings.FRONT_LEFT_DRIVING,
          CANMappings.FRONT_LEFT_TURNING,
          DrivetrainConfig.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET);

  private final SwerveModule frontRight =
      new SwerveModule(
          CANMappings.FRONT_RIGHT_DRIVING,
          CANMappings.FRONT_RIGHT_TURNING,
          DrivetrainConfig.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET);

  private final SwerveModule rearLeft =
      new SwerveModule(
          CANMappings.REAR_LEFT_DRIVING,
          CANMappings.REAR_LEFT_TURNING,
          DrivetrainConfig.BACK_LEFT_CHASSIS_ANGULAR_OFFSET);

  private final SwerveModule rearRight =
      new SwerveModule(
          CANMappings.REAR_RIGHT_DRIVING,
          CANMappings.REAR_RIGHT_TURNING,
          DrivetrainConfig.BACK_RIGHT_CHASSIS_ANGULAR_OFFSET);

  // The gyro sensor
  @NotLogged private final AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

  // creates new field 2d to be shared with smart dashboard
  @NotLogged private final Field2d field = new Field2d();

  // Slew rate filter variables for controlling lateral acceleration
  @NotLogged
  private SlewRateLimiter magLimiter = new SlewRateLimiter(DrivetrainConfig.MAX_ACCELERATION);

  // slew rate filter variables for controlling rotation acceleration
  @NotLogged
  private SlewRateLimiter rotLimiter =
      new SlewRateLimiter(DrivetrainConfig.MAX_ROTATIONAL_ACCELERATION);

  // creates new robot chassis speeds
  @Logged private ChassisSpeeds desiredChassisSpeeds = new ChassisSpeeds();

  // creates new swerve drive pose estimator
  @NotLogged
  public SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(
          DrivetrainConfig.DRIVE_KINEMATICS,
          getHeading(),
          new SwerveModulePosition[] {
            this.frontLeft.getPosition(),
            this.frontRight.getPosition(),
            this.rearLeft.getPosition(),
            this.rearRight.getPosition()
          },
          new Pose2d(0, 0, getHeading()));

  public Drivetrain() {
    // sends field data to smart dashboard
    SmartDashboard.putData(field);
  }

  // Logs desired swerve module states
  @Logged(name = "desiredStates")
  public SwerveModuleState[] getDesiredStates() {
    return new SwerveModuleState[] {
      this.frontLeft.getDesiredState(),
      this.frontRight.getDesiredState(),
      this.rearLeft.getDesiredState(),
      this.rearRight.getDesiredState()
    };
  }

  // Logs actual swerve module states
  @Logged(name = "actualStates")
  public SwerveModuleState[] getActualStates() {
    return new SwerveModuleState[] {
      this.frontLeft.getState(),
      this.frontRight.getState(),
      this.rearLeft.getState(),
      this.rearRight.getState()
    };
  }

  // logs actual chassis speeds
  @Logged(name = "actualChassisSpeeds")
  public ChassisSpeeds getChassisSpeeds() {
    return DrivetrainConfig.DRIVE_KINEMATICS.toChassisSpeeds(
        new SwerveModuleState[] {
          this.frontLeft.getState(),
          this.frontRight.getState(),
          this.rearLeft.getState(),
          this.rearRight.getState()
        });
  }

  // sets desired chassis speeds given a speed and gives swerveModuleStates new module states
  public void setChassisSpeeds(ChassisSpeeds speeds) {
    desiredChassisSpeeds = speeds;
    var swerveModuleStates = DrivetrainConfig.DRIVE_KINEMATICS.toSwerveModuleStates(speeds);

    // renormalizes wheel speeds if any individual speed is above the specified maximum given the
    // current swerve module states and the absolute maximum speed the swerve modules can reach
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DrivetrainConfig.MAX_SPEED_METERS_PER_SECOND);

    // sets the desired states of the swerve modules
    this.frontLeft.setDesiredState(swerveModuleStates[0]);
    this.frontRight.setDesiredState(swerveModuleStates[1]);
    this.rearLeft.setDesiredState(swerveModuleStates[2]);
    this.rearRight.setDesiredState(swerveModuleStates[3]);
  }

  @Override
  public void periodic() {
    field.setRobotPose(
        this.poseEstimator.update(
            getHeading(),
            new SwerveModulePosition[] {
              this.frontLeft.getPosition(),
              this.frontRight.getPosition(),
              this.rearLeft.getPosition(),
              this.rearRight.getPosition()
            }));
  }

  // Returns the currently-estimated pose of the robot.
  @Logged
  public Pose2d getPose() {
    return this.poseEstimator.getEstimatedPosition();
  }

  // Initializes and logs targetAngle and lastDistance
  @Logged public Rotation2d targetAngle;
  @Logged public double lastDistance;

  /**
   * Method to drive the robot using joystick info. This method may not work as intended if the
   * joystick does not move in a circle
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   * @param rateLimit Whether to enable rate limiting for smoother control.
   */
  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    double xSpeedCommanded;
    double ySpeedCommanded;
    double rotationCommanded;

    if (rateLimit) {
      // Convert XY to polar(theta and magnitude) for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag =
          Math.min(
              Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2))
                  * DrivetrainConfig.MAX_SPEED_METERS_PER_SECOND,
              DrivetrainConfig.MAX_SPEED_METERS_PER_SECOND);

      inputTranslationMag = magLimiter.calculate(inputTranslationMag);

      xSpeedCommanded = inputTranslationMag * Math.cos(inputTranslationDir);
      ySpeedCommanded = inputTranslationMag * Math.sin(inputTranslationDir);
      rotationCommanded = rotLimiter.calculate(rot) * DrivetrainConfig.MAX_ANGULAR_SPEED;
    } else {
      xSpeedCommanded = xSpeed * DrivetrainConfig.MAX_SPEED_METERS_PER_SECOND;
      ySpeedCommanded = ySpeed * DrivetrainConfig.MAX_SPEED_METERS_PER_SECOND;
      rotationCommanded = rot * DrivetrainConfig.MAX_ANGULAR_SPEED;
    }

    this.setChassisSpeeds(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeedCommanded,
                ySpeedCommanded,
                rotationCommanded,
                getPose()
                    .getRotation()
                    .plus(
                        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                            ? Rotation2d.fromDegrees(0)
                            : Rotation2d.fromDegrees(180)))
            : new ChassisSpeeds(xSpeedCommanded, ySpeedCommanded, rotationCommanded));
  }

  // Sets the wheels into an X formation to prevent movement.
  public void setX() {
    this.frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    this.frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    this.rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    this.rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }
  // Returns the robot's heading in degrees, from -180 to 180
  private Rotation2d getHeading() {
    return Rotation2d.fromDegrees(
        this.gyro.getAngle() * (DrivetrainConfig.GYRO_IS_REVERSED ? -1.0 : 1.0));
  }
}
