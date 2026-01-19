package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

/** for swerve */
@Logged
public class Drivetrain extends SubsystemBase {

  @NotLogged
  SwerveModule frontLeft = new SwerveModule(frontLeftDriveCANId, frontLeftTurnCANId);
  @NotLogged
  SwerveModule frontRight = new SwerveModule(frontRightDriveCANId, frontRightTurnCANId);
  @NotLogged
  SwerveModule backLeft = new SwerveModule(backLeftDriveCANId, backLeftTurnCANId);
  @NotLogged
  SwerveModule backRight = new SwerveModule(backRightDriveCANId, backRightTurnCANId);

  boolean usingVision = true;

  PhotonCamera leftCam;
  PhotonCamera rightCam;

  boolean filterByDistanceFromOdometryPose = false;

  ArrayList<Pose2d> leftList;
  ArrayList<Pose2d> rightList;

  double lastLinearAccelX = 0;
  double lastLinearAccelY = 0;

  PhotonPoseEstimator rightEstimator;
  PhotonPoseEstimator leftEstimator;
  PhotonPipelineResult lastResult = new PhotonPipelineResult();

  double rightTagDistance = 0;
  double rightAmbiguity = 0;
  double rightBestID = 0;
  double rightOffsetDistance = 0;
  Pose3d rightPoseEstimate = new Pose3d();

  double leftTagDistance = 0;
  double leftAmbiguity = 0;
  double leftBestID = 0;
  double leftOffsetDistance = 0;
  Pose3d leftPoseEstimate = new Pose3d();

  Pose2d desiredPose = Pose2d.kZero;

  @NotLogged
  SwerveDrivePoseEstimator odometry;
  AHRS gyro;
  PIDController x, y, theta;
  String command = "None";
  Field2d field = new Field2d();
  SwerveSample trajectoryPose = new SwerveSample(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, new double[] { 0, 0, 0, 0 },
      new double[] { 0, 0, 0, 0 });

  @NotLogged private static Drivetrain singleton;

  private Drivetrain() {
    SmartDashboard.putData("field", field);
    gyro = new AHRS(NavXComType.kUSB1);
    odometry = new SwerveDrivePoseEstimator(
        kDriveKinematics,
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        },
        new Pose2d());
    odometry.setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, 15));

    x = new PIDController(kP, kI, kD);
    y = new PIDController(kP, kI, kD);
    theta = new PIDController(kPAngular, kIAngular, kDAngular);
    theta.enableContinuousInput(-Math.PI, Math.PI);

    if (usingVision) {
      setupVision();
    }    
  }

  public static Drivetrain getInstance() {
    if (singleton == null)
      singleton = new Drivetrain();
    return singleton;
  }

  @Override
  public void periodic() {
    odometry.update(
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        });
    field.setRobotPose(getPose());

    if (usingVision) {
      runVision();
    }
  }

  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }

  public void setCommand(String name) {
    command = name;
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        },
        pose);
  }

  public Rotation2d getFieldRelativeOffset() {
    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == Alliance.Blue) {
        return odometry.getEstimatedPosition().getRotation();
      } else {
        return odometry
            .getEstimatedPosition()
            .getRotation()
            .minus(Rotation2d.fromDegrees(180)); // DO NOT USE IF WE DONT RUN A PATH
      }
    } else {
      return odometry.getEstimatedPosition().getRotation();
    }
  }

  public void driveFromJoysticks(double xSpeed, double ySpeed, double rot) {

    var swerveModuleStates = kDriveKinematics.toSwerveModuleStates(
        ChassisSpeeds.discretize(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rot, getFieldRelativeOffset())
                : new ChassisSpeeds(xSpeed, ySpeed, rot),
            .02));
    setModuleStates(swerveModuleStates);
  }

  public void driveFieldRelative(double xSpeed, double ySpeed, double rot) {
    var swerveModuleStates = kDriveKinematics.toSwerveModuleStates(
        ChassisSpeeds.discretize(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rot, odometry.getEstimatedPosition().getRotation())
                : new ChassisSpeeds(xSpeed, ySpeed, rot),
            .02));
    setModuleStates(swerveModuleStates);
  }

  public void driveWithChassisSpeeds(ChassisSpeeds speeds) {
    var swerveModuleStates = kDriveKinematics.toSwerveModuleStates(ChassisSpeeds.discretize(speeds, .02));
    setModuleStates(swerveModuleStates);
  }

  public void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, maxVelocityMetersPerSec);
    frontLeft.setDesiredState(states[0]);
    frontRight.setDesiredState(states[1]);
    backLeft.setDesiredState(states[2]);
    backRight.setDesiredState(states[3]);
  }

  public void resetEncoders() {
    frontLeft.resetEncoders();
    frontRight.resetEncoders();
    backLeft.resetEncoders();
    backRight.resetEncoders();
  }

  @Logged
  public SwerveModuleState[] getSwerveModuleStates() {
    return new SwerveModuleState[] {
        frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState(),
    };
  }

  @Logged
  public SwerveModuleState[] getDesiredStates() {
    return new SwerveModuleState[] {
        frontLeft.getDesiredState(), frontRight.getDesiredState(), backLeft.getDesiredState(),
        backRight.getDesiredState()
    };
  }

  public SwerveModulePosition[] getSwerveModulePosition() {
    return new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition(),
    };
  }

  public void zeroHeading() {
    // gyro.reset();
    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == Alliance.Red) {
        // gyro.setAngleAdjustment(180);
        odometry.resetRotation(new Rotation2d(Math.PI));
      } else {
        odometry.resetRotation(new Rotation2d(0));
        // gyro.setAngleAdjustment(0);
      }
    }
  }

  public Command zeroHeadingCommand() {
    return Commands.runOnce(
        () -> {
          this.zeroHeading();
        });
  }

  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }

  public double getTurnRate() {
    return gyro.getRate();
  }

  public Command choreoCommand(
          Trajectory<SwerveSample> trajectory,
          BooleanSupplier isReversed) {

      var time = new Timer();

      return new FunctionalCommand(
         time::restart,
         () -> {// execute
            driveWithChassisSpeeds(choreoController(trajectory.sampleAt(time.get(), isReversed.getAsBoolean()).get()));
      }, 
      (interrupted) -> {//end
        time.stop();
        // if (interrupted) {
        //    driveWithChassisSpeeds(new ChassisSpeeds()); 
        // } else {
        //     driveWithChassisSpeeds((choreoController(trajectory.getFinalSample(isReversed.getAsBoolean()).get())));
        // }
        driveFromJoysticks(0, 0, 0);; 

        setCommand("None");
      }, 
      () -> {//isFinished
          var distanceFromGoal = getPose().relativeTo(trajectory.getFinalPose(isReversed.getAsBoolean()).get());
          //is this good?
          return (time.hasElapsed(trajectory.getTotalTime()) 
            && Math.abs(distanceFromGoal.getX()) < positionTolerance
            && Math.abs(distanceFromGoal.getY()) < positionTolerance
            && Math.abs(distanceFromGoal.getRotation().getDegrees()) < angleTolerance);
            // || (time.hasElapsed(0.08) && detectCrash.getAsBoolean() && hasCrashed());
      },
      this);
  }

  public void setDesiredPose(Pose2d pose){
    desiredPose = pose;
  }

  public boolean hasCrashed() {

    double linearAccelX = gyro.getWorldLinearAccelX();
    double jerkX = linearAccelX - lastLinearAccelX;
    lastLinearAccelX = linearAccelX;
    double linearAccelY = gyro.getWorldLinearAccelY();
    double jerkY = linearAccelY - lastLinearAccelY;
    lastLinearAccelY = linearAccelY;

    return Math.abs(jerkX) > 0.75 || Math.abs(jerkY) > 0.75;

  }

  public ChassisSpeeds choreoController(SwerveSample referenceState) {
    Pose2d currentPose = getPose();
    desiredPose = referenceState.getPose();
    double xFF = referenceState.vx;
    double yFF = referenceState.vy;
    double rotationFF = referenceState.omega;
    double xFeedback = x.calculate(currentPose.getX(), referenceState.x);
    double yFeedback = y.calculate(currentPose.getY(), referenceState.y);
    double rotationFeedback = theta.calculate(currentPose.getRotation().getRadians(), referenceState.heading);

    ChassisSpeeds retval = ChassisSpeeds.fromFieldRelativeSpeeds(
        xFF + xFeedback, yFF + yFeedback, rotationFF + rotationFeedback, currentPose.getRotation());
    return retval;
  }

  // =========VISION STUFF=========

  /**
   * run this whenever you want vision stuff, otherwise, leave it out
   */
  private void setupVision() {
    leftCam = new PhotonCamera("LeftCam");
    rightCam = new PhotonCamera("RightCam");

    rightEstimator = new PhotonPoseEstimator(
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark),
        PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        rightCameraLocation // TODO: Make real numbers
    );
    leftEstimator = new PhotonPoseEstimator(
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark),
        PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        leftCameraLocation // TODO: Make real numbers
    );
  }

  private void updatePoseEstimate(Optional<EstimatedRobotPose> estimate) {
    if (estimate.isPresent()) {
      odometry.addVisionMeasurement(estimate.get().estimatedPose.toPose2d(), estimate.get().timestampSeconds);
    }
  }

  @NotLogged
  public Transform3d getRightCamToTarget() {
    return rightCam.getLatestResult().getBestTarget().bestCameraToTarget;
  }

  @NotLogged
  public int getBestTagId(boolean rightSide) {
    if (rightSide) {
      return leftCam.getLatestResult().getBestTarget().fiducialId;
    } else {
      return rightCam.getLatestResult().getBestTarget().fiducialId;
    }
  }

  /**
   * 
   * 
   */
  @NotLogged
  public PhotonPipelineResult getLatestResult(boolean rightSideOfReef) {
    if (rightSideOfReef) {
      return leftCam.getLatestResult();
    } else {
      return rightCam.getLatestResult();
    }
  }

  /**
   * wrapper for running all periodic vision code
   */
  private void runVision() {
    // rightEstimator.addHeadingData(Timer.getFPGATimestamp(), odometry.getEstimatedPosition().getRotation());
    for (var result : rightCam.getAllUnreadResults()) {
      if (!result.hasTargets()) continue;
      // if best visible target is too far away for our liking, discard it, else use
      // it
      rightTagDistance = result.getBestTarget().bestCameraToTarget.getTranslation().getNorm();
      if (rightTagDistance > maxVisionDistanceTolerance)
        continue;
      // Check if 3D ambiguity is too high and skip if it is. May not be needed
      // because of chosen estimator strategy
      rightAmbiguity = result.getBestTarget().poseAmbiguity;
      if (rightAmbiguity > maxAmbiguity)
        continue;

      var em = rightEstimator.update(result);
      rightPoseEstimate = em.get().estimatedPose;
      // Check if estimate has us flying in the air and reject
      if (rightPoseEstimate.getZ() > zTolerance)
        continue;
      // Check if estimate says we're rolled and reject
      if (rightPoseEstimate.getRotation().getX() > rollPitchTolerance)
        continue;
      // Check if estimate says we're pitched and reject
      if (rightPoseEstimate.getRotation().getY() > rollPitchTolerance)
        continue;
      // Check if estimate is close enough to where we think we are
      rightOffsetDistance = rightPoseEstimate.toPose2d().getTranslation()
          .getDistance(odometry.getEstimatedPosition().getTranslation());
      if (rightOffsetDistance > visionPoseDiffTolerance)
        continue;
      // Measurement passed all filters, add to global pose estimate
      odometry.addVisionMeasurement(rightPoseEstimate.toPose2d(), em.get().timestampSeconds);
    }

    // leftEstimator.addHeadingData(Timer.getFPGATimestamp(), odometry.getEstimatedPosition().getRotation());
    for (var result : leftCam.getAllUnreadResults()) {
      if (!result.hasTargets()) continue;
      // if best visible target is too far away for our liking, discard it, else use
      // it
      leftTagDistance = result.getBestTarget().bestCameraToTarget.getTranslation().getNorm();
      if (leftTagDistance > maxVisionDistanceTolerance)
        continue;
      // Check if 3D ambiguity is too high and skip if it is. May not be needed
      // because of chosen estimator strategy
      leftAmbiguity = result.getBestTarget().poseAmbiguity;
      if (leftAmbiguity > maxAmbiguity)
        continue;

      var em = leftEstimator.update(result);
      leftPoseEstimate = em.get().estimatedPose;
      // Check if estimate has us flying in the air and reject
      if (leftPoseEstimate.getZ() > zTolerance)
        continue;
      // Check if estimate says we're rolled and reject
      if (leftPoseEstimate.getRotation().getX() > rollPitchTolerance)
        continue;
      // Check if estimate says we're pitched and reject
      if (leftPoseEstimate.getRotation().getY() > rollPitchTolerance)
        continue;
      // Check if estimate is close enough to where we think we are
      leftOffsetDistance = leftPoseEstimate.toPose2d().getTranslation()
          .getDistance(odometry.getEstimatedPosition().getTranslation());
      if (leftOffsetDistance > visionPoseDiffTolerance)
        continue;
      // Measurement passed all filters, add to global pose estimate
      odometry.addVisionMeasurement(leftPoseEstimate.toPose2d(), em.get().timestampSeconds);
    }
  }
}
