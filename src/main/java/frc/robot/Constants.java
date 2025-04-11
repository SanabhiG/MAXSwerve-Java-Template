// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.Matrix;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 11;
    public static final int kRearLeftDrivingCanId = 13;
    public static final int kFrontRightDrivingCanId = 15;
    public static final int kRearRightDrivingCanId = 17;

    public static final int kFrontLeftTurningCanId = 10;
    public static final int kRearLeftTurningCanId = 12;
    public static final int kFrontRightTurningCanId = 14;
    public static final int kRearRightTurningCanId = 16;

    public static final boolean kGyroReversed = false;
    public static final SimpleMotorFeedforward kDriveFeedForward = null;
    public static final SimpleMotorFeedforward kTurningFeedForward = null;
    public static final Pose2d kInitialRedPose = null;
    public static final Pose2d kInitialBluePose = null;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion

    public static final double kTurningMotorReduction = 1;  //TUNE THIS
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class NeoVortexMotorConstants {
    public static final double kFreeSpeedRpm = 5676; //TUNE THIS
  }

  public static final class LimelightConstants {
    public static final Matrix<N3, N1> m_stateStdDevs =
        VecBuilder.fill(0.15, 0.15, 0.00001); // TODO: needs tuning
    public static final Matrix<N3, N1> m_visionStdDevs =
        VecBuilder.fill(0.00001, 0.00001, 999999); // TODO: needs
    // tuning
    public static final String kRightLimelightName = "limelight-right";
    public static final String kLeftLimelightName = "limelight-left";
    public static final String kBackLimelightName = "limelight-back";

    public static final double kRightCameraHeight = 19.942862; // 0.5065486948 meters
    public static final double kLeftCameraHeight = 19.942862; // 0.5065486948 meters

    // Parallel to elevator
    public static final double kRightCameraXOffset = 12.153079; // 0.3086882066 meters
    public static final double kLeftCameraXOffset = -12.153079; // -0.3086882066 meters
    public static final double kBackCameraXOffset = 0; // -0.3086882066 meters

    // Perpendicular to elevator
    public static final double kRightCameraYOffset = 11.940763; // 0.3032953802 meters
    public static final double kLeftCameraYOffset = 11.940763; // 0.3032953802 meters
    public static final double kBackCameraYOffset = -3.086657; // -0.3086882066 meters

    public static final double kBackCameraHeight = 38.868062; // TODO

    public static final double kRightMountingPitch = -45;
    public static final double kLeftMountingPitch = -45;

    public static final double kRightMountingYaw = -24.499987;
    public static final double kLeftMountingYaw = 180 - 24.499987;

    public static final double kBackMountingPitch = 20; // TODO

    public static final double kBackMountingYaw = 0; // TODO

    public static final double kReefTagHeight = 12;
    public static final double kProcessorTagHeight = 0; // tune later
    public static final double kCoralStationTagHeight = 53.25; // tune later
    public static final int kProcessorPipeline = 0; // TBD
    public static final int kRightReefBranchPipeline = 1;
    public static final int kLeftReefBranchPipeline = 2;
    public static final int kRedPosePipeline = 3; // TBD
    public static final int kBluePosePipeline = 4; // TBD
    public static final int kCoralStationPipeline = 5;

    public static final double kCoralStationDistanceThreshold = 0; // TODO: tune
  }

  public static final class LEDConstants {
    public static final int kBlinkinPort = 2;

    public static final double kBlue = 0.87;
    public static final double kGreen = 0.77;
    public static final double kWhite = 0.93;
    public static final double kYellow = 0.69;
    public static final double kHeartbeatRed = -0.25;
    public static final double kViolet = 0.91;
    public static final double kOrange = 0.65;
  }

}
