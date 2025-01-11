package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class Constants {

  public static final boolean usingThriftyEncoder = true;

  /* Voltage Compensation */
  public static final int voltageComp = 10;

  public static class CommandConstants {
    public static final double kXP = 0.1;
    public static final double kXI = 0;
    public static final double kxD = 0;

    public static final double kYP = 0.1;
    public static final double kYI = 0;
    public static final double kYD = 0;

    public static final double kThetaP = 0.1;

    public static final double MaxVelocity = 3;
    public static final double MaxAcceleration = 3;

    public static final double xGoal = 0;
    public static final double yGoal = 0;

    public static final double errorTolerence = 0.3;
  }

  /** Constants related to operator controls. */
  public static class OperatorConstants {
    public static double stickDeadband = 0.1;

    public static int driverPort = 0;
    public static int operatorPort = 1;

    public static final double DirectionSlewRate = 1.2; // radians per second
    public static final double MagnitudeSlewRate =
        1.8; // 2.0; //1.8; // percent per second (1 = 100%)
    public static final double RotationalSlewRate =
        2.0; // 20.0; //2.0; // percent per second (1 = 100%)
  }

  /** Constants related to the robot modules. */
  public static class ModuleConstants {

    /* Front Left Module */
    public static final class ModFL {
      public static final String name = "Front Left Module1";
      public static final int moduleNumber = 3;
      public static final int driveMotorID = 7;
      public static final int angleMotorID = 8;
      public static final int encoderID = 1;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(124.2);
      // public static final Rotation2d angleOffset = Rotation2d.fromRadians(0); //
    }

    /* Front Right Module */
    public static final class ModFR {
      public static final String name = "Front Right Module1";
      public static final int moduleNumber = 2;
      public static final int driveMotorID = 5;
      public static final int angleMotorID = 6;
      public static final int encoderID = 2;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-123.1);
      // public static final Rotation2d angleOffset = Rotation2d.fromRadians(0); //
    }

    /* Back Left Module */
    public static final class ModBL {
      public static final String name = "Back Left Module1";
      public static final int moduleNumber = 1;
      public static final int driveMotorID = 4;
      public static final int angleMotorID = 3;
      public static final int encoderID = 3;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(70.1); //
      // public static final Rotation2d angleOffset = Rotation2d.fromRadians(0); //
    }

    /* Back Right Module */
    public static final class ModBR {
      public static final String name = "Back Right Module1";
      public static final int moduleNumber = 0;
      public static final int driveMotorID = 1;
      public static final int angleMotorID = 2;
      public static final int encoderID = 4;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(95.8); //
      // public static final Rotation2d angleOffset = Rotation2d.fromRadians(0); //
    }
  }

  /** Constants related to the drive system. */
  public static class DriveConstants {

    public static final double MinAngle = -180;
    public static final double MaxAngle = 180;

    public static final int GyroID = 0;
    public static final boolean GyroInvert = false;

    public static final double TrackWidth = Units.inchesToMeters(20.75);
    public static final double WheelBase = Units.inchesToMeters(20.75);
    public static final double WheelDiameter = Units.inchesToMeters(3.8);
    public static final double WheelCircumference = WheelDiameter * Math.PI;

    public static final double CenterToWheel =
        Math.sqrt(Math.pow(WheelBase / 2.0, 2) + Math.pow(TrackWidth / 2.0, 2));

    public static final Translation2d[] moduleTranslations =
        new Translation2d[] {
          new Translation2d(WheelBase / 2.0, TrackWidth / 2.0),
          new Translation2d(WheelBase / 2.0, -TrackWidth / 2.0),
          new Translation2d(-WheelBase / 2.0, TrackWidth / 2.0),
          new Translation2d(-WheelBase / 2.0, -TrackWidth / 2.0)
        };

    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(moduleTranslations);

    /* Conversion Factors */

    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double DriveFreeSpinRPS = NeoMotorConstants.FreeSpeedRPM / 60;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    // public static final double DriveMotorReduction = (45.0 * 17 * 50) /
    // (kDrivingMotorPinionTeeth * 15 * 27);
    public static final double DriveMotorReduction = 6.75 / 1.0;
    public static final double DriveFreeWheelSpeedRPS =
        (DriveFreeSpinRPS * WheelCircumference) / DriveMotorReduction;

    public static final double TurnMotorReduction =
        150.0 / 7.0 / 1.0; // ratio between internal relative encoder
    // and Through Bore (or Thrifty in our case)
    // absolute encoder - 150.0 / 7.0

    public static final double DriveConversionFactor =
        (WheelDiameter * Math.PI) / DriveMotorReduction;
    public static final double AngleConversionFactor = 360 / TurnMotorReduction; // (2 * Math.PI) /
    // TurnMotorReduction;

    public static final double DriveVelocityConversionFactor =
        ((WheelDiameter * Math.PI) / DriveMotorReduction) / 60.0; // meters per second, per RPM
    public static final double AngleVelocityConversionFactor =
        (2 * Math.PI) / TurnMotorReduction / 60;

    /* Speed */
    public static final double MaxSpeed = 4;
    public static final double MaxAngularSpeed = Math.PI;
    /* Feed Forward */
    public static final double DriveKS = 0;
    public static final double DriveKV = 0;
    public static final double DriveKA = 0;

    /* Drive PID */
    public static final double DriveP = 0.04;
    public static final double DriveI = 0.0;
    public static final double DriveD = 0.0;

    public static final double DriveF = 1 / DriveFreeWheelSpeedRPS;

    /* Turn PID */
    public static final double TurnP = 0.01;
    public static final double TurnI = 0.0;
    public static final double TurnD = 0.0;

    public static final double TurnF = 0.0;

    /* Drive Motor */
    public static final int DriveContinuousCurrentLimit = 40;
    public static final boolean DriveInvert = false;
    public static final IdleMode DriveIdleMode = IdleMode.kBrake;

    /* Angle Motor */
    public static final int AngleContinuousCurrentLimit = 20;
    public static final boolean AngleInvert = true;
    public static final IdleMode AngleIdleMode = IdleMode.kCoast;
  }

  public static class IntakeConstants {
    public static final int upIntakeID = 9;
    public static final int lowIntakeID = 10;

    public static final boolean lowInvert = true;
    public static final boolean upInvert = false;

    public static final double intakeSpeed = 0.2;
    public static final double feedSpeed = -0.6;
    public static final double ampSpeed = -0.25;
    public static final double extakeSpeed = -0.5;
    public static final double shootSpeed = -0.95;

    public static final int intakeBreakID = 1;

    public static final int shootRPM = 4400;

    public static final String doIntake = "INTAKE";
    public static final String doExtale = "EXTAKE";
    public static final String doShoot = "SHOOT";
    public static final String doNothing = "STOP";
    public static final String doAmp = "AMP";

    public static final int intakeAngle = 200;
  }

  public static class WristConstants {
    public static final int wristPort = 0;
    public static final double offset = 0.0;
    public static final int wristID = 15;
    public static final int wrist2ID = 13;

    public static final double wristP = 0.0065;
    public static final double wristI = 0.0;
    public static final double wristD = 0.0;

    public static final double STOW = 145;
    public static final double AMP = 139;
    public static final double PICKUP = 26; // Decrease Value from 46
    public static final double SUBWOOFER = 134;
    public static final double INTAKEGOOD = 90;
    public static final double PODIUM = 115;

    public static final boolean leftInvert = false;
    public static final boolean rightInvert = true;

    public static final double MaxRotateSpeed = -0.4; // increase speed value from -0.2
    public static final double MaxRotateUpSpeed = 0.5; // increase speed value from 0.3
  }

  public static class FeederConstants {
    public static final int topFeederID = 11;
    public static final int bottomFeederID = 12;
    public static final double feedSpeed = -0.8;
    public static final double ampSpeed = -0.5;

    public static final boolean topInvert = true;
    public static final boolean bottomInvert = false;

    public static final int feedBrakeID = 2;

    public static final double intakeSpeed = 0.05;
  }

  public static class ClimberConstants {
    public static final int leftMotorID = 16;
    public static final int rightMotorID = 17;

    public static final boolean leftInvert = false;
    public static final boolean rightInvert = false;

    public static final double climbSpeed = 0.8;
  }

  public static class Auto {

    public static final double MaxSpeed = 5;
    public static final double MaxAcceleration = 5;
    public static final double MaxAngularSpeed = Math.PI;
    public static final double MaxAngularAcceleration = Math.PI;

    public static final TrapezoidProfile.Constraints ThetaControllerContraints =
        new TrapezoidProfile.Constraints(MaxAngularSpeed, MaxAngularAcceleration);

    /* PID */
    public static final double AutoTurnP = 1.7;
    public static final double AutoTurnI = 0.0;
    public static final double AutoTurnD = 0.0;

    public static final double AutoDriveP = 1.05;
    public static final double AutoDriveI = 0.0;
    public static final double AutoDriveD = 0.0;
  }

  /** Constants related to vision processing. */
  public static final class VisionConstants {
    public static final double CENTER_SPEAKER_TOPTAG_HEIGHT = Units.inchesToMeters(60.375);
    public static final double SPEAKER_SCORE_HEIGHT = Units.inchesToMeters(78 + 4);
    public static final double WRIST_AXLE_HEIGHT = Units.inchesToMeters(11);

    public static final double FRONT_HEIGHT = Units.inchesToMeters(25);
    public static final double FRONT_FORBACK = Units.inchesToMeters(-4);
    public static final double FRONT_LEFTRIGHT = Units.inchesToMeters(-0.5);
    public static final double FRONT_PITCH = Units.degreesToRadians(0);
    public static final double FRONT_YAW = Units.degreesToRadians(0);
    public static final double FRONT_ROLL = Units.degreesToRadians(0);
    public static final String frontCamName = "FrontCamera";
    public static final Transform3d frontRobotToCam =
        new Transform3d(
            FRONT_FORBACK,
            FRONT_LEFTRIGHT,
            FRONT_HEIGHT,
            new Rotation3d(FRONT_ROLL, FRONT_PITCH, FRONT_YAW));

    public static final double LEFT_HEIGHT = Units.inchesToMeters(25);
    public static final double LEFT_FORBACK = Units.inchesToMeters(-7.5);
    public static final double LEFT_LEFTRIGHT = Units.inchesToMeters(-3);
    public static final double LEFT_PITCH = Units.degreesToRadians(0);
    public static final double LEFT_YAW = Units.degreesToRadians(-120);
    public static final double LEFT_ROLL = Units.degreesToRadians(0);
    public static final String leftCamName = "LeftCamera";
    public static final Transform3d leftRobotToCam =
        new Transform3d(
            LEFT_FORBACK,
            LEFT_LEFTRIGHT,
            LEFT_HEIGHT,
            new Rotation3d(LEFT_ROLL, LEFT_PITCH, LEFT_YAW));

    public static final double RIGHT_HEIGHT = Units.inchesToMeters(25);
    public static final double RIGHT_FORBACK = Units.inchesToMeters(-7.5);
    public static final double RIGHT_LEFTRIGHT = Units.inchesToMeters(1);
    public static final double RIGHT_PITCH = Units.degreesToRadians(0);
    public static final double RIGHT_YAW = Units.degreesToRadians(120);
    public static final double RIGHT_ROLL = Units.degreesToRadians(0);
    public static final String rightCamName = "RightCamera";
    public static final Transform3d rightRobotToCam =
        new Transform3d(
            RIGHT_FORBACK,
            RIGHT_LEFTRIGHT,
            RIGHT_HEIGHT,
            new Rotation3d(RIGHT_ROLL, RIGHT_PITCH, RIGHT_YAW));

    public enum AprilTagIDs {
      RedSpeakerCenter(4),
      RedSpeakerSide(3),
      BlueSpeakerCenter(7),
      BlueSpeakerSide(8),
      RedSourceDriverStationClose(9),
      RedSourceDriverStationFar(10),
      BlueSourceDriverStationClose(2),
      BlueSourceDriverStationFar(1),
      RedAmp(5),
      BlueAmp(6);

      private final int ID;

      AprilTagIDs(int ID) {
        this.ID = ID;
      }

      public int getID() {
        return ID;
      }
    }

    public static final double visionP = 0.0055;
    public static final double visionI = 0.0;
    public static final double visionD = 0.0;

    //

  }

  /** Constants related to the NeoMotor. */
  public static final class NeoMotorConstants {
    public static final double FreeSpeedRPM = 5676;
  }
}
