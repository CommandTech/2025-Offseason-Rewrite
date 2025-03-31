package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static final class DriveConstants {
        public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 1;
        public static final int FRONT_LEFT_TURN_MOTOR_ID = 2;
        public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 3;
        public static final int FRONT_RIGHT_TURN_MOTOR_ID = 4;
        public static final int BACK_LEFT_DRIVE_MOTOR_ID = 5;
        public static final int BACK_LEFT_TURN_MOTOR_ID = 6;
        public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 7;
        public static final int BACK_RIGHT_TURN_MOTOR_ID = 8;

        public static final int DRIVE_CURRENT_LIMIT = 50;
        public static final int TURN_CURRENT_LIMIT = 20;

        public static final double MAX_SPEED = Units.feetToMeters(9.824);
        public static final double MAX_ANGULAR_SPEED = Units.feetToMeters(Math.PI * Units.feetToMeters(1));
        
        public static final double TRACK_WIDTH = Units.inchesToMeters(25);
        public static final double WHEEL_BASE = Units.inchesToMeters(25);
  
        public static final Translation2d FRONT_LEFT_LOCATION = new Translation2d(TRACK_WIDTH/2, WHEEL_BASE/2);
        public static final Translation2d FRONT_RIGHT_LOCATION = new Translation2d(TRACK_WIDTH/2, -WHEEL_BASE/2);
        public static final Translation2d BACK_LEFT_LOCATION = new Translation2d(-TRACK_WIDTH/2, WHEEL_BASE/2);
        public static final Translation2d BACK_RIGHT_LOCATION = new Translation2d(-TRACK_WIDTH/2, -WHEEL_BASE/2);

        public static final double WHEEL_RADIUS = Units.inchesToMeters(0.0508);
        public static final int ENCODER_RESOLUTION = 4096;
        
        public static final double MODULE_MAX_ANGULAR_VELOCITY = MAX_ANGULAR_SPEED;
        public static final double MODULE_MAX_ANGULAR_ACCELERATION = 2 * Math.PI; // radians per second squared

        public static final int DRIVE_GEARING = 14;
        public static final double DRIVE_REDUCTION = (45.0 * 22) / (DRIVE_GEARING * 15);
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_RADIUS* 2 * Math.PI;

        public static final double NEO_FREE_SPEED = 5676;
        public static final double DRIVE_MOTOR_FREE_SPEED = NEO_FREE_SPEED / 60;
        public static final double DRIVE_WHEEL_FREE_SPEED = (DRIVE_MOTOR_FREE_SPEED * WHEEL_CIRCUMFERENCE)
            / DRIVE_REDUCTION;

        public static final double DRIVE_P = 1;
        public static final double DRIVE_I = 0;
        public static final double DRIVE_D = 0;
        public static final double DRIVE_FF = 1 / DRIVE_WHEEL_FREE_SPEED;
        
        public static final double TURN_P = 1;
        public static final double TURN_I = 0;
        public static final double TURN_D = 0;
        public static final double TURN_FF = 0.5;

        public static final SparkMaxConfig DRIVE_CONFIG = new SparkMaxConfig();
        public static final SparkMaxConfig TURN_CONFIG = new SparkMaxConfig();
        
        static {
            double drivingFactor = WHEEL_RADIUS* 2 * Math.PI
                    / DRIVE_REDUCTION;
            double turningFactor = 2 * Math.PI;

            DRIVE_CONFIG
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(DRIVE_CURRENT_LIMIT);
            DRIVE_CONFIG.encoder
                    .positionConversionFactor(drivingFactor)
                    .velocityConversionFactor(drivingFactor / 60.0);
            DRIVE_CONFIG.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pid(DRIVE_P, DRIVE_I, DRIVE_D)
                    .velocityFF(DRIVE_FF)
                    .outputRange(-1, 1);

            TURN_CONFIG
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(TURN_CURRENT_LIMIT);
            TURN_CONFIG.absoluteEncoder
                    .inverted(true)
                    .positionConversionFactor(turningFactor)
                    .velocityConversionFactor(turningFactor / 60.0);
            TURN_CONFIG.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    .pidf(TURN_P, TURN_I, TURN_D, TURN_FF)
                    .outputRange(-1, 1)
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, turningFactor);
        }

        public static final double FRONT_LEFT_ANGULAR_OFFSET = -Math.PI / 2;
        public static final double FRONT_RIGHT_ANGULAR_OFFSET = 0;
        public static final double BACK_LEFT_ANGULAR_OFFSET = Math.PI;
        public static final double BACK_RIGHT_ANGULAR_OFFSET = Math.PI / 2;

        public static final double POSE_TOLERANCE = Units.inchesToMeters(1);
    }

    public static final class VisionConstants {
        private static final double CAMERA_X_OFFSET = 0;
        private static final double CAMERA_Y_OFFSET = 0;
        private static final double CAMERA_Z_OFFSET = 0;

        private static final double CAMERA_ROLL = 0;
        private static final double CAMERA_PITCH = 0;
        private static final double CAMERA_YAW = 0;
        
        public static final Transform3d CAMERA_OFFSET = new Transform3d(CAMERA_X_OFFSET, 
                                                            CAMERA_Y_OFFSET, 
                                                            CAMERA_Z_OFFSET, 
                                                            new Rotation3d(
                                                                CAMERA_ROLL, 
                                                                CAMERA_PITCH, 
                                                                CAMERA_YAW));
    }

    public static final class GoalConstants {
        public static final Transform2d ROBOT_ALGAE_OFFSET = new Transform2d();

        // Starting at the side that points at the drive station and moving clockwise
        public static final int[] BLUE_TAGS = new int[]{18, 19, 20, 21, 22, 17};
        // Following the diagram in the game manual
        public static final Pose2d BLUE_REEF_A = new Pose2d(0, 0, new Rotation2d());
        public static final Pose2d BLUE_REEF_B = new Pose2d(1, 0, new Rotation2d());
        public static final Pose2d BLUE_REEF_C = new Pose2d(2, 0, new Rotation2d());
        public static final Pose2d BLUE_REEF_D = new Pose2d(3, 0, new Rotation2d());
        public static final Pose2d BLUE_REEF_E = new Pose2d(4, 0, new Rotation2d());
        public static final Pose2d BLUE_REEF_F = new Pose2d(5, 0, new Rotation2d());
        public static final Pose2d BLUE_REEF_G = new Pose2d(6, 0, new Rotation2d());
        public static final Pose2d BLUE_REEF_H = new Pose2d(7, 0, new Rotation2d());
        public static final Pose2d BLUE_REEF_I = new Pose2d(8, 0, new Rotation2d());
        public static final Pose2d BLUE_REEF_J = new Pose2d(9, 0, new Rotation2d());
        public static final Pose2d BLUE_REEF_K = new Pose2d(10, 0, new Rotation2d());
        public static final Pose2d BLUE_REEF_L = new Pose2d(11, 0, new Rotation2d());
        
        public static final Pose2d[] BLUE_BRANCHES = new Pose2d[]{BLUE_REEF_A, BLUE_REEF_B, BLUE_REEF_C, BLUE_REEF_D, BLUE_REEF_E, BLUE_REEF_F, BLUE_REEF_G, BLUE_REEF_H, BLUE_REEF_I, BLUE_REEF_J, BLUE_REEF_K, BLUE_REEF_L};

        // Starting at the side that points at the drive station and moving clockwise
        public static final int[] RED_TAGS = new int[]{7, 6, 11, 10, 9, 8};
        //Can probably do math to flip it around but for right now hard coded is fine
        public static final Pose2d RED_REEF_A = new Pose2d();
        public static final Pose2d RED_REEF_B = new Pose2d();
        public static final Pose2d RED_REEF_C = new Pose2d();
        public static final Pose2d RED_REEF_D = new Pose2d();
        public static final Pose2d RED_REEF_E = new Pose2d();
        public static final Pose2d RED_REEF_F = new Pose2d();
        public static final Pose2d RED_REEF_G = new Pose2d();
        public static final Pose2d RED_REEF_H = new Pose2d();
        public static final Pose2d RED_REEF_I = new Pose2d();
        public static final Pose2d RED_REEF_J = new Pose2d();
        public static final Pose2d RED_REEF_K = new Pose2d();
        public static final Pose2d RED_REEF_L = new Pose2d();

        public static final Pose2d[] RED_BRANCHES = new Pose2d[]{RED_REEF_A, RED_REEF_B, RED_REEF_C, RED_REEF_D, RED_REEF_E, RED_REEF_F, RED_REEF_G, RED_REEF_H, RED_REEF_I, RED_REEF_J, RED_REEF_K, RED_REEF_L};
    }

    public static final class ElevatorConstants {
        public static final int ELEVATOR_MOTOR_ID = 9;
        public static final int ELEVATOR_CURRENT_LIMIT = 30;

        public static final double ELEVATOR_MAX_HEIGHT = Units.inchesToMeters(80);
        public static final double ELEVATOR_MIN_HEIGHT = Units.inchesToMeters(0);
        public static final double ELEVATOR_STOW_HEIGHT = Units.inchesToMeters(1);

        public static final double ELEVATOR_L1_HEIGHT = Units.inchesToMeters(10);
        public static final double ELEVATOR_L2_HEIGHT = Units.inchesToMeters(20);
        public static final double ELEVATOR_L3_HEIGHT = Units.inchesToMeters(30);

        public static final double ELEVATOR_ALGAE_HIGH_HEIGHT = Units.inchesToMeters(40);
        public static final double ELEVATOR_ALGAE_LOW_HEIGHT = Units.inchesToMeters(50);

        public static final double ELEVATOR_HEIGHT_TOLERANCE = Units.inchesToMeters(1);

        //good thing to look at for tuning
        //https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-elevator.html
        public static final double ELEVATOR_MAX_SPEED = Units.inchesToMeters(1);
        public static final double ELEVATOR_MAX_ACCELERATION = Units.inchesToMeters(1);
        public static final double ELEVATOR_P = 1;
        public static final double ELEVATOR_I = 0;
        public static final double ELEVATOR_D = 0;
        public static final double ELEVATOR_FF = 0.5;
        public static final double ELEVATOR_G = 0;
        public static final double ELEVATOR_V = 0;
        public static final double ELEVATOR_A = 0;

        public static final double ELEVATOR_REDUCTION = 1;

        public static final SparkMaxConfig ELEVATOR_CONFIG = new SparkMaxConfig();
        static {
            double elevatorFactor = 2 * Math.PI / ELEVATOR_REDUCTION;
            ELEVATOR_CONFIG
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(ELEVATOR_CURRENT_LIMIT);
            ELEVATOR_CONFIG.encoder
                .positionConversionFactor(elevatorFactor)
                .velocityConversionFactor(elevatorFactor / 60.0);
            ELEVATOR_CONFIG.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(ELEVATOR_P, ELEVATOR_I, ELEVATOR_D)
                .velocityFF(ELEVATOR_FF)
                .outputRange(-1, 1);
        }
    }

    public static final class ClimberConstants {
        public static final int PH_CAN_ID = 1;
        public static final int COMP_START_PRESSURE = 100;
        public static final int COMP_STOP_PRESSURE = 120;
        public static final int HIGH_SIDE_PRESSURE_SENSOR_ID = 0;
    }

    public static final class AlgaeConstants {
        public static final int PH_CAN_ID = 1;
        public static final int ALGAE_STOPPER_RETRACT_CHANNEL = 4;
        public static final int ALGAE_STOPPER_EXTEND_CHANNEL = 5;

        public static final int TUSKS_MOTOR_ID = 10;
        public static final SparkMaxConfig ALGAE_CONFIG = new SparkMaxConfig();
        
        public static final double TUSKS_MAX_HEIGHT = 0.660;
        public static final double TUSKS_MIN_HEIGHT = 0.166;
        public static final double AT_TARGET_THRESHOLD = 0.05;
        public static final double HOME_POSITION = 0.660;
        public static final double LOWER_POSITION = 0.166;
        public static final double APPROACH_POSITION = 0.375;

        public static final double TUSKS_P = 1;
        public static final double TUSKS_I = 0;
        public static final double TUSKS_D = 0;

        static {
            // Configure Current Limit
            ALGAE_CONFIG.smartCurrentLimit(5);

            // Configure integrated Encoder
            ALGAE_CONFIG.encoder.positionConversionFactor(1).velocityConversionFactor(1);

            // Configure Closed Loop Controller and MaxMotion
            ALGAE_CONFIG
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(TUSKS_P)
                .i(TUSKS_I)
                .d(TUSKS_D)
                .outputRange(-1, 1);
            ALGAE_CONFIG
                .closedLoop
                .maxMotion
                .maxVelocity(4800)
                .maxAcceleration(4800)
                .allowedClosedLoopError(AT_TARGET_THRESHOLD);

            // Enable Brake Mode
            ALGAE_CONFIG.idleMode(IdleMode.kBrake);
            
            SoftLimitConfig softLimits = new SoftLimitConfig();
            softLimits
                .forwardSoftLimit(TUSKS_MAX_HEIGHT)
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimit(TUSKS_MIN_HEIGHT)
                .reverseSoftLimitEnabled(true);
            ALGAE_CONFIG.softLimit.apply(softLimits);
        }
    }

    public static final class CoralConstants {
        public static final int PH_CAN_ID = 1;
        public static final int CORAL_STOPPER_EXTEND_CHANNEL = 6;
        public static final int CORAL_STOPPER_RETRACT_CHANNEL = 7;
        public static final int CORAL_SENSOR_ANALOG_PORT = 0;
        public static final double CORAL_SENSE_THRESHOLD = 650;
        public static final double AUTO_CORAL_SENCE_DELAY = 3;


    }
}
