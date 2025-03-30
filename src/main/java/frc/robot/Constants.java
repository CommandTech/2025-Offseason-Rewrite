package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static final class DriveConstants {
        public static final int FRONT_LEFT_DRIVE_MOTOR = 1;
        public static final int FRONT_LEFT_TURN_MOTOR = 2;
        public static final int FRONT_RIGHT_DRIVE_MOTOR = 3;
        public static final int FRONT_RIGHT_TURN_MOTOR = 4;
        public static final int BACK_LEFT_DRIVE_MOTOR = 5;
        public static final int BACK_LEFT_TURN_MOTOR = 6;
        public static final int BACK_RIGHT_DRIVE_MOTOR = 7;
        public static final int BACK_RIGHT_TURN_MOTOR = 8;

        public static final int DRIVE_CURRENT_LIMIT = 50;
        public static final int TURN_CURRENT_LIMIT = 20;

        public static final double MAX_SPEED = 3.0; // 3 meters per second
        public static final double MAX_ANGULAR_SPEED = Math.PI; // 1/2 rotation per second
        
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
        public static final double DRIVE_FF = 1;
        
        public static final double TURN_P = 1;
        public static final double TURN_I = 0;
        public static final double TURN_D = 0;
        public static final double TURN_FF = 0.5;

        public static final SparkMaxConfig DRIVE_CONFIG = new SparkMaxConfig();
        public static final SparkMaxConfig TURN_CONFIG = new SparkMaxConfig();
        
        static {
            // Use module constants to calculate conversion factors and feed forward gain.
            double drivingFactor = WHEEL_RADIUS* 2 * Math.PI
                    / DRIVE_REDUCTION;
            double turningFactor = 2 * Math.PI;
            double drivingVelocityFeedForward = 1 / DRIVE_WHEEL_FREE_SPEED;

            DRIVE_CONFIG
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(DRIVE_CURRENT_LIMIT);
            DRIVE_CONFIG.encoder
                    .positionConversionFactor(drivingFactor) // meters
                    .velocityConversionFactor(drivingFactor / 60.0); // meters per second
            DRIVE_CONFIG.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pidf(DRIVE_P, DRIVE_I, DRIVE_D, DRIVE_FF)
                    .velocityFF(drivingVelocityFeedForward)
                    .outputRange(-1, 1);

            TURN_CONFIG
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(TURN_CURRENT_LIMIT);
            TURN_CONFIG.absoluteEncoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering motor in the MAXSwerve Module.
                    .inverted(true)
                    .positionConversionFactor(turningFactor) // radians
                    .velocityConversionFactor(turningFactor / 60.0); // radians per second
            TURN_CONFIG.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pidf(TURN_P, TURN_I, TURN_D, TURN_FF)
                    .outputRange(-1, 1)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, turningFactor);
        }

        public static final double FRONT_LEFT_ANGULAR_OFFSET = -Math.PI / 2;
        public static final double FRONT_RIGHT_ANGULAR_OFFSET = 0;
        public static final double BACK_LEFT_ANGULAR_OFFSET = Math.PI;
        public static final double BACK_RIGHT_ANGULAR_OFFSET = Math.PI / 2;
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

        public static final Pose2d BLUE_REEF_A = new Pose2d();
        public static final Pose2d BLUE_REEF_B = new Pose2d();
        public static final Pose2d BLUE_REEF_C = new Pose2d();
        public static final Pose2d BLUE_REEF_D = new Pose2d();
        public static final Pose2d BLUE_REEF_E = new Pose2d();
        public static final Pose2d BLUE_REEF_F = new Pose2d();
        public static final Pose2d BLUE_REEF_G = new Pose2d();
        public static final Pose2d BLUE_REEF_H = new Pose2d();
        public static final Pose2d BLUE_REEF_I = new Pose2d();
        public static final Pose2d BLUE_REEF_J = new Pose2d();
        public static final Pose2d BLUE_REEF_K = new Pose2d();
        public static final Pose2d BLUE_REEF_L = new Pose2d();
        
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
    }
}
