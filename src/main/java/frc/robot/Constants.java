package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class Constants {
    public static final class DriveConstants {
        public static final double MAX_SPEED = 3.0; // 3 meters per second
        public static final double MAX_ANGULAR_SPEED = Math.PI; // 1/2 rotation per second
        
        public static final double WHEEL_RADIUS = 0.0508;
        public static final int ENCODER_RESOLUTION = 4096;
        
        public static final double MODULE_MAX_ANGULAR_VELOCITY = MAX_ANGULAR_SPEED;
        public static final double MODULE_MAX_ANGULAR_ACCELERATION = 2 * Math.PI; // radians per second squared

        public static final double DRIVE_P = 1;
        public static final double DRIVE_I = 0;
        public static final double DRIVE_D = 0;
        public static final double DRIVE_FF = 1;
        
        public static final double TURN_P = 1;
        public static final double TURN_I = 0;
        public static final double TURN_D = 0;
        public static final double TURN_FF = 0.5;
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
}
