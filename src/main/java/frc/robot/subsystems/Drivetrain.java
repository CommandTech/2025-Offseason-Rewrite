// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GoalConstants;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {
  private final SwerveModule m_frontLeft = new SwerveModule(DriveConstants.FRONT_LEFT_DRIVE_MOTOR_ID, DriveConstants.FRONT_LEFT_TURN_MOTOR_ID, DriveConstants.FRONT_LEFT_ANGULAR_OFFSET);
  private final SwerveModule m_frontRight = new SwerveModule(DriveConstants.FRONT_RIGHT_DRIVE_MOTOR_ID, DriveConstants.FRONT_RIGHT_TURN_MOTOR_ID, DriveConstants.FRONT_RIGHT_ANGULAR_OFFSET);
  private final SwerveModule m_backLeft = new SwerveModule(DriveConstants.BACK_LEFT_DRIVE_MOTOR_ID, DriveConstants.BACK_LEFT_TURN_MOTOR_ID, DriveConstants.BACK_LEFT_ANGULAR_OFFSET);
  private final SwerveModule m_backRight = new SwerveModule(DriveConstants.BACK_RIGHT_DRIVE_MOTOR_ID, DriveConstants.BACK_RIGHT_TURN_MOTOR_ID, DriveConstants.BACK_RIGHT_ANGULAR_OFFSET);

  public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
  public static final double kFrontRightChassisAngularOffset = 0;
  public static final double kBackLeftChassisAngularOffset = Math.PI;
  public static final double kBackRightChassisAngularOffset = Math.PI / 2;

  private RobotConfig config;

  private Vision m_camera;

  private final AHRS m_gyro = new AHRS(NavXComType.kI2C);
  
  private Pose2d m_targetPose;
  
  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          DriveConstants.FRONT_LEFT_LOCATION, DriveConstants.FRONT_RIGHT_LOCATION, DriveConstants.BACK_LEFT_LOCATION, DriveConstants.BACK_RIGHT_LOCATION);

  private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(m_kinematics, m_gyro.getRotation2d(), new SwerveModulePosition[] {
    m_frontLeft.getPosition(),
    m_frontRight.getPosition(),
    m_backLeft.getPosition(),
    m_backRight.getPosition()
  }, new Pose2d());

  //Publishing data to the network table
  StructPublisher<Pose2d> robotPose = NetworkTableInstance.getDefault()
  .getStructTopic("Robot Pose", Pose2d.struct).publish();
  
  StructArrayPublisher<SwerveModuleState> swerveStates = NetworkTableInstance.getDefault()
  .getStructArrayTopic("Swerve Module States", SwerveModuleState.struct).publish();

  public Drivetrain(Vision camera) {
    this.m_camera = camera;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(DriveConstants.DRIVE_P, DriveConstants.DRIVE_I, DriveConstants.DRIVE_D), // Translation PID constants
                    new PIDConstants(DriveConstants.TURN_P, DriveConstants.TURN_I, DriveConstants.TURN_D) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }

  @Override
  public void periodic(){
    updateOdometry();

    m_poseEstimator.addVisionMeasurement(m_camera.getVisionPose().toPose2d(), m_camera.getTimestamp());
    robotPose.set(m_poseEstimator.getEstimatedPosition());
    swerveStates.set(getModuleStates());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot),
                periodSeconds));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.MAX_SPEED);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_poseEstimator.update(m_gyro.getRotation2d(), getModulePositions());
  }

  public Pose2d getPose(){
    return m_poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose){
    m_poseEstimator.resetPosition(m_gyro.getRotation2d(), getModulePositions(), pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds(){
    return m_kinematics.toChassisSpeeds(getModuleStates());
  }

  public void driveRobotRelative(ChassisSpeeds speeds){
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = m_kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, DriveConstants.MAX_SPEED);

    m_frontLeft.setDesiredState(setpointStates[0]);
    m_frontRight.setDesiredState(setpointStates[1]);
    m_backLeft.setDesiredState(setpointStates[2]);
    m_backRight.setDesiredState(setpointStates[3]);
  }

  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    states[0] = m_frontLeft.getState();
    states[1] = m_frontRight.getState();
    states[2] = m_backLeft.getState();
    states[3] = m_backRight.getState();
    return states;
  }

  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    positions[0] = m_frontLeft.getPosition();
    positions[1] = m_frontRight.getPosition();
    positions[2] = m_backLeft.getPosition();
    positions[3] = m_backRight.getPosition();
    return positions;
  }

  public Command driveCommand(double xSpeed, double ySpeed, double rot){
    return runOnce(() -> drive(xSpeed, ySpeed, rot, true, 0.02));
  }

  public Pose2d getTargetPose(){
    return m_targetPose;
  }

  public Command goToPose(Pose2d targetPose, boolean forAlgae){
    m_targetPose = targetPose;
    // Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
            3.0, 4.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    Command pathfindingCommand = AutoBuilder.pathfindToPose(
            forAlgae ? targetPose.plus(GoalConstants.ROBOT_ALGAE_OFFSET) : targetPose,
            constraints
    );
    return pathfindingCommand;
  }

  public Command goToClosestReef(boolean forAlgae){
    Pose2d targetPose = getClosestBranch(forAlgae);
    return goToPose(targetPose, forAlgae);
  }

  public Pose2d getClosestBranch(boolean forAlgae){
    Pose2d currentPose = getPose();
    Pose2d closestBranch = null;
    double closestScore = Double.MAX_VALUE;
    int closestTag = -1;
    int[] tags = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? GoalConstants.BLUE_TAGS : GoalConstants.RED_TAGS;
    Pose2d[] branches = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? GoalConstants.BLUE_BRANCHES : GoalConstants.RED_BRANCHES;
    
    for (int tag : tags){
      Pose2d tagLocation = m_camera.getFieldLayout().getTagPose(tag).get().toPose2d();
      double distance = currentPose.getTranslation().getDistance(tagLocation.getTranslation());

      double angleDifference = Math.abs(currentPose.getRotation().getRadians() - tagLocation.getRotation().getRadians());
      double score = distance + angleDifference; // Combine distance and rotation difference

      if (score < closestScore) {
        closestScore = score;
        closestBranch = tagLocation;
        closestTag = tag;
      }
    }

    Pose2d[] selectbranches = new Pose2d[2];
    switch (closestTag){
      case 8:
      case 17:
        selectbranches = new Pose2d[]{branches[2], branches[3]};
        break;
      case 7:
      case 18:
        selectbranches = new Pose2d[]{branches[0], branches[1]};
        break;
      case 6:
      case 19:
        selectbranches = new Pose2d[]{branches[10], branches[11]};
        break;
      case 11:
      case 20:
        selectbranches = new Pose2d[]{branches[8], branches[9]};
        break;
      case 10:
      case 21:
        selectbranches = new Pose2d[]{branches[6], branches[7]};
        break;
      case 9:
      case 22:
        selectbranches = new Pose2d[]{branches[4], branches[5]};
        break;
    }

    double closestDistance = Double.MAX_VALUE;
    for (Pose2d branch : selectbranches){
      double distance = currentPose.getTranslation().getDistance(branch.getTranslation());

      if (distance < closestDistance) {
        closestDistance = distance;
        closestBranch = branch;
      }
    }

    return closestBranch;
  }

  public Command driveToCage(int cageNum){
    switch (cageNum) {
      case 1:
        return runOnce(() -> goToPose(ClimberConstants.CAGE1_POSE, true));
      case 2:
        return runOnce(() -> goToPose(ClimberConstants.CAGE2_POSE, true));
      case 3:
        return runOnce(() -> goToPose(ClimberConstants.CAGE3_POSE, true));
      default:
        break;
    }
    return runOnce(() -> goToPose(getPose(), false));
  }
}
