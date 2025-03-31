package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.GoalConstants;
import frc.robot.subsystems.*;

public class RobotContainer {
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final CommandXboxController m_manipController = new CommandXboxController(1);

  private final Vision m_camera = new Vision();
  private final Drivetrain m_swerve = new Drivetrain(m_camera);
  private final Elevator m_elevator = new Elevator();
  private final Climber m_climber = new Climber();
  private final Coral m_coral = new Coral();
  private final Algae m_algae = new Algae();
  AutoScorer m_autoScorer = new AutoScorer(m_coral, m_swerve, m_elevator);

  private SendableChooser<Command> m_chooser = new SendableChooser<>();

  public RobotContainer() {
    m_swerve.setDefaultCommand(m_swerve.driveCommand(m_driverController.getLeftX(), m_driverController.getLeftY(), m_driverController.getRightX()));
    SmartDashboard.putData(CommandScheduler.getInstance());
    SmartDashboard.putData("Auto Mode", m_chooser);

    configureButtonBindings();
  }

  public void configureButtonBindings() {
    // Climb Commands
    m_driverController.leftTrigger(50).onTrue(m_climber.toggleClimber());

    // Elevator Commands for the Manipulator
    m_manipController.povUp().onTrue(m_elevator.setElevatorHeight(ElevatorConstants.ELEVATOR_L3_HEIGHT));
    m_manipController.povRight().onTrue(m_elevator.setElevatorHeight(ElevatorConstants.ELEVATOR_L2_HEIGHT));
    m_manipController.povDown().onTrue(m_elevator.setElevatorHeight(ElevatorConstants.ELEVATOR_STOW_HEIGHT));
    m_manipController.povLeft().onTrue(m_elevator.setElevatorHeight(400));

    m_manipController.a().onTrue(m_swerve.goToPose(GoalConstants.BLUE_REEF_A,true));
    
    // Coral Flume Commands for the Manipulator
    m_manipController.leftBumper().onTrue(m_coral.autoLockRelease());

    // Algae Holder Commands for the Manipulator
    m_manipController.rightBumper().onTrue(m_algae.toggleAlgaeHolder());
    
    // Alage Tusk Commands for the Manipulator
    m_manipController.y().onTrue(m_algae.setTuskPosition(AlgaeConstants.HOME_POSITION));
    m_manipController.x().onTrue(m_algae.setTuskPosition(AlgaeConstants.APPROACH_POSITION));
    m_manipController.a().onTrue(m_algae.setTuskPosition(AlgaeConstants.LOWER_POSITION));
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
