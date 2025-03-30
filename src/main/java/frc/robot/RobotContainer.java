package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.GoalConstants;
import frc.robot.subsystems.*;

public class RobotContainer {
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final CommandXboxController m_manipController = new CommandXboxController(1);

  private final Vision m_camera = new Vision();
  private final Drivetrain m_swerve = new Drivetrain(m_camera);
  private final Elevator m_elevator = new Elevator();
  private final Coral m_coral = new Coral();
  private final Algae m_algae = new Algae();

  private SendableChooser<Command> m_chooser = new SendableChooser<>();

  public RobotContainer() {
    m_swerve.setDefaultCommand(m_swerve.driveCommand(m_driverController.getLeftX(), m_driverController.getLeftY(), m_driverController.getRightX()));
    SmartDashboard.putData(CommandScheduler.getInstance());
    SmartDashboard.putData("Auto Mode", m_chooser);

    configureButtonBindings();
  }

  public void configureButtonBindings() {
    // Configure your button bindings here
    
    m_manipController.a().onTrue(m_swerve.goToPose(GoalConstants.BLUE_REEF_A,true));
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
