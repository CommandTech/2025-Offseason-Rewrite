// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;

public class AutoScorer extends SubsystemBase {
  private Coral m_coral;
  private Drivetrain m_drivetrain;
  private Elevator m_elevator;

  public AutoScorer(Coral coral, Drivetrain drivetrain, Elevator elevator) {
    m_coral = coral;
    m_drivetrain = drivetrain;
    m_elevator = elevator;
  }

  @Override
  public void periodic() {
    if (m_drivetrain.getTargetPose().minus(m_drivetrain.getPose()).getTranslation().getNorm() < DriveConstants.POSE_TOLERANCE && 
    (m_elevator.getTargetHeight() - m_elevator.getHeight() < ElevatorConstants.ELEVATOR_HEIGHT_TOLERANCE)) {
      m_coral.retractCoralStop();
    } else {
      m_coral.extendCoralStop();
    };
  }
}
