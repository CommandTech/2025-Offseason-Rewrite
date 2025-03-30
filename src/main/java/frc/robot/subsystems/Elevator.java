// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Util;

public class Elevator extends SubsystemBase {
  private final SparkMax m_elevatorMotor;
  private final RelativeEncoder m_elevatorEncoder;
  private final SparkClosedLoopController m_elevatorController;

  private double m_targetHeight = 0.0;

  /** Creates a new Elevator. */
  public Elevator() {
    m_elevatorMotor = new SparkMax(ElevatorConstants.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
    m_elevatorMotor.configure(ElevatorConstants.ELEVATOR_CONFIG, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
      
    m_elevatorEncoder = m_elevatorMotor.getEncoder();
    m_elevatorEncoder.setPosition(0);

    m_elevatorController = m_elevatorMotor.getClosedLoopController();
  }

  @Override
  public void periodic() {
    //If it doesn't work, set the voltage here to be the target position - current position and clamp it between -7 and 7
  }

  public double getHeight() {
    return m_elevatorEncoder.getPosition();
  }

  public double getTargetHeight() {
    return m_targetHeight;
  }

  public void setTargetHeight(double height) {
    double newHeight = Util.clamp(height, ElevatorConstants.ELEVATOR_MIN_HEIGHT, ElevatorConstants.ELEVATOR_MAX_HEIGHT);

    m_elevatorController.setReference(newHeight, ControlType.kPosition);
  }

  public Command setElevatorHeight(double height) {
    return runOnce(() -> setTargetHeight(height));
  }
}
