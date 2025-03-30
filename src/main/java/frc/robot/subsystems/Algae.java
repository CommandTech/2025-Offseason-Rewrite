// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util;
import frc.robot.Constants.AlgaeConstants;

public class Algae extends SubsystemBase {

  private DoubleSolenoid m_algaeStopper;
  private boolean algaeStopped = false;

  private SparkMax m_tusks;
  private SparkClosedLoopController m_ClosedLoopController;
  private SparkAbsoluteEncoder m_tuskEncoder;
  
  public Algae() {
    m_algaeStopper =
        new DoubleSolenoid(
            AlgaeConstants.PH_CAN_ID,
            PneumaticsModuleType.REVPH,
            AlgaeConstants.ALGAE_STOPPER_RETRACT_CHANNEL,
            AlgaeConstants.ALGAE_STOPPER_EXTEND_CHANNEL);
    m_algaeStopper.set(DoubleSolenoid.Value.kForward);
    algaeStopped = true;

    m_tusks = new SparkMax(AlgaeConstants.TUSKS_MOTOR_ID, MotorType.kBrushed);
    m_ClosedLoopController = m_tusks.getClosedLoopController();
    m_tuskEncoder = m_tusks.getAbsoluteEncoder();

    m_tusks.configure(AlgaeConstants.ALGAE_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {

  }

  public void extendAlgaeStop() {
    m_algaeStopper.set(DoubleSolenoid.Value.kForward);
    algaeStopped = true;
  }

  public void retractAlgaeStop() {
    m_algaeStopper.set(DoubleSolenoid.Value.kReverse);
    algaeStopped = false;
  }

  public double getCurrentPosition() {
    return m_tuskEncoder.getPosition();
  }

  public void setTargetPosition(double target) {
    double targetPosition = Util.clamp(target, AlgaeConstants.TUSKS_MIN_HEIGHT, AlgaeConstants.TUSKS_MAX_HEIGHT);
    m_ClosedLoopController.setReference(targetPosition, ControlType.kMAXMotionPositionControl);
  }
  
  public Command toggleAlgaeHolder() {
    return runOnce(algaeStopped ? () -> retractAlgaeStop() : () -> extendAlgaeStop());
  }

  public Command setTuskPosition(double height){
    return runOnce(() -> setTargetPosition(height));
  }
}
