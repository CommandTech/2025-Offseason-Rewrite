// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {

  public PneumaticHub m_ph;
  private DoubleSolenoid cylinder1;
  private Solenoid cylinder2;

  private boolean climberDown = false;

  public Climber() {
    m_ph = new PneumaticHub(ClimberConstants.PH_CAN_ID);
    m_ph.enableCompressorAnalog(ClimberConstants.COMP_START_PRESSURE, ClimberConstants.COMP_STOP_PRESSURE);
    
    cylinder1 = new DoubleSolenoid(ClimberConstants.PH_CAN_ID, PneumaticsModuleType.REVPH, 0, 1);
    cylinder2 = new Solenoid(ClimberConstants.PH_CAN_ID, PneumaticsModuleType.REVPH, 3);
    cylinder1.set(DoubleSolenoid.Value.kReverse);
    cylinder2.set(false);
    climberDown = false;
  }

  @Override
  public void periodic() {

  }
  
  public void extendClimber() {
    cylinder1.set(DoubleSolenoid.Value.kForward);
    cylinder2.set(false);
    climberDown = true;
  }

  public void retractClimber() {
    cylinder1.set(DoubleSolenoid.Value.kReverse);
    cylinder2.set(true);
    climberDown = false;
  }

  public Command toggleClimber() {
    return runOnce(climberDown ? () -> retractClimber() : () -> extendClimber());
  }
}
