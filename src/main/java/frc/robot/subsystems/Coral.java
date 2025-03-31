// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;

public class Coral extends SubsystemBase {
  private AnalogInput m_coralSense;
  private DoubleSolenoid m_coralStopper;
  private Timer m_timer;
  
  private boolean coralStopped = false;
  private boolean enableAutoStopCoral = true;
  private boolean coralAutostopped = false;
  private boolean requestedAutoStopRelease = false;

  public Coral() {
    m_coralStopper =
        new DoubleSolenoid(
            CoralConstants.PH_CAN_ID,
            PneumaticsModuleType.REVPH,
            CoralConstants.CORAL_STOPPER_EXTEND_CHANNEL,
            CoralConstants.CORAL_STOPPER_RETRACT_CHANNEL);
    m_coralStopper.set(DoubleSolenoid.Value.kReverse);
    coralStopped = false;

    // Setup Coral Sense Analog Port
    m_coralSense = new AnalogInput(CoralConstants.CORAL_SENSOR_ANALOG_PORT);

    // Init Timer
    m_timer = new Timer();
    m_timer.start();
  }

  @Override
  public void periodic() {
    if (coralStopped == false & isCoralSensed() == true & isAutoStopEnabled() == true
        && m_timer.hasElapsed(CoralConstants.AUTO_CORAL_SENCE_DELAY)) {
      System.out.print("Coral Flume - Auto Stopped");
      extendCoralStop();
      coralAutostopped = true;
    } else if (requestedAutoStopRelease == true) {
      System.out.print("Coral Flume - Auto Released");
      retractCoralStop();
      requestedAutoStopRelease = false;
      m_timer.reset();
    } else if (isCoralSensed() == false
        & coralStopped == false
        & m_coralStopper.get() == DoubleSolenoid.Value.kForward) {
      System.out.print("Coral Flume - Mismatch Detected, Releasing");
      retractCoralStop();
    }
  }

  public boolean isCoralSensed() {
    return m_coralSense.getValue() <= CoralConstants.CORAL_SENSE_THRESHOLD;
  }

  public boolean isCoralAutoStopped() {
    return coralAutostopped;
  }

  public boolean isAutoStopEnabled() {
    return enableAutoStopCoral;
  }

  public void setAutoStop(boolean isEnabled) {
    enableAutoStopCoral = isEnabled;
  }

  public void extendCoralStop() {
    System.out.print("Coral Flume - Cylinder Extended");
    m_coralStopper.set(DoubleSolenoid.Value.kForward);
    coralStopped = true;
  }

  public void retractCoralStop() {
    System.out.print("Coral Flume - Cylinder Released");
    m_coralStopper.set(DoubleSolenoid.Value.kReverse);
    coralStopped = false;
    coralAutostopped = false;
  }

  public void requestedAutoStopRelease() {
    requestedAutoStopRelease = true;
  }

  public Command lockCoral() {
    return run(() -> extendCoralStop());
  }

  public Command releaseCoral() {
    return run(() -> retractCoralStop());
  }

  public Command autoLockRelease() {
    return run(() -> requestedAutoStopRelease());
  }
}
