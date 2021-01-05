/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.*;


public class DriveTrain extends SubsystemBase {
  private final DifferentialDrive m_myRobot;
  private final SpeedController LFmotor;
  private final SpeedController LMmotor;
  private final SpeedController LRmotor;
  private final SpeedController RFmotor;
  private final SpeedController RMmotor;
  private final SpeedController RRmotor;
  private final SpeedControllerGroup m_Lmotors;
  private final SpeedControllerGroup m_Rmotors;

  /**
   * Creates a new ExampleSubsystem.
   */
  public DriveTrain() {
    LFmotor = new WPI_VictorSPX(0);
    LMmotor = new WPI_VictorSPX(1);
    LRmotor = new WPI_VictorSPX(2);
    RFmotor = new WPI_VictorSPX(3);
    RMmotor = new WPI_VictorSPX(4);
    RRmotor = new WPI_VictorSPX(5);
    m_Lmotors = new SpeedControllerGroup(LFmotor, LMmotor, LRmotor);
    m_Rmotors = new SpeedControllerGroup(RFmotor, RMmotor, RRmotor);
    m_myRobot = new DifferentialDrive(m_Lmotors, m_Rmotors);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void tankDrive(DoubleSupplier m_leftCtrl, DoubleSupplier m_rightCtrl){
    m_myRobot.tankDrive(m_leftCtrl.getAsDouble(), m_rightCtrl.getAsDouble());
  }
}
