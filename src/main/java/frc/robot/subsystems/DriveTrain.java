/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.*;


public class DriveTrain extends SubsystemBase {
  private final DifferentialDrive m_driveTrain;
  private final SpeedController Lmotor1;
  private final SpeedController Lmotor2;
  private final SpeedController Lmotor3;
  private final SpeedController Rmotor1;
  private final SpeedController Rmotor2;
  private final SpeedController Rmotor3;
  private final SpeedControllerGroup m_Lmotors;
  private final SpeedControllerGroup m_Rmotors;
  private final Encoder encoderL;
  private final Encoder encoderR;
  private final ADXRS450_Gyro gyro;

  /**
   * Creates a new ExampleSubsystem.
   */
  public DriveTrain() {
    Lmotor1 = new WPI_VictorSPX(DriveTrainConst.kLeftMotor1Port);
    Lmotor2 = new WPI_VictorSPX(DriveTrainConst.kLeftMotor2Port);
    Lmotor3 = new WPI_VictorSPX(DriveTrainConst.kLeftMotor3Port);
    Rmotor1 = new WPI_VictorSPX(DriveTrainConst.kRightMotor1Port);
    Rmotor2 = new WPI_VictorSPX(DriveTrainConst.kRightMotor2Port);
    Rmotor3 = new WPI_VictorSPX(DriveTrainConst.kRightMotor3Port);

    m_Lmotors = new SpeedControllerGroup(Lmotor1, Lmotor1, Lmotor3);
    m_Rmotors = new SpeedControllerGroup(Rmotor1, Rmotor2, Rmotor3);
    m_driveTrain = new DifferentialDrive(m_Lmotors, m_Rmotors);

    encoderL = new Encoder(DriveTrainConst.kLeftEncoderPort[0], DriveTrainConst.kLeftEncoderPort[1]);
    encoderR = new Encoder(DriveTrainConst.kRightEncoderPort[0], DriveTrainConst.kRightEncoderPort[1]);

    gyro = new ADXRS450_Gyro();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void tankDrive(DoubleSupplier m_leftCtrl, DoubleSupplier m_rightCtrl){
    m_driveTrain.tankDrive(m_leftCtrl.getAsDouble(), m_rightCtrl.getAsDouble());
  }
}
