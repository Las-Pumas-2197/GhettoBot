// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class differentialdrive extends SubsystemBase {

  private final SparkMax FL = new SparkMax(configuration.c_FLcanID, MotorType.kBrushless);
  private final SparkMax RL = new SparkMax(configuration.c_RLcanID, MotorType.kBrushless);
  private final SparkMax FR = new SparkMax(configuration.c_FRcanID, MotorType.kBrushless);
  private final SparkMax RR = new SparkMax(configuration.c_RRcanID, MotorType.kBrushless);
  private final RelativeEncoder enc_L = FL.getEncoder();
  private final RelativeEncoder enc_R = FR.getEncoder();
  private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();
  private final DifferentialDriveKinematics kine_drive = new DifferentialDriveKinematics(configuration.c_drivetrackwidth);
  private final DifferentialDriveOdometry odo_drive = new DifferentialDriveOdometry(new Rotation2d(Math.toRadians(gyro.getAngle())), enc_L.getPosition(), enc_R.getPosition());
  private double d_drivespeedmult;

  public differentialdrive() {
    FL.configure(configuration.cfg_frontleft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    RL.configure(configuration.cfg_rearleft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    FR.configure(configuration.cfg_frontright, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    RR.configure(configuration.cfg_rearright, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    gyro.calibrate();
  }

  public void drive(double Xspeed, double Zspeed, boolean turbo_mode, boolean reset_enc){

    //drive speed multiplier for controllability
    d_drivespeedmult = turbo_mode ? 1 : 0.25;
    if(reset_enc){
      enc_R.setPosition(0);
      enc_L.setPosition(0);
    }

    //compose drive speeds
    ChassisSpeeds speeds = new ChassisSpeeds(Xspeed  * d_drivespeedmult * (-1), 0, Zspeed * (-1));
    DifferentialDriveWheelSpeeds wheelspeeds = kine_drive.toWheelSpeeds(speeds);

    //update odometry
    odo_drive.update(new Rotation2d(Math.toRadians(gyro.getAngle())), enc_L.getPosition(), enc_R.getPosition());

    //Telelmety
    SmartDashboard.putNumber("X speed", speeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Rotation", speeds.omegaRadiansPerSecond);
    SmartDashboard.putNumber("Left Speed", wheelspeeds.leftMetersPerSecond);
    SmartDashboard.putNumber("Right Speed", wheelspeeds.rightMetersPerSecond);
    SmartDashboard.putNumber("Drive Speed Mult", d_drivespeedmult);
    // Encoder Telemetry
    SmartDashboard.putNumber("RightPos", enc_R.getPosition());
    SmartDashboard.putNumber("LeftPos", enc_L.getPosition());
    RL.setVoltage(wheelspeeds.leftMetersPerSecond * 12);
    RR.setVoltage(wheelspeeds.rightMetersPerSecond * 12);

    SmartDashboard.putNumber("Xpos", odo_drive.getPoseMeters().getX());
    SmartDashboard.putNumber("Ypos", odo_drive.getPoseMeters().getY());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  

    
  }
}
