// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class configuration {

  //can IDs for spark maxes
  public static final int c_FLcanID = 4;
  public static final int c_RLcanID = 8;
  public static final int c_FRcanID = 2;
  public static final int c_RRcanID = 5;

  //misc drive subsystem parameters
  public static final int c_drivecurrentlimit = 50;
  public static final double c_drivespeedmax = 1; //meters per second
  public static final double c_driveconvfactor = (0.47877/10.71);
  public static final double c_drivetrackwidth = Units.inchesToMeters(21.875);

  public static final SparkMaxConfig cfg_frontleft = new SparkMaxConfig();
  public static final SparkMaxConfig cfg_rearleft = new SparkMaxConfig();
  public static final SparkMaxConfig cfg_frontright = new SparkMaxConfig();
  public static final SparkMaxConfig cfg_rearright = new SparkMaxConfig();

  static {
    cfg_frontleft.smartCurrentLimit(c_drivecurrentlimit);
    cfg_frontleft.follow(c_RLcanID);
    cfg_frontleft.encoder.positionConversionFactor(c_driveconvfactor);

    cfg_frontright.smartCurrentLimit(c_drivecurrentlimit);
    cfg_frontright.follow(c_RRcanID);
    cfg_frontright.encoder.positionConversionFactor(c_driveconvfactor);

    cfg_frontright.inverted(true);

    cfg_rearleft.smartCurrentLimit(c_drivecurrentlimit);
    cfg_rearleft.encoder.positionConversionFactor(c_driveconvfactor);

    cfg_rearright.smartCurrentLimit(c_drivecurrentlimit);
    cfg_rearright.inverted(true);
    cfg_rearright.encoder.positionConversionFactor(c_driveconvfactor);
  }

}

