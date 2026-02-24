// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import frc.demacia.utils.motors.BaseMotorConfig.Canbus;
import frc.demacia.utils.motors.TalonFXConfig;
import frc.demacia.utils.motors.TalonSRXConfig;

/** Add your docs here. */
public class IntakeConstants {
    //the all intake canbus
    public static final Canbus INTAKE_CANBUS = Canbus.Rio;

    //the motors max power
    public static final double MAX_POWER = 0.8;

    //the motors max pose
    public static final double MAX_POSITION = 6;
    public static final double MIN_POSITION = 0;

    public static final int INTAKE_ID = 20;
    public static final String INTAKE_NAME = "Intake/Motor";

    //intake motor
    public static final TalonFXConfig INTAKE_CONFIG = new TalonFXConfig(INTAKE_ID, INTAKE_CANBUS, INTAKE_NAME).withInvert(true);
    
    //battery value
    public static final int BATTERY_ID = 34;
    public static final String BATTER_NAME = "Shinua/Battery";

    //battery pid feed forward
    public static final double BATTERY_KP = 20;
    public static final double BATTERT_KI = 0;
    public static final double BATTERY_KD = 0;
    public static final double BATTERT_KS = 0;
    public static final double BATTERY_KV = 0;
    public static final double BATTERY_KA = 0;
    public static final double BATTER_KG = 0;

    //the battery motor config
    public static final TalonFXConfig BATTERY_CONFIG = new TalonFXConfig(BATTERY_ID, INTAKE_CANBUS, BATTER_NAME)
    .withBrake(true)
    .withInvert(false)
    .withCurrent(20)
    .withPID(BATTERY_KP, BATTERT_KI, BATTERY_KD, BATTERT_KS, BATTERY_KV, BATTERY_KA, BATTER_KG)
    .withRadiansMotor(36);

    //indexer on the top value
    public static final int INDEXER_ON_TOP_ID = 30;
    public static final String INDEXER_TOP_NAME  = "Shinua/Top";
    public static final TalonFXConfig INDEXER_ON_TOP_CONFIG = new TalonFXConfig(INDEXER_ON_TOP_ID, Canbus.CANIvore, INDEXER_TOP_NAME)
    .withBrake(true)
    .withInvert(true);

    //indexer close value
    public static final int INDEXER_CLOSE_ID = 32;
    public static final String INDEXER_CLOSE_NAME  = "Indexer Close motor";
    public static final TalonSRXConfig INDEXER_CLOSE_CONFIG = new TalonSRXConfig(INDEXER_CLOSE_ID, INDEXER_CLOSE_NAME)
    .withBrake(true).withInvert(false).withCurrent(15);

    //indexer close value
    public static final int INDEXER_FAR_ID = 33;
    public static final String INDEXER_FAR_NAME  = "Indexer far motor";
    public static final TalonSRXConfig INDEXER_FAR_CONFIG = new TalonSRXConfig(INDEXER_FAR_ID, INDEXER_FAR_NAME)
    .withBrake(true)
    .withCurrent(15)
    .withInvert(false);
}
