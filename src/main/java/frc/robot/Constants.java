// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/** Add your docs here. */
public final class Constants {

    public static final class Elevator{
        public static final int ELEVATOR_ONE_CANID = 20;
        public static final int ELEVATOR_TWO_CANID = 21;
        public static final int ELEVATOR_HOMESWITCH_PORT = 2;  //RoboRIO DIO Port for elevator's lower position

        public static final SparkMaxConfig MOTOR_CONFIG = new SparkMaxConfig();
        public static final IdleMode ELEVATOR_MOTOR_IDLE_MODE = IdleMode.kBrake;
        public static final int MAX_CURRENT_DRAW = 30;

        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double kS = 0.0;    // volts
        public static final double kG = 0.0;    // volts
        public static final double kV = 0.0;    // volts * seconds / distance
        public static final double kA = 0.0;    // volts * seconds^2 / distance

        static{
            MOTOR_CONFIG.smartCurrentLimit(MAX_CURRENT_DRAW);
            MOTOR_CONFIG.idleMode(ELEVATOR_MOTOR_IDLE_MODE);
            MOTOR_CONFIG.inverted(true);
        }

        public static final double L1 = 0.0;
        public static final double L2 = 0.0;
        public static final double L3 = 0.0;
        public static final double L4 = 0.0;
    }

    public static final class Wrist{
        public static final int WRIST_CANID = 22;
        public static final int WRIST_ENCODER_PORT = 5;

        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double errTolerance = 0.1;

        public static final TalonFXConfiguration WRIST_CONFIG = new TalonFXConfiguration();

        static{
            WRIST_CONFIG.CurrentLimits.StatorCurrentLimit = 30;
            WRIST_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
            WRIST_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            WRIST_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        }

        public static final double L1 = 0.0;
        public static final double L2 = 0.0;
        public static final double L3 = 0.0;
        public static final double L4 = 0.0;
    }

    public static final class Manipulator{
        public static final int MAN_CANID = 42;
        public static final int CORAL_SENSOR_PORT = 0;
        public static final int ALGAE_SENSOR_PORT = 9;
        public static final double ALGAE_INTAKE_SPEED = 0.25;
        public static final double CORAL_INTAKE_SPEED = 0.15;
        public static final double OUTTAKE_SPEED = 0.10;
    }
}
