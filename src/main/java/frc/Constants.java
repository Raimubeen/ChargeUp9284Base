// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

/** Centralized location for constants.  Constants can include but are 
 * not limited to CAN IDs, SENSOR/MOTOR IDs, Subsystem specific limits and strings.
 */
public final class Constants {

    public static final class DriveConstants {
        public static final int CAN_ID_DRIVE_LEFT_FRONT = 13;
        public static final int CAN_ID_DRIVE_LEFT_BACK = 14;
        public static final int CAN_ID_DRIVE_RIGHT_FRONT  = 11;
        public static final int CAN_ID_DRIVE_RIGHT_BACK  = 12;

        public static final double LOWER_FORWARD_THRESHOLD = 0.1;
        public static final double LOWER_TURN_THRESHOLD = 0.15;
    }

    public static final class ArmConstants {
        public static final int CAN_ID_ARM = 21;
        /**
          * How many amps the arm motor can use.
          */
        public static final int ARM_CURRENT_LIMIT_A = 30;

        /**
          * Percent output to run the arm up/down at
          */
        public static final double ARM_OUTPUT_POWER = 0.6;
        public static final double AUTO_ARM_OUTPUT_POWER = 0.5;
        public static final double ARM_HOLDSTALL_POWER = 0.1;
         /**
           * Time to extend or retract arm in auto
           */
        public static final double ARM_EXTEND_TIME_S = 2.0;
        public static final double ARM_DEADZONE_THRESHOLD = 0.04;
        public static final double ARM_RAISE_MAX_OUTPUT = 1.0;
        public static final double ARM_LOWER_MAX_OUTPUT = 0.30;
    }

    public static final class IntakeConstants {
        public static final int CAN_ID_INTAKE = 31;
        /**
          * How many amps the intake can use while picking up
          */
        public static final int INTAKE_CURRENT_LIMIT_A = 25;

        /**
          * How many amps the intake can use while holding
          */
        public static final int INTAKE_HOLD_CURRENT_LIMIT_A = 5;

        /**
          * Percent output for intaking
          */
        public static final double INTAKE_OUTPUT_POWER = 1.0;

        /**
          * Percent output for holding
          */
        public static final double INTAKE_HOLD_POWER = 0.07;
    }

    public static final class AutoConstants {
        public static final String kNothingAuto = "do nothing";
        public static final String kConeAuto = "cone";
        public static final String kCubeAuto = "cube";



        public static final double AUTO_THROW_TIME_S = 0.375;

        /**
          * Time to drive back in auto
          */
        public static final double AUTO_DRIVE_TIME1 = 0.5;
        public static final double AUTO_DRIVE_TIME2 = 0.5;
        public static final double AUTO_DRIVE_TIME3 = 6.0;

        /**
          * Speed to drive backwards in auto
          */
        public static final double AUTO_DRIVE_SPEED = -0.25;
    }

    public static final class JoystickConstants {
        public static final int JOYSTICK_ID_DRIVER = 0;
        public static final int JOYSTICK_AXIS_ID_LOWER_ARM = 2;
        public static final int JOYSTICK_AXIS_ID_RAISE_ARM = 3;
        public static final int JOYSTICK_BUTTON_ID_CUBE_IN_OR_CONE_OUT = 6; // cube in or cone out
        public static final int JOYSTICK_BUTTON_ID_CONE_IN_OR_CUBE_OUT = 5; // cone in or cube out
        public static final int JOYSTICK_AXIS_FORWARD = 1;
        public static final int JOYSTICK_AXIS_TURN = 4;
    }

    public static final class GamePieceConstants {
        /**
          * Used to remember the last game piece picked up to apply some holding power.
          */
        public static final int CONE = 1;
        public static final int CUBE = 2;
        public static final int NOTHING = 3;
    }

}
