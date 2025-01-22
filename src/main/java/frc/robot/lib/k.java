//Copyright (c) 2020-2023 Essexville Hampton Public Schools (FRC 8517)

package frc.robot.lib;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class k {
  public static class CANIVORE_IDS {
    public static final String NAME = "CANivore";
    public static final int PIGEON2_CANID = 10;
    // public static final int DRIVE_LEFT_CANID = 12;
    // public static final int DRIVE_RIGHT_CANID = 13;
    // public static final int DRIVE_BACK_CANID = 11;

    // public static final int STEER_LEFT_CANID = 22;
    // public static final int STEER_RIGHT_CANID = 23;
    // public static final int STEER_BACK_CANID = 21;

    // public static final int CANCODER_LEFT_CANID = 3;
    // public static final int CANCODER_RIGHT_CANID = 2;
    // public static final int CANCODER_BACK_CANID = 1;
  }

  public static class ROBORIO_CAN_IDS {
    public static final String NAME = "5216 CANivore";
  }

  public static class CV {
    public static final double DEGREES_TO_RADIANS = 0.017453292519943295;
    public static final double RADIANS_TO_DEGREES = 57.29577951308232;
    public static final double INCHES_TO_METERS = 0.0254;
  }

  public static class ROBOT {
    public static final double PERIOD = 0.02;

    public static final double BATTERY_MAX_VOLTS = 12.8;

    public static final int PD_CANID = 1; // Power Distribution, Rev or CTRE

  }

  public static class OI {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }
//   public static class OI {

//     // Driver controller
//     public static final int DRIVER_CONTROLLER_PORT = 0;
//     public static Trigger DRIVER_SHOT_POSITION_LEFT = RobotContainer.s_driverController.square();
//     public static Trigger DRIVER_SHOT_POSITION_PODIUM = RobotContainer.s_driverController.triangle();
//     public static Trigger DRIVER_SHOT_POSITION_RIGHT = RobotContainer.s_driverController.circle();
//     public static Trigger DRIVER_SHOT_POSITION_STRAIGHT = RobotContainer.s_driverController.cross();

//     public static Trigger DRIVER_DRIVE_MODE_FIELDCENTRIC = RobotContainer.s_driverController.povLeft();
//     public static Trigger DRIVER_DRIVE_MODE_ROBOTCENTRIC = RobotContainer.s_driverController.povRight();
//     public static Trigger DRIVER_DRIVE_MODE_ANGLEFIELDCENTRIC = RobotContainer.s_driverController.povDown();
//     public static Trigger DRIVER_DRIVE_MODE_ROTATEFIELDCENTRIC = RobotContainer.s_driverController.povUp();

//     public static Trigger DRIVER_DRIVE_MODE_SPEED_HI = RobotContainer.s_driverController.R1();
//     public static Trigger DRIVER_DRIVE_MODE_SPEED_LOW = RobotContainer.s_driverController.L1();

//     public static Trigger DRIVER_RESET_YAW = RobotContainer.s_driverController.create();

//     // Operator controller
//     public static final int OPERATOR_CONTROLLER_PORT = 1;
//     public static Trigger OPERATOR_FLIPPER_EXTEND = RobotContainer.s_operatorController.R1();
//     public static Trigger OPERATOR_FLIPPER_PRELOAD = RobotContainer.s_operatorController.L1();
//     public static Trigger OPERATOR_FLIPPER_BACK = RobotContainer.s_operatorController.touchpad();

//     public static Trigger OPERATOR_INTAKE_SPIN_ON = RobotContainer.s_operatorController.triangle();
//     public static Trigger OPERATOR_INTAKE_SPIN_OFF = RobotContainer.s_operatorController.cross();
//     public static Trigger OPERATOR_INTAKE_SPIN_REVERSE = RobotContainer.s_operatorController.square();

//     public static Trigger OPERATOR_SHOOTER_FEED = RobotContainer.s_operatorController.povUp();
//     public static Trigger OPERATOR_SHOOTER_OFF = RobotContainer.s_operatorController.povDown();

//     public static Trigger OPERATOR_AMP_UP = RobotContainer.s_operatorController.povLeft();
//     public static Trigger OPERATOR_AMP_DOWN = RobotContainer.s_operatorController.povRight();

//   }

  public static class DRIVEBASE {
    public static final double WHEEL_BASE_Y_m = 0.5;
    public static final double WHEEL_BASE_X_m = 0.5;
    private static final double WHEEL_BASE_XY_AVG_m = (WHEEL_BASE_Y_m + WHEEL_BASE_X_m) / 2.0;
    private static final double WHEEL_BASE_CIRCUMFERENCE_m = Math.PI * WHEEL_BASE_XY_AVG_m;
    private static final double WHEEL_BASE_MeterPerRad = WHEEL_BASE_CIRCUMFERENCE_m / (2 * Math.PI);
    public static final double TURN_KP = 4;
    public static final double TURN_KI = 0.0;
    public static final double TURN_KD = 0.0;
  }

  public static class DRIVE {
    public static final String T_DRIVER_MODE = "DriveMode";
    private static final double MOTOR_PINION_TEETH = 12.0;
    private static final double GEAR_1_TEETH = 34.0; // This number changed X1: 34 | X3: 32
    private static final double GEAR_2_DRIVE_TEETH = 26.0; // This number changed X1: 26 | X3: 28
    private static final double GEAR_2_DRIVEN_TEETH = 20.0; // This number changed X1: 20 | X3: 18
    private static final double GEAR_BEVEL_DRIVE_TEETH = 15.0;
    private static final double GEAR_BEVEL_DRIVEN_TEETH = 45.0;

    public static final double GEAR_RATIO = (GEAR_1_TEETH / MOTOR_PINION_TEETH)
        * (GEAR_2_DRIVEN_TEETH / GEAR_2_DRIVE_TEETH) * (GEAR_BEVEL_DRIVEN_TEETH / GEAR_BEVEL_DRIVE_TEETH);
    public static final double WHEEL_DIAMETER_m = .1015; // .10287
    private static final double WHEEL_CIRCUMFERENCE_m = Math.PI * WHEEL_DIAMETER_m;
    public static final double WHEEL_MotRotPerMeter = GEAR_RATIO / WHEEL_CIRCUMFERENCE_m;
    private static final double MOTOR_MAX_VELOCITY_RotPerMin = 6080.0;
    private static final double MOTOR_MAX_VELOCITY_RotPerSec = MOTOR_MAX_VELOCITY_RotPerMin / 60.0;
    private static final double WHEEL_MAX_VELOCITY_RotPerSec = MOTOR_MAX_VELOCITY_RotPerSec / GEAR_RATIO;
    private static final double MOTOR_PEAK_EFFICIENCY_percent = 100;
    public static final double MAX_VELOCITY_MeterPerSec = WHEEL_CIRCUMFERENCE_m * WHEEL_MAX_VELOCITY_RotPerSec * MOTOR_PEAK_EFFICIENCY_percent / 100.0;
    // 10T = 3.63 Mps = 11.9 FtPSec,
    // 11T 3.994 Mps = 13.1 FtPSec,
    // 12T 4.35 Mps = 14.27 FtPSec, 91% Eff
    public static final double MAX_ANGULAR_VELOCITY_RadianPerSec = MAX_VELOCITY_MeterPerSec
        * (1 / DRIVEBASE.WHEEL_BASE_MeterPerRad);
    public static final double PID_Kp = 1.0;
    public static final double PID_Ki = 0.0;
    public static final double PID_Kv = k.ROBOT.BATTERY_MAX_VOLTS / k.DRIVE.MAX_VELOCITY_MeterPerSec;
    public static final double PID_Ks = 0.0;
    public static final double TARGET_ANGLE_DEADBAND = 0.8;
    public static final double APRIL_AREA_FACTOR = 3.0;
    public static final double APRIL_AREA_SHOT = 1.0;
    public static final double APRIL_YAW_FACTOR = 1.50;
    public static final double NOTE_YAW_FACTOR = 2;

    public static final double ROTATE_FIELDCENTRIC_SPEED_RadPSec = 10.0;
  }

  public static class STEER {
    private static final double MOTOR_PINION_TEETH = 8.0;
    private static final double MOTOR_DRIVE_GEAR_TEETH = 24.0;
    private static final double GEAR_1_DRIVE_TEETH = 14.0;
    private static final double GEAR_1_DRIVEN_TEETH = 72.0;
    private static final double CANCODER_GEAR_RATIO = 1.0;
    public static final double GEAR_RATIO = 1
        / ((MOTOR_PINION_TEETH / MOTOR_DRIVE_GEAR_TEETH) * (GEAR_1_DRIVE_TEETH / GEAR_1_DRIVEN_TEETH));
    public static final double GEAR_RATIO_TO_CANCODER = GEAR_RATIO * CANCODER_GEAR_RATIO;
    public static final double PID_Kp = 0.1;
    public static final double PID_Ki = 0.0;
    public static final double PID_MaxV = 0.0;
    public static final double PID_MaxA = 0.0;

  }

//   public static class SHOOTER {
//     // public static final double ROTATE_GEAR_RATIO = 3*4*5;
//     // private static final double ROTATE_MOTOR_CNT_PER_REV = 42;
//     // private static final double ROTATE_SHAFT_CNTS_PER_REV =
//     // ROTATE_MOTOR_CNT_PER_REV * ROTATE_GEAR_RATIO;
//     // public static final double ROTATE_CNTS_PER_DEG = ROTATE_SHAFT_CNTS_PER_REV /
//     // 360;
//     private static final double SPIN_DRIVE_PULLEY_TEETH_COUNT = 24;
//     private static final double SPIN_DRIVEN_PULLEY_TEETH_COUNT = 18;
//     public static final double SPIN_PULLEY_RATIO = SPIN_DRIVEN_PULLEY_TEETH_COUNT / SPIN_DRIVE_PULLEY_TEETH_COUNT;
//     public static final double SPIN_VELOCITY_MAX_ROT_PER_SEC = 100;

//     public static final double SPIN_SPEED_HIGH_LONG = 0.68;
//     public static final double SPIN_SPEED_HIGH_SHORT = 0.55;

//     public static final double SPIN_SPEED_LOW = 0.38;
//     public static final double SPIN_SPEED_OFF = 0.0;

//     // public static double ROTATE_PID_KP = 0.05;//10.0;
//     // public static double ROTATE_PID_KI = 0.004;//4.0;
//     // public static double ROTATE_DS_OFFSET_DISTANCE_MM = 58;
//     // public static double ROTATE_DS_MAX = 170;
//     // public static double ROTATE_DS_SHOT1_MM = 25.0;
//     // public static double ROTATE_DS_SHOT2_MM = 50.0;
//     // public static double ROTATE_DS_SHOT3_MM = 75.0;
//     // public static double ROTATE_DS_SHOT4_MM = 100.0;
//     // public static double ROTATE_DS_SHOT5_MM = 125.0;

//   }

//   public static class INTAKE {
//     public static final double SPIN_SPEED_DEFAULT_VOLT = 5;
//     public static final double NOTE_CURRENT = 80;
//     public static final double INTAKE_RUNNING_CURRENT = 50;

//   }

//   public static class CLIMBER {
//     public static final double LIMIT_UP_ROTATIONS = -98;
//     public static final double LIMIT_DOWN_ROTATIONS = 74;
//   }

//   public static class AMP {
//     public static final double LIMPT_UP_ROTATION = 7.5;

//   }

}
