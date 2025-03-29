package org.firstinspires.ftc.teamcode;

public final class Constants
{
    public static final String SAMPLE_SCORING = "SAMPLE-SCORING";
    public static final String SPECIMEN_SCORING = "SPECIMEN-SCORING";
    public static final double backward_drive_power = 0.2;

    //================================================================================
    // Hardware Map
    //================================================================================
    public static final String PICK_ARM_MOTOR_NAME = "pickMotor";
    public static final String PICK_ARM_SWING_SERVO_NAME = "pickswingservo";
    public static final String PICK_ARM_SPIN_SERVO_NAME = "pickspinservo";
    public static final String LIFT_ARM_MOTOR_NAME = "liftMotor";
    public static final String LIFT_ARM_SWING_SERVO_NAME = "liftswingservo";
    public static final String LIFT_ARM_SPIN_SERVO_NAME = "liftspinservo";
    public static final String LIFT_ARM_CLIP_SERVO_NAME = "clip";
    public static final String BATTERING_RAM_SERVO_NAME = "batteringramservo";

    //================================================================================
    // Drivetrain
    //================================================================================
    public static final double drive_low_power = 0.2;
    public static final double drive_high_power = 0.9;
    public static final double BATTERING_RAM_OUT = 0.37;
    public static final double BATTERING_RAM_IN = 0.73;
    // path profile parameters for sample pickup auto (in inches)
    public static final double maxWheelVel = 80;
    public static final double maxProfileAccel = 50;

    //================================================================================
    // Pick Arm
    //================================================================================
    public static final double PICK_ARM_SWING_SERVO_TUCK = 0.2;
    public static final double PICK_ARM_SWING_SERVO_FORWARD = 0.77;
    public static final double PICK_ARM_SWING_SERVO_BACKWARD = 0.5;
    public static final int PICK_ARM_MOTOR_EXTEND = 1200;

    //================================================================================
    // Lift Arm
    //================================================================================
    public static final int LIFT_ARM_MOTOR_HIGH_BASKET = 3100;
    public static final int LIFT_ARM_MOTOR_LOW_BASKET = 2300;
    public static final int LIFT_ARM_MOTOR_HIGH_RUNG = 1500;
    public static final int LIFT_ARM_MOTOR_BELOW_HIGH_RUNG = 875;
    public static final double LIFT_ARM_SWING_SERVO_HIGH_BASKET = 0.55;
    public static final double LIFT_ARM_SWING_SERVO_LOW_BASKET = 0.79;
    public static final double LIFT_ARM_SWING_SERVO_TUCK = 0.3;
    public static final double LIFT_ARM_CLIP_CLOSED_POSITION = 0.5;
    public static final double LIFT_ARM_CLIP_OPEN_POSITION = 0.23;
}
