package org.firstinspires.ftc.teamcode.components;

import static org.firstinspires.ftc.teamcode.Constants.PICK_ARM_SWING_SERVO_BACKWARD;
import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class PickArm extends Arm
{
    public boolean pickArmWasIntaking = false;
    private Gamepad gamepad;
    private Telemetry telemetry;
    private Drivetrain drivetrain;


    public PickArm(HardwareMap hardwareMap, Gamepad gamepad, Telemetry telemetry, Drivetrain drivetrain)
    {
        super(hardwareMap, Constants.PICK_ARM_MOTOR_NAME, Constants.LIFT_ARM_SWING_SERVO_NAME, Constants.LIFT_ARM_SPIN_SERVO_NAME);
        this.gamepad = gamepad;
        this.telemetry = telemetry;
        this.drivetrain = drivetrain;
    }

    public void initPickArm()
    {
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setDirection(DcMotor.Direction.REVERSE);
        swingServo.setPosition(Constants.PICK_ARM_SWING_SERVO_TUCK);
    }

    public void addTelemetryData()
    {
        telemetry.addData("PickMotor Power", armMotor.getPower());
        telemetry.addData("PickMotor Encoder", armMotor.getCurrentPosition());
        telemetry.addData("PickSwing Position", swingServo.getPosition());
    }

    public void preventPickMotorOverheating()
    {
        if (((DcMotorEx) armMotor).getVelocity() == 0)
        {
            armMotor.setPower(0);
        }
    }

    public void extend()
    {
        double power = 1.0;
        moveArmMotor(Constants.PICK_ARM_MOTOR_EXTEND, power);
    }

    public void retract()
    {
        double power = 1.0;
        moveArmMotor(0, power);
    }

    public void pickArmOuttake()
    {
        while (gamepad.right_bumper)
        {
            spinServo.setPower(-0.5);
            drivetrain.driveMotor();
        }
        spinServo.setPower(0);
        pickArmWasIntaking = false;
    }

    public void pickArmIntake()
    {
        spinServo.setDirection(CRServo.Direction.FORWARD);
        while (gamepad.right_bumper)
        {
            spinServo.setPower(0.3);
            drivetrain.driveMotor();
        }
        spinServo.setPower(0.1);
        pickArmWasIntaking = true;
    }

    public void pickArmSwingBackward()
    {
        swingServo.setPosition(PICK_ARM_SWING_SERVO_BACKWARD);
    }

    public void pickArmSwingForward()
    {
        swingServo.setPosition(Constants.PICK_ARM_SWING_SERVO_FORWARD);
    }

}
