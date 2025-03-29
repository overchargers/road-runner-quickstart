package org.firstinspires.ftc.teamcode.components;

import static java.lang.Thread.sleep;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class LiftArm extends Arm
{
    public boolean liftArmWasIntaking = false;
    private final Gamepad gamepad;
    private final Telemetry telemetry;
    private Servo clip;
    private final Drivetrain drivetrain;

    public LiftArm(HardwareMap hardwareMap, Gamepad gamepad, Telemetry telemetry, Drivetrain drivetrain)
    {
        super(hardwareMap, Constants.LIFT_ARM_MOTOR_NAME, Constants.LIFT_ARM_SWING_SERVO_NAME, Constants.LIFT_ARM_SPIN_SERVO_NAME);
        clip = hardwareMap.get(Servo.class, Constants.LIFT_ARM_CLIP_SERVO_NAME);
        this.gamepad = gamepad;
        this.telemetry = telemetry;
        this.drivetrain = drivetrain;
    }

    public void initLiftArm()
    {
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setDirection(DcMotor.Direction.REVERSE);
        swingServo.setPosition(Constants.LIFT_ARM_SWING_SERVO_TUCK);
    }

    public void addTelemetryData()
    {
        telemetry.addData("LiftMotor Power", armMotor.getPower());
        telemetry.addData("LiftMotor Position", armMotor.getCurrentPosition());
        telemetry.addData("LiftMotor Velocity", ((DcMotorEx) armMotor).getVelocity());
        telemetry.addData("LiftSwing Position", swingServo.getPosition());

    }

    public void preventLiftMotorOverheating()
    {
        if (((DcMotorEx) armMotor).getVelocity() == 0 && armMotor.getTargetPosition() > 100)
        {
            armMotor.setPower(0.1);
        }
        if (((DcMotorEx) armMotor).getVelocity() == 0 && armMotor.getTargetPosition() < 100)
        {
            armMotor.setPower(0);
        }
    }

    public void closeClip()
    {
        clip.setPosition(Constants.LIFT_ARM_CLIP_CLOSED_POSITION);
    }

    public Action closeClipAction()
    {
        return new Action()
        {
            @Override
            public boolean run(@NonNull TelemetryPacket packet)
            {
                closeClip();
                return false;
            }
        };
    }

    public void openClip()
    {
        clip.setPosition(Constants.LIFT_ARM_CLIP_OPEN_POSITION);
    }

    public Action openClipAction()
    {
        return new Action()
        {
            @Override
            public boolean run(@NonNull TelemetryPacket packet)
            {
                openClip();
                return false;
            }
        };
    }

    public void liftArmToHighBasket()
    {
        double power = 1.0;
        moveArmMotor(Constants.LIFT_ARM_MOTOR_HIGH_BASKET, power);
    }

    public void liftArmToLowBasket()
    {
        double power = 1.0;
        moveArmMotor(Constants.LIFT_ARM_MOTOR_LOW_BASKET, power);
    }

    public void liftArmToZero()
    {
        double power = 1.0;
        int target = 0;
        moveArmMotor(target, power);
    }

    public void liftArmToZeroSpecimen()
    {
        double power = 1;
        int target = 0;
        moveArmMotor(target, power);
    }

    public void liftArmOuttake()
    {
        while (gamepad.left_bumper)
        {
            spinServo.setPower(-0.5);
            telemetry.update();
            drivetrain.driveMotor();
        }
        spinServo.setPower(0);
        liftArmWasIntaking = false;
    }

    public void liftArmIntake()
    {
        spinServo.setDirection(CRServo.Direction.FORWARD);
        while (gamepad.left_bumper)
        {
            spinServo.setPower(0.4);
            telemetry.update();
            drivetrain.driveMotor();
        }
        spinServo.setPower(0.1);
        liftArmWasIntaking = true;
    }

    public void liftArmToHighRung()
    {
        double power = 1.0;
        moveArmMotor(Constants.LIFT_ARM_MOTOR_HIGH_RUNG, power);
    }

    public void liftArmToBelowHighRung()
    {
        double power = 1.0;
        moveArmMotor(Constants.LIFT_ARM_MOTOR_BELOW_HIGH_RUNG, power);
    }

    public void liftArmSwingToLowBasket()
    {
        swingServo.setPosition(Constants.LIFT_ARM_SWING_SERVO_LOW_BASKET);
    }

    public void liftArmSwingToHighBasket()
    {
        swingServo.setPosition(Constants.LIFT_ARM_SWING_SERVO_HIGH_BASKET);
    }

    public Action liftArmToHighBasketAction()
    {
        return new Action()
        {
            @Override
            public boolean run(@NonNull TelemetryPacket packet)
            {
                liftArmToHighBasket();
                liftArmSwingToHighBasket();
                return false;
            }
        };
    }

    public Action liftArmToZeroAction()
    {
        return new Action()
        {
            @Override
            public boolean run(@NonNull TelemetryPacket packet)
            {
                liftArmToZero();
                return false;
            }
        };
    }

    public Action liftArmToZeroAndSwingToLowBasketAction()
    {
        return new Action()
        {
            @Override
            public boolean run(@NonNull TelemetryPacket packet)
            {
                liftArmToZero();
                liftArmSwingToLowBasket();
                return false;
            }
        };
    }

    // intake
    public Action lifyArmIntakeAction()
    {
        return new Action()
        {
            @Override
            public boolean run(@NonNull TelemetryPacket packet)
            {
                spinServo.setPower(0.4);
                try
                {
                    sleep(250);
                } catch (InterruptedException e)
                {
                    throw new RuntimeException(e);
                }
                spinServo.setPower(0.1);
                return false;
            }
        };
    }

    public Action liftArmOuttakeAction()
    {
        return new Action()
        {
            @Override
            public boolean run(@NonNull TelemetryPacket packet)
            {
                spinServo.setPower(-0.5);
                try
                {
                    sleep(250);
                } catch (InterruptedException e)
                {
                    throw new RuntimeException(e);
                }
                spinServo.setPower(0);
                return false;
            }
        };
    }

    public Action highRungAction()
    {
        return new Action()
        {
            @Override
            public boolean run(@NonNull TelemetryPacket packet)
            {
                liftArmToHighRung();
                return false;
            }
        };
    }

    public Action lowRungAction()
    {
        return new Action()
        {
            @Override
            public boolean run(@NonNull TelemetryPacket packet)
            {
                liftArmToBelowHighRung();
                return false;
            }
        };
    }

}
