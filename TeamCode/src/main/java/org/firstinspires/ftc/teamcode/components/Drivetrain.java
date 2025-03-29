package org.firstinspires.ftc.teamcode.components;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public class Drivetrain
{
    private MecanumDrive drive;
    private Gamepad gamepad;
    private double drivePower;
    private boolean directionIsReverse;
    private Servo batteringRamServo;
    private Telemetry telemetry;

    public Drivetrain(HardwareMap hardwareMap, Gamepad gamepad, MecanumDrive drive, Telemetry telemetry)
    {
        this.gamepad = gamepad;
        this.drive = drive;
        this.telemetry = telemetry;
        batteringRamServo = hardwareMap.get(Servo.class, Constants.BATTERING_RAM_SERVO_NAME);
    }

    public void addTelemetryData()
    {
        telemetry.addData("FrontRight Power", drive.rightFront.getPower());
        telemetry.addData("BackRight Power", drive.rightBack.getPower());
        telemetry.addData("FrontLeft Power", drive.leftFront.getPower());
        telemetry.addData("BackLeft Power", drive.leftBack.getPower());
        telemetry.addData("MotorDirection Reverse", directionIsReverse);
    }

    public void driveMotor()
    {
        int reverse_turn;

        if (directionIsReverse)
        {
            reverse_turn = -1;
        }
        else
        {
            reverse_turn = 1;
        }

        // Drive by gamepad
        drive.setDrivePowers(
                new PoseVelocity2d(new Vector2d(-gamepad.left_stick_y, -gamepad.left_stick_x), -gamepad.right_stick_x));
        drive.updatePoseEstimate();

        // Display telemetry on the driver hub
        /*
         * telemetry.addData("x", drive.pose.position.x); telemetry.addData("y", drive.pose.position.y);
         * telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
         * telemetry.update();
         */

    }

    public void setDrivePower(double drivePower)
    {
        this.drivePower = drivePower;
    }

    public void reverseDriveDirection()
    {
        directionIsReverse = !directionIsReverse;
    }

    public void batteringRamIn()
    {
        batteringRamServo.setPosition(Constants.BATTERING_RAM_IN);
    }

    public Action batteringRamInAction()
    {
        return new Action()
        {
            @Override
            public boolean run(@NonNull TelemetryPacket packet)
            {
                batteringRamIn();
                return false;
            }
        };
    }

    private void batteringRamOut()
    {
        batteringRamServo.setPosition(Constants.BATTERING_RAM_OUT);
    }

    public Action batteringRamOutAction()
    {
        return new Action()
        {
            @Override
            public boolean run(@NonNull TelemetryPacket packet)
            {
                batteringRamOut();
                return false;
            }
        };
    }

}
