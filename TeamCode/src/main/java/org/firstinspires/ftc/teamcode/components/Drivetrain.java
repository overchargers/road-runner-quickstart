package org.firstinspires.ftc.teamcode.components;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class Drivetrain {
    MecanumDrive drive;
    Gamepad gamepad1;
    double drivePower;
    boolean direction_is_reverse;

    public Drivetrain(HardwareMap hardwareMap, Gamepad gp, MecanumDrive md){
        gamepad1 = gp;
        drive = md;
    }
    public void drive_motor(){
        int reverse_turn;

        if (direction_is_reverse) {
            reverse_turn = -1;
        } else {
            reverse_turn = 1;
        }

        // Drive by gamepad
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
            -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x
                ),
                -gamepad1.right_stick_x
        ));
        drive.updatePoseEstimate();

    // Display telemetry on the driver hub
        /*
        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
        telemetry.update();
         */

    }
    public void setDrivePower(double drivePower){
        this.drivePower = drivePower;
    }
    public void reverse_drive_direction() {
        direction_is_reverse = !direction_is_reverse;
    }
}
