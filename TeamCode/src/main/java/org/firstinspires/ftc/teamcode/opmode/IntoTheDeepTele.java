package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.Constants.drive_high_power;
import static org.firstinspires.ftc.teamcode.Constants.drive_low_power;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.components.Drivetrain;
import org.firstinspires.ftc.teamcode.components.LiftArm;
import org.firstinspires.ftc.teamcode.components.PickArm;
import org.firstinspires.ftc.teamcode.components.Status;

@TeleOp(name = "IntoTheDeepTele", group = "Into The Deep")
public class IntoTheDeepTele extends LinearOpMode {
    final private ElapsedTime runtime = new ElapsedTime();
    LiftArm liftArm;
    PickArm pickArm;
    Drivetrain driveTrain;
    MecanumDrive drive;
    Status status;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        driveTrain = new Drivetrain(hardwareMap, gamepad1, drive);
        pickArm = new PickArm(hardwareMap,gamepad1,telemetry, driveTrain);
        liftArm = new LiftArm(hardwareMap,gamepad1,telemetry, driveTrain );
        status = new Status(telemetry, driveTrain, drive, pickArm, liftArm);

        // Initialize
        pickArm.init_pick_arm();
        liftArm.init_lift_arm();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        if (opModeIsActive()) {
            // Main loop
            while (opModeIsActive()) {
                status.show_status();
                if (gamepad1.dpad_right) {
                    pickArm.pick_arm_extend();
                    pickArm.pick_arm_swing_forward();
                    driveTrain.setDrivePower(drive_low_power);
                }
                if (gamepad1.dpad_left) {
                    pickArm.pick_arm_retract();
                    pickArm.pick_arm_swing_backward();
                    driveTrain.setDrivePower(drive_high_power);
                }
                // Lifts arm to high basket setting and swings it ready for deposit.
                if (gamepad1.dpad_up) {
                    liftArm.lift_arm_to_high_rung();
                    driveTrain.setDrivePower(drive_low_power);
                }
                // Lifts arm to high basket setting and swings it ready for deposit.
                if (gamepad1.dpad_down) {
                    liftArm.lift_arm_to_below_high_rung();
                    driveTrain.setDrivePower(drive_low_power);
                }
                if (gamepad1.y) {
                    liftArm.lift_arm_to_high_basket();
                    liftArm.lift_arm_swing_to_high_basket();
                    driveTrain.setDrivePower(drive_low_power);
                }
                if (gamepad1.x) {
                    liftArm.lift_arm_to_low_basket();
                    liftArm.lift_arm_swing_to_low_basket_and_zero();
                    driveTrain.setDrivePower(drive_low_power);
                }
                if (gamepad1.a) {
                    if (liftArm.liftMotor.getCurrentPosition() > 1600) {
                        liftArm.liftswingservo.setPosition(Constants.lift_arm_swing_servo_tuck);
                        sleep(500);
                    }
                    liftArm.lift_arm_to_zero();
                    sleep(700);
                    liftArm.lift_arm_swing_to_low_basket_and_zero();
                    driveTrain.setDrivePower(drive_high_power);
                }
                if (gamepad1.right_bumper) {
                    if (pickArm.pick_arm_was_intaking) {
                        pickArm.pick_arm_outtake();
                    } else {
                        pickArm.pick_arm_intake();
                    }
                }
                if (gamepad1.left_bumper) {
                    if (liftArm.lift_arm_was_intaking) {
                        liftArm.lift_arm_outtake();
                    } else {
                        liftArm.lift_arm_intake();
                    }
                }
                if (gamepad1.b) {
                    liftArm.liftswingservo.setPosition(Constants.lift_arm_swing_servo_tuck);
                }
                if (gamepad1.ps) {
                    driveTrain.reverse_drive_direction();
                }
                driveTrain.drive_motor();
                pickArm.prevent_pick_motor_overheating();
                liftArm.prevent_lift_motor_overheating();
            }
        }
        runtime.reset();

//        while (opModeIsActive())
//        {
//
//            // Display telemetry on the FTC dashboard
//            TelemetryPacket packet = new TelemetryPacket();
//            packet.fieldOverlay().setStroke("#3F51B5");
//            Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
//            FtcDashboard.getInstance().sendTelemetryPacket(packet);
//        }

    }
}