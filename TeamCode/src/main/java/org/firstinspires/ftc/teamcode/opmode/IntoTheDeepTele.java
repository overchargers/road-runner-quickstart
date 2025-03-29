package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.Constants.drive_high_power;
import static org.firstinspires.ftc.teamcode.Constants.drive_low_power;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.components.Drivetrain;
import org.firstinspires.ftc.teamcode.components.LiftArm;
import org.firstinspires.ftc.teamcode.components.PickArm;
import org.firstinspires.ftc.teamcode.components.Status;

@TeleOp(name = "IntoTheDeepTele", group = "Into The Deep")
@Disabled
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
        driveTrain = new Drivetrain(hardwareMap, gamepad1, drive, telemetry);
        pickArm = new PickArm(hardwareMap,gamepad1,telemetry, driveTrain);
        liftArm = new LiftArm(hardwareMap,gamepad1,telemetry, driveTrain );
        status = new Status(telemetry, driveTrain, drive, pickArm, liftArm);

        // Initialize
        pickArm.initPickArm();
        liftArm.initLiftArm();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        if (opModeIsActive()) {
            // Main loop
            while (opModeIsActive()) {
                status.show_status();
                if (gamepad1.dpad_right) {
                    pickArm.extend();
                    pickArm.pickArmSwingForward();
                    driveTrain.setDrivePower(drive_low_power);
                }
                if (gamepad1.dpad_left) {
                    pickArm.retract();
                    pickArm.pickArmSwingBackward();
                    driveTrain.setDrivePower(drive_high_power);
                }
                // Lifts arm to high basket setting and swings it ready for deposit.
                if (gamepad1.dpad_up) {
                    liftArm.liftArmToHighRung();
                    driveTrain.setDrivePower(drive_low_power);
                }
                // Lifts arm to high basket setting and swings it ready for deposit.
                if (gamepad1.dpad_down) {
                    liftArm.liftArmToBelowHighRung();
                    driveTrain.setDrivePower(drive_low_power);
                }
                if (gamepad1.y) {
                    liftArm.liftArmToHighBasket();
//                    liftArm.lift_arm_swing_to_high_basket();
                    driveTrain.setDrivePower(drive_low_power);
                }
                if (gamepad1.x) {
                    liftArm.liftArmToLowBasket();
//                    liftArm.lift_arm_swing_to_low_basket_and_zero();
                    driveTrain.setDrivePower(drive_low_power);
                }
                if (gamepad1.a) {
                    if (liftArm.getArmMotorPosition() > 1600) {
                        liftArm.setSwingServoPosition(Constants.LIFT_ARM_SWING_SERVO_TUCK);
                        sleep(500);
                    }
                    liftArm.liftArmToZero();
                    sleep(700);
//                    liftArm.lift_arm_swing_to_low_basket_and_zero();
                    driveTrain.setDrivePower(drive_high_power);
                }
                if (gamepad1.right_bumper) {
                    if (pickArm.pickArmWasIntaking) {
                        pickArm.pickArmOuttake();
                    } else {
                        pickArm.pickArmIntake();
                    }
                }
                if (gamepad1.left_bumper) {
                    if (liftArm.liftArmWasIntaking) {
                        liftArm.liftArmOuttake();
                    } else {
                        liftArm.liftArmIntake();
                    }
                }
                if (gamepad1.b) {
                    liftArm.setSwingServoPosition(Constants.LIFT_ARM_SWING_SERVO_TUCK);
                }
                if (gamepad1.ps) {
                    driveTrain.reverseDriveDirection();
                }
                driveTrain.driveMotor();
                pickArm.preventPickMotorOverheating();
                liftArm.preventLiftMotorOverheating();
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
