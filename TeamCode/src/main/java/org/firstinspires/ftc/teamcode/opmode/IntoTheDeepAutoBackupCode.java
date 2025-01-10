package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.Constants.battering_ram_is_out;
import static org.firstinspires.ftc.teamcode.Constants.drive_low_power;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.components.Drivetrain;
import org.firstinspires.ftc.teamcode.components.LiftArm;
import org.firstinspires.ftc.teamcode.components.PickArm;
import org.firstinspires.ftc.teamcode.components.Status;

@Config
@Autonomous(name = "IntoTheDeepAutoBackUpCode", group = "Into The Deep")
@Disabled
public class IntoTheDeepAutoBackupCode extends LinearOpMode {

    final private ElapsedTime runtime = new ElapsedTime();
    LiftArm liftArm;
    PickArm pickArm;
    Drivetrain driveTrain;
    MecanumDrive drive;
    Status status;

    @Override
    public void runOpMode() throws InterruptedException {
        // sample starting pose
//        Pose2d startingPose = new Pose2d(0, 0, 0);
        // specimen starting pose
        driveTrain = new Drivetrain(hardwareMap, gamepad1, drive);
        pickArm = new PickArm(hardwareMap,gamepad1,telemetry, driveTrain);
        liftArm = new LiftArm(hardwareMap,gamepad1,telemetry, driveTrain );
        status = new Status(telemetry, driveTrain, drive, pickArm, liftArm);

        //------------------------------------------------------------------------------------------
        // INITIALIZE ROBOT
        pickArm.init_pick_arm();
        liftArm.init_lift_arm();
        driveTrain.battering_ram_in();

        // Actions that need to happen on init; for instance, a claw tightening
//        Actions.runBlocking(claw.closeClaw());

        // Determine whether sample scoring or specimen scoring via button pressed

        String startPosition = "";
        while (!opModeIsActive())
        {
            if (gamepad1.a) {
                Pose2d startingPose = new Pose2d(0, 0, 0);
                drive = new MecanumDrive(hardwareMap, startingPose);
                startPosition = Constants.SAMPLE_SCORING;
            }
            if (gamepad1.b) {
                Pose2d startingPose = new Pose2d(0, 0, -90);
                drive = new MecanumDrive(hardwareMap, startingPose);
                startPosition = Constants.SPECIMEN_SCORING;
            }
            telemetry.addData("Starting Position", startPosition);
            telemetry.update();
            idle();
        }
        //------------------------------------------------------------------------------------------


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        if (startPosition.equals(Constants.SAMPLE_SCORING))
        {
            // @formatter:off
            Action trajectoryAction1 = drive.actionBuilder(drive.pose)
                    .lineToX(-16)
                    .build();

            // @formatter:off
            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryAction1,
                            liftArm.lift_arm_to_high_basket_action()
                    )
            );
            // @formatter:on

            Action trajectoryAction2 = drive.actionBuilder(drive.pose)
                    .waitSeconds(1.25)
                    .lineToX(-22)
                    .waitSeconds(1)
                    .build();

            // @formatter:off
            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryAction2,
                            liftArm.lift_arm_outtake_action()
                    )
            );
            Action trajectoryAction3 = drive.actionBuilder(drive.pose)
                    .lineToX(-2)
                    .build();

            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryAction3,
                            liftArm.lift_arm_to_zero_action()
                    )
            );
            Action trajectoryAction4 = drive.actionBuilder(drive.pose)
                    .strafeTo(new Vector2d(-5,40))
                    .build();

            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryAction4,
                            liftArm.lify_arm_intake_action()
                    )
            );

            Action trajectoryAction5 = drive.actionBuilder(drive.pose)
                    .lineToX(-12)
                    .waitSeconds(.5)
                    .strafeTo(new Vector2d(-12,-2))
                    .build();

            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryAction5,
                            liftArm.lift_arm_to_high_basket_action()

                    )
            );
            Action trajectoryAction6 = drive.actionBuilder(drive.pose)
                    .waitSeconds(1.25)
                    .lineToX(-23)
                    .waitSeconds(1)
                    .build();

            // @formatter:off
            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryAction6,
                            liftArm.lift_arm_outtake_action()
                    )
            );
            Action trajectoryAction7 = drive.actionBuilder(drive.pose)
                    .lineToX(-17)
                    .build();

            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryAction7,
                            liftArm.lift_arm_to_zero_action()
                    )
            );
            Action trajectoryAction8 = drive.actionBuilder(drive.pose)
                    .strafeTo(new Vector2d(-17,52.5))
                    .build();

            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryAction8
                    )
            );
            Action trajectoryAction9 = drive.actionBuilder(drive.pose)
                    .lineToX(10)
                    .build();

            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryAction9
                    )
            );

            Action trajectoryActionEnd = drive.actionBuilder(drive.pose)
                    .waitSeconds(3)
                    .build();

            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryActionEnd
                    )
            );
        }
        else if (startPosition.equals(Constants.SPECIMEN_SCORING))
        {
            // @formatter:off
            Action trajectoryAction1 = drive.actionBuilder(drive.pose)
//                    .strafeTo(new Vector2d(-2, 0))
//                    .setTangent(Math.toRadians(-90))
                    .lineToY(20)
                    .build();
            // @formatter:on

            // @formatter:off
            Actions.runBlocking(
                    new SequentialAction(
                            driveTrain.battering_ram_out_action(),
                            liftArm.high_rung_action(),
                            trajectoryAction1

                    )
            );
            Action trajectoryAction2 = drive.actionBuilder(drive.pose)
                    .lineToY(27)
                    .build();
            // @formatter:on

            // @formatter:off
            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryAction2
                    )
            );

            Action trajectoryAction3 = drive.actionBuilder(drive.pose)
                    .waitSeconds(0.5)
                    .build();
            // @formatter:on

            // @formatter:off
            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryAction3,
                            liftArm.low_rung_action()
                    )
            );
            Action trajectoryAction4 = drive.actionBuilder(drive.pose)
                    .waitSeconds(0.5)
                    .lineToY(5)
                    .turn(Math.toRadians(195))
                    .build();
            // @formatter:on

            // @formatter:off
            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryAction4
                    )
            );
            Action trajectoryAction5 = drive.actionBuilder(drive.pose)
                    .strafeTo(new Vector2d(58, -16))
                    .build();
            // @formatter:on

            // @formatter:off
            Actions.runBlocking(
                    new SequentialAction(
                            liftArm.lift_arm_to_zero_action(),
                            driveTrain.battering_ram_in_action(),
                            trajectoryAction5

                    )
            );
            Action trajectoryAction6 = drive.actionBuilder(drive.pose)
                    .lineToY(-31)
                    .build();
            // @formatter:on

            // @formatter:off
            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryAction6,
                            liftArm.high_rung_action()
                    )
            );
            Action trajectoryAction7 = drive.actionBuilder(drive.pose)
                    .waitSeconds(0.5)
                    .lineToY(-15)
                    .strafeTo(new Vector2d(0, 15))
                    .turn(Math.toRadians(190))
                    .build();
            // @formatter:on

            // @formatter:off
            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryAction7,
                            driveTrain.battering_ram_out_action()
                    )
            );
            Action trajectoryAction8 = drive.actionBuilder(drive.pose)
                    .lineToY(26)
                    .lineToY(30)
//                    .waitSeconds(1)
                    .build();
            // @formatter:on

            // @formatter:off
            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryAction8,
                            liftArm.low_rung_action()
                    )
            );
            Action trajectoryAction9 = drive.actionBuilder(drive.pose)
                    .lineToY(0)
                    .strafeTo(new Vector2d(56, -18))
                    .build();
            // @formatter:on

            // @formatter:off
            Actions.runBlocking(
                    new SequentialAction(
                            liftArm.lift_arm_to_zero_action(),
                            driveTrain.battering_ram_in_action(),
                            trajectoryAction9
                    )
            );

            Action trajectoryActionEnd = drive.actionBuilder(drive.pose)
                    .waitSeconds(3)
                    .build();

            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryActionEnd
                    )
            );
        }
    }
}