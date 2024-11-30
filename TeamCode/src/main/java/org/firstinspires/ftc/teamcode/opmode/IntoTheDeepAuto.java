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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.components.Drivetrain;
import org.firstinspires.ftc.teamcode.components.LiftArm;
import org.firstinspires.ftc.teamcode.components.PickArm;
import org.firstinspires.ftc.teamcode.components.Status;

@Config
@Autonomous(name = "IntoTheDeepAuto", group = "Into The Deep")
public class IntoTheDeepAuto extends LinearOpMode {

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
        Pose2d startingPose = new Pose2d(0, 0, -90);

        drive = new MecanumDrive(hardwareMap, startingPose);
        driveTrain = new Drivetrain(hardwareMap, gamepad1, drive);
        pickArm = new PickArm(hardwareMap,gamepad1,telemetry, driveTrain);
        liftArm = new LiftArm(hardwareMap,gamepad1,telemetry, driveTrain );
        status = new Status(telemetry, driveTrain, drive, pickArm, liftArm);

        //------------------------------------------------------------------------------------------
        // INITIALIZE ROBOT
        pickArm.init_pick_arm();
        liftArm.init_lift_arm();
        driveTrain.battering_ram_in();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        status.show_status();

        // Actions that need to happen on init; for instance, a claw tightening
//        Actions.runBlocking(claw.closeClaw());

        // Determine whether sample scoring or specimen scoring via button pressed

        int SAMPLE_SCORING = 1;
        int SPECIMEN_SCORING = 2;
//        int startPosition = SAMPLE_SCORING;
        int startPosition = SPECIMEN_SCORING;
//        if (gamepad1.a) {
//            startPosition = SAMPLE_SCORING;
//        } else if (gamepad1.b) {
//            startPosition = SPECIMEN_SCORING;
//        }
        //------------------------------------------------------------------------------------------


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        if (startPosition == SAMPLE_SCORING)
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
                    .lineToX(-23.5)
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
                    .strafeTo(new Vector2d(-17,51))
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



            // @formatter:on



//            Action trajectoryAction2 = drive.actionBuilder(drive.pose)
//                    .lineToY(37)
//                    .setTangent(Math.toRadians(0))
//                    .lineToX(18)
//                    .waitSeconds(3)
//                    .setTangent(Math.toRadians(0))
//                    .lineToXSplineHeading(46, Math.toRadians(180))
//                    .waitSeconds(3)
//                    .build();
//            Action trajectoryAction3 = drive.actionBuilder(drive.pose)
//                    .lineToYSplineHeading(33, Math.toRadians(180))
//                    .waitSeconds(2)
//                    .strafeTo(new Vector2d(46, 30))
//                    .waitSeconds(3)
//                    .build();
            // @formatter:on

            // @formatter:off
//            Actions.runBlocking(
//                    new SequentialAction(
//                            trajectoryAction1
//                            liftArm.lift_arm_to_high_basket_action(),
//                            trajectoryAction2
//                            liftArm.lify_arm_outtake_action()

//                            trajectoryAction2,
//                        lift.liftUp(),
//                        claw.openClaw(),
//                        lift.liftDown()
//                            trajectoryAction3
//                    )
//            );
            // @formatter:on

            Action trajectoryActionEnd = drive.actionBuilder(drive.pose)
                    .waitSeconds(3)
                    .build();

            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryActionEnd
                    )
            );
        }
        else if (startPosition == SPECIMEN_SCORING)
        {
            // @formatter:off
            Action trajectoryAction1 = drive.actionBuilder(drive.pose)
                    .lineToY(20)
                    .strafeTo(new Vector2d(-2, 20))
                    .build();
            // @formatter:on

            // @formatter:off
            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryAction1,
                            driveTrain.battering_ram_out_action(),
                            liftArm.high_rung_action()
                    )
                    );

            Action trajectoryAction2 = drive.actionBuilder(drive.pose)
                    .lineToY(30)
                    .build();
            // @formatter:on

            // @formatter:off
            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryAction2,
                            liftArm.low_rung_action()

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
