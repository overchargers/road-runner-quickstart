package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.Constants.battering_ram_is_out;
import static org.firstinspires.ftc.teamcode.Constants.clip_open;
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

import org.firstinspires.ftc.teamcode.Constants;
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
            //Moves forward and raises liftarm for first sample
            Action trajectoryAction1 = drive.actionBuilder(drive.pose, Constants.maxWheelVel, Constants.maxProfileAccel)
                    .build();
            Actions.runBlocking(
                    new SequentialAction(
                            liftArm.lift_arm_to_high_basket_action(),
                            trajectoryAction1
                    )
            );

            // Move forward till over the basket then outtakes sample into basket
            Action trajectoryAction2 = drive.actionBuilder(drive.pose, Constants.maxWheelVel, Constants.maxProfileAccel)
                    .waitSeconds(1)
                    .lineToX(-22.45)
                    .waitSeconds(2)
                    .build();
            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryAction2,
                            liftArm.lift_arm_outtake_action()
                    )
            );

            //When arm is up, moves back and lowers
            Action trajectoryAction3 = drive.actionBuilder(drive.pose, Constants.maxWheelVel, Constants.maxProfileAccel)
                    .lineToX(-1)
                    .build();
            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryAction3,
                            liftArm.lift_arm_to_zero_and_swing_to_low_basket_action()
                    )
            );

            //Strafes to second sample then starts intaking
            Action trajectoryAction4 = drive.actionBuilder(drive.pose, Constants.maxWheelVel, Constants.maxProfileAccel)
                    .strafeTo(new Vector2d(-5,40.55))
                    .build();
            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryAction4,
                            liftArm.lify_arm_intake_action()
                    )
            );
            //Moves forward into sample then strafes back to starting position and lifts basket
            Action trajectoryAction5 = drive.actionBuilder(drive.pose, Constants.maxWheelVel, Constants.maxProfileAccel)
                    .lineToX(-12)
                    .waitSeconds(.5)
                    .build();
            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryAction5,
                            liftArm.lift_arm_to_high_basket_action()


                    )
            );
            Action trajectoryAction6 = drive.actionBuilder(drive.pose, Constants.maxWheelVel, Constants.maxProfileAccel)
                    .waitSeconds(.2)
                    .strafeTo(new Vector2d(-12,-3))
                    .build();
            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryAction6


                    )
            );

            //Moves into basket then outtakes
            Action trajectoryAction7 = drive.actionBuilder(drive.pose, Constants.maxWheelVel, Constants.maxProfileAccel)
                    .lineToX(-22.4)
                    .waitSeconds(1.8)
                    .build();
            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryAction7,
                            liftArm.lift_arm_outtake_action()
                    )
            );
            //Moves back and lift arm to zero
            Action trajectoryAction8 = drive.actionBuilder(drive.pose, Constants.maxWheelVel, Constants.maxProfileAccel)
                    .lineToX(-14)
                    .build();
            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryAction8,
                            liftArm.lift_arm_to_zero_and_swing_to_low_basket_action()
                    )
            );
            //Strafes to y position of second sample
            Action trajectoryAction9 = drive.actionBuilder(drive.pose, Constants.maxWheelVel, Constants.maxProfileAccel)

                    .strafeTo(new Vector2d(-14,40.25))
                    .build();
            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryAction9,
                            liftArm.lify_arm_intake_action()
                    )
            );
            //It moves into third sample and lifts it up
            Action trajectoryAction10 = drive.actionBuilder(drive.pose, Constants.maxWheelVel, Constants.maxProfileAccel)
                    .lineToX(-22.75)
                    .waitSeconds(0.8)
//                    .lineToX(-12)
//                    .strafeTo(new Vector2d(-12,-2))
//                    .waitSeconds(0.5)
                    .build();

            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryAction10,
                            liftArm.lift_arm_to_high_basket_action()
                    )
            );
            //Waits so high basket can lift
            Action trajectoryAction11 = drive.actionBuilder(drive.pose, Constants.maxWheelVel, Constants.maxProfileAccel)
                    .waitSeconds(0.5)
//                    .lineToX(-12)
//                    .strafeTo(new Vector2d(-12,-2))
//                    .waitSeconds(0.5)
                    .build();
            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryAction11
                    )
            );
            //Strafes straight to high basket and outtakes
            Action trajectoryAction12 = drive.actionBuilder(drive.pose, Constants.maxWheelVel, Constants.maxProfileAccel)
                    .strafeTo(new Vector2d(-22.75,-3))
                    .waitSeconds(1.4)
                    .build();

            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryAction12,
                            liftArm.lift_arm_outtake_action()
                    )
            );
//Moves back and resets arm
            Action trajectoryAction13 = drive.actionBuilder(drive.pose, Constants.maxWheelVel, Constants.maxProfileAccel)
                    .lineToX(-14)
                    .waitSeconds(0.3)
                    .build();
            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryAction13,
                            liftArm.lift_arm_to_zero_and_swing_to_low_basket_action()
                    )
            );
            //waits 3 second for program to end
            Action trajectoryActionEnd = drive.actionBuilder(drive.pose, Constants.maxWheelVel, Constants.maxProfileAccel)
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
                            liftArm.close_clip_action(),
                            driveTrain.battering_ram_out_action(),
                            liftArm.high_rung_action(),
                            trajectoryAction1

                    )
            );
            Action trajectoryAction2 = drive.actionBuilder(drive.pose)
                    .lineToY(25.5)
                    .build();
            // @formatter:on

            // @formatter:off
            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryAction2
                    )
            );

            Action trajectoryAction3 = drive.actionBuilder(drive.pose)
                    .waitSeconds(0.45)
                    .build();
            // @formatter:on

            // @formatter:off
            Actions.runBlocking(
                    new SequentialAction(
                            liftArm.low_rung_action(),
                            trajectoryAction3,
                            liftArm.open_clip_action()
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
            Action trajectoryAction5 = drive.actionBuilder(drive.pose, Constants.maxWheelVel, Constants.maxProfileAccel)
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
                    .lineToY(-25.5)
                    .waitSeconds(0.4)
                    .build();
            // @formatter:on

            // @formatter:off
            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryAction6,
                            liftArm.close_clip_action()
                    )
            );
            Action trajectoryAction7 = drive.actionBuilder(drive.pose)
                    .waitSeconds(0.3)
                    .build();
            // @formatter:on

            // @formatter:off
            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryAction7,
                            liftArm.high_rung_action()
                    )
            );
            Action trajectoryAction8 = drive.actionBuilder(drive.pose, Constants.maxWheelVel, Constants.maxProfileAccel)
                    .waitSeconds(0.5)
                    .lineToY(-15)
                    .strafeTo(new Vector2d(0, 15))
                    .turn(Math.toRadians(190))
                    .build();
            // @formatter:on

            // @formatter:off
            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryAction8,
                            driveTrain.battering_ram_out_action()
                    )
            );
            Action trajectoryAction9 = drive.actionBuilder(drive.pose)
                    .lineToY(26)
                    .lineToY(30)
                    .waitSeconds(1)
//                    .waitSeconds(1)
                    .build();
            // @formatter:on

            // @formatter:off
            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryAction9,
                            liftArm.low_rung_action()
                    )
            );
            Action trajectoryAction10 = drive.actionBuilder(drive.pose)
                    .waitSeconds(.45)
                    .build();
            // @formatter:on

            // @formatter:off
            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryAction10,
                            liftArm.open_clip_action()
                    )
            );


            Action trajectoryAction11 = drive.actionBuilder(drive.pose, Constants.maxWheelVel, Constants.maxProfileAccel)
//                    .waitSeconds(0.5)
                    .lineToY(16)
                    .strafeTo(new Vector2d(44, 16))
                    .setTangent(Math.toRadians(90))
                    .lineToY(33)
                    .strafeTo(new Vector2d(55, 30))
                    .setTangent(Math.toRadians(-90))
//                    .lineToY(-17)
                    .strafeTo(new Vector2d(43, -20))
//                    .waitSeconds(1)
                    .build();
            // @formatter:on

            // @formatter:off
            Actions.runBlocking(
                    new SequentialAction(
                            liftArm.lift_arm_to_zero_action(),
                            driveTrain.battering_ram_in_action(),
                            trajectoryAction11


                    )
            );

//            Action trajectoryAction9 = drive.actionBuilder(drive.pose, Constants.maxWheelVel, Constants.maxProfileAccel)
//                    .lineToY(0)
//                    .strafeTo(new Vector2d(56, -18))
//                    .build();
//            // @formatter:on
//
//            // @formatter:off
//            Actions.runBlocking(
//                    new SequentialAction(
//                            liftArm.lift_arm_to_zero_action(startPosition),
//                            driveTrain.battering_ram_in_action(),
//                            trajectoryAction9
//                    )
//            );

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