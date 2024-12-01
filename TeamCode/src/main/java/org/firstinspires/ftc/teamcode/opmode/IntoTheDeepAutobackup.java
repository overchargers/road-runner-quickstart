package org.firstinspires.ftc.teamcode.opmode;

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

@Config
@Autonomous(name = "IntoTheDeepAutoBackup", group = "Into The Deep")
public class IntoTheDeepAutobackup extends LinearOpMode {

    final private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode()
    {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(-90)));
//        PickArm pickarm = new PickArm(hardwareMap);
//        LiftArm liftarm = new LiftArm(hardwareMap);

        //------------------------------------------------------------------------------------------
        // INITIALIZE ROBOT

        // Actions that need to happen on init; for instance, a claw tightening
//        Actions.runBlocking(claw.closeClaw());

        // Determine whether sample scoring or specimen scoring via button pressed
        int SAMPLE_SCORING = 1;
        int SPECIMEN_SCORING = 2;
        int startPosition = SAMPLE_SCORING;

        //------------------------------------------------------------------------------------------

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        if (startPosition == SAMPLE_SCORING)
        {
            // @formatter:off
            Action trajectoryAction1 = drive.actionBuilder(drive.pose)
                    .strafeTo(new Vector2d(-23.65,0))
                    .setTangent(Math.toRadians(-90))
                    .lineToY(50)
                    // push 1st sample
                   .strafeTo(new Vector2d(-35,50))
                    .setTangent(Math.toRadians(-90))
                    .lineToY(0)
                    .lineToY(50)
                    // push 2nd sample
                    .strafeTo(new Vector2d(-45,50))
                    .setTangent(Math.toRadians(-90))
                    .lineToY(10)
                    .lineToY(50)
                    // push last sample
                    .strafeTo(new Vector2d(-49,50))
                    .setTangent(Math.toRadians(-90))
                    .lineToY(15)
                    .lineToY(50)
                    // park captain hook!
                    .strafeTo(new Vector2d(-6,50))

                    .build();
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
            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryAction1
//                            trajectoryAction2,
//                        lift.liftUp(),
//                        claw.openClaw(),
//                        lift.liftDown()
//                            trajectoryAction3
                    )
            );
            // @formatter:on

        }
//        else if (startPosition == SPECIMEN_SCORING)
//        {
//            // @formatter:off
//            Action trajectoryAction1 = drive.actionBuilder(drive.pose)
//                    .lineToYSplineHeading(33, Math.toRadians(0))
//                    .waitSeconds(2)
//                    .setTangent(Math.toRadians(90))
//                    .lineToY(48)
//                    .setTangent(Math.toRadians(0))
//                    .lineToX(32)
//                    .strafeTo(new Vector2d(44.5,30))
//                    .turn(Math.toRadians(180))
//                    .lineToX(47.5)
//                    .waitSeconds(3)
//                    .build();
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
//            // @formatter:on
//
//            // @formatter:off
//            Actions.runBlocking(
//                    new SequentialAction(
//                            trajectoryAction1,
//                            trajectoryAction2,
////                        lift.liftUp(),
////                        claw.openClaw(),
////                        lift.liftDown()
//                            trajectoryAction3
//                    )
//            );
            // @formatter:on

//        }
    }
}
