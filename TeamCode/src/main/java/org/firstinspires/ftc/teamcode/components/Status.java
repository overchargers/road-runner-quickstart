package org.firstinspires.ftc.teamcode.components;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public class Status {

    Telemetry telemetry;
    Drivetrain drivetrain;
    MecanumDrive drive;
    PickArm pickArm;
    LiftArm liftArm;

    public Status(Telemetry telemetry, Drivetrain drivetrain, MecanumDrive drive, PickArm pickArm, LiftArm liftArm) {
      this.telemetry = telemetry;
      this.drivetrain = drivetrain;
      this.drive = drive;
      this.pickArm = pickArm;
      this.liftArm = liftArm;
    }

    public void show_status() {
        telemetry.addData("FrontRight Power", drive.rightFront.getPower());
        telemetry.addData("BackRight Power", drive.rightBack.getPower());
        telemetry.addData("FrontLeft Power", drive.leftFront.getPower());
        telemetry.addData("BackLeft Power", drive.leftBack.getPower());
        telemetry.addData("PickMotor Power", pickArm.pickMotor.getPower());
        telemetry.addData("PickMotor Encoder", pickArm.pickMotor.getCurrentPosition());
        telemetry.addData("LiftMotor Power", liftArm.liftMotor.getPower());
        telemetry.addData("LiftMotor Encoder", liftArm.liftMotor.getCurrentPosition());
        telemetry.addData("PickSwing Position", Constants.pick_arm_swing_servo_backwards);
        telemetry.addData("LiftSwing Position", Constants.lift_arm_swing_servo_tuck);
        telemetry.addData("MotorDirection Reverse", drivetrain.direction_is_reverse);
        telemetry.update();
    }
}
