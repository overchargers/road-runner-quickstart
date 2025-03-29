package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public class Status
{
  Telemetry telemetry;
  Drivetrain drivetrain;
  MecanumDrive drive;
  PickArm pickArm;
  LiftArm liftArm;

  public Status(Telemetry telemetry, Drivetrain drivetrain, MecanumDrive drive, PickArm pickArm, LiftArm liftArm)
  {
    this.telemetry = telemetry;
    this.drivetrain = drivetrain;
    this.drive = drive;
    this.pickArm = pickArm;
    this.liftArm = liftArm;
  }

  public void show_status()
  {
    telemetry.update();
  }
}
