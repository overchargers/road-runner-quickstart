package org.firstinspires.ftc.teamcode.components;

import static org.firstinspires.ftc.teamcode.Constants.pick_arm_swing_servo_backwards;
import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class PickArm {
    public boolean pick_arm_was_intaking = false;
    private CRServo pickspinservo;
    private Servo pickswingservo;
    public DcMotor pickMotor;
    private Gamepad gamepad1;
    private Telemetry telemetry;

    private Drivetrain drivetrain;


    public PickArm(HardwareMap hardwareMap, Gamepad gp, Telemetry telem, Drivetrain drivetrain)
    {
        pickswingservo = hardwareMap.get(Servo.class, "pickswingservo");
        pickspinservo = hardwareMap.get(CRServo.class, "pickspinservo");
        pickMotor = hardwareMap.get(DcMotor.class, "pickMotor");
        gamepad1=gp;
        telemetry=telem;
        this.drivetrain = drivetrain;
    }

    public void prevent_pick_motor_overheating() throws InterruptedException {
        if (((DcMotorEx) pickMotor).getVelocity() == 0) {
            pickMotor.setPower(0);
        }
    }

    public void init_pick_arm() {
        pickMotor.setTargetPosition(0);
        pickMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pickMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pickMotor.setDirection(DcMotor.Direction.REVERSE);
        pickswingservo.setPosition(Constants.pick_arm_swing_servo_tuck);
    }
    public void pick_arm_move(int target, double power) {
        pickMotor.setTargetPosition((int) target);
        pickMotor.setPower(power);
    }

    public void pick_arm_extend() throws InterruptedException {
        int target;
        double power;
        target = 1200;
        power = 1;
        pick_arm_move((int) target, power);
        // Extend Arm, Wait (Sleep), then Extend Swing Servo.
        sleep(100);
    }
    public void pick_arm_retract() throws InterruptedException {
        int target;
        double power;
        target = 0;
        power = 1;
        pick_arm_move((int) target, power);
        // Extend Arm, Wait (Sleep), then Extend Swing Servo.
        sleep(100);
    }
    public void pick_arm_outtake() {
        boolean pick_arm_was_intaking;
        while (gamepad1.right_bumper) {
            int count = 0;
            pickspinservo.setPower(-0.5);
            count = 1 + count;
            telemetry.addData("PickArm Outtake", count);
            telemetry.update();
            drivetrain.drive_motor();
        }
        pickspinservo.setPower(0);
        pick_arm_was_intaking = false;
    }
    public void pick_arm_intake() {
        int count = 0;
        boolean pick_arm_was_intaking;
        count = 1;
        pickspinservo.setDirection(CRServo.Direction.FORWARD);
        while (gamepad1.right_bumper) {
            pickspinservo.setPower(0.3);
            count = 1 + count;
            telemetry.addData("PickArm Intake", count);
            telemetry.update();
            drivetrain.drive_motor();
        }
        pickspinservo.setPower(0.1);
        pick_arm_was_intaking = true;
    }
    public void pick_arm_swing_backward() {
        pickswingservo.setPosition(pick_arm_swing_servo_backwards);
    }
    public void pick_arm_swing_forward() {
        pickswingservo.setPosition(0.83);
    }

}
