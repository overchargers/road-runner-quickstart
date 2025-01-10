package org.firstinspires.ftc.teamcode.components;

import static org.firstinspires.ftc.teamcode.Constants.clip_close;
import static org.firstinspires.ftc.teamcode.Constants.clip_open;
import static java.lang.Thread.sleep;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class LiftArm {
    public boolean lift_arm_was_intaking = false;
    public DcMotor liftMotor;
    public Servo liftswingservo;
    private CRServo liftspinservo;
    private Gamepad gamepad1;
    private Telemetry telemetry;
    public Servo clip;
    private Drivetrain drivetrain;

    public LiftArm(HardwareMap hardwareMap, Gamepad gp, Telemetry telem, Drivetrain drivetrain) {
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        liftswingservo = hardwareMap.get(Servo.class, "liftswingservo");
        liftspinservo = hardwareMap.get(CRServo.class, "liftspinservo");
        clip = hardwareMap.get(Servo.class, "clip");
        gamepad1 = gp;
        telemetry = telem;
        this.drivetrain = drivetrain;
    }

    public void prevent_lift_motor_overheating() throws InterruptedException {
        if (((DcMotorEx) liftMotor).getVelocity() == 0 && liftMotor.getTargetPosition() > 100) {
            liftMotor.setPower(0.1);
        }
        if (((DcMotorEx) liftMotor).getVelocity() == 0 && liftMotor.getTargetPosition() < 100) {
            liftMotor.setPower(0);
        }
    }

    public void close_clip() {
        clip.setPosition(clip_close);
    }

    public class CloseClip implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
                close_clip();
            return false;
        }
    }

    public Action close_clip_action() {
        return new CloseClip();
    }

    public void open_clip() {
        clip.setPosition(clip_open);
    }
    public class OpenClip implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
                open_clip();

            return false;
        }
    }
    public Action open_clip_action() {
        return new OpenClip();
    }
    private void lift_arm_move(int target, double power) {
        liftMotor.setTargetPosition((int) target);
        liftMotor.setPower(power);
    }

    public void lift_arm_to_high_basket() throws InterruptedException {
        int target = 3150;
        double power = 1;
        lift_arm_move((int) target, power);
        sleep(100);
    }

    public void init_lift_arm() {
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        liftswingservo.setPosition(Constants.lift_arm_swing_servo_tuck);
    }

    public void lift_arm_to_low_basket() throws InterruptedException {
        double power = 1;
        int target = 2300;
        lift_arm_move((int) target, power);
        sleep(100);
    }

    public void lift_arm_to_zero() throws InterruptedException {
        double power = 1;
        int target = 0;
        lift_arm_move((int) target, power);
        sleep(100);
    }
    public void lift_arm_to_zero_specimen() throws InterruptedException {
        double power = 1;
        int target = 0;
        lift_arm_move((int) target, power);
        sleep(100);
    }

    public void lift_arm_outtake() {
        int count = 0;
        while (gamepad1.left_bumper) {
            liftspinservo.setPower(-0.5);
            count = 1 + count;
            telemetry.addData("count", count);
            telemetry.update();
            drivetrain.drive_motor();
        }
        liftspinservo.setPower(0);
        lift_arm_was_intaking = false;
    }

    public void lift_arm_intake() {
        int count = 0;
        liftspinservo.setDirection(CRServo.Direction.FORWARD);
        while (gamepad1.left_bumper) {
            liftspinservo.setPower(0.4);
            count = 1 + count;
            telemetry.addData("count", count);
            telemetry.update();
            drivetrain.drive_motor();
        }
        liftspinservo.setPower(0.1);
        lift_arm_was_intaking = true;
    }

    public void lift_arm_to_high_rung() throws InterruptedException {
        double power = 1;
        int target = 1500;
        lift_arm_move((int) target, power);
        sleep(100);
    }

    public void lift_arm_to_below_high_rung() throws InterruptedException {
        double power = 1;
        int target = 875;
        lift_arm_move((int) target, power);
        sleep(100);
    }

    public void lift_arm_swing_to_low_basket() {
        liftswingservo.setPosition(Constants.low_basket);
    }

    public void lift_arm_swing_to_high_basket() {
        liftswingservo.setPosition(Constants.high_basket);
    }

    // lift arm to high basket
    public class LiftArmToHighBasket implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            try {
                lift_arm_to_high_basket();
                lift_arm_swing_to_high_basket();
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            return false;
        }
    }

    public Action lift_arm_to_high_basket_action() {
        return new LiftArmToHighBasket();
    }

    // lift arm to zero
    public class LiftArmToZero implements Action {
        public LiftArmToZero()
        {
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            try {
                lift_arm_to_zero();
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            return false;
        }
    }
    public Action lift_arm_to_zero_action() {
        return new LiftArmToZero();
    }

    public class LiftArmToZeroAndSwingToLowBasket implements Action {
        public LiftArmToZeroAndSwingToLowBasket()
        {
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            try {
                lift_arm_to_zero();
                lift_arm_swing_to_low_basket();
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            return false;
        }
    }
    public Action lift_arm_to_zero_and_swing_to_low_basket_action() {
        return new LiftArmToZeroAndSwingToLowBasket();
    }

    // intake
    public class LiftArmIntake implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            liftspinservo.setPower(0.4);
            try {
                sleep(1000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            liftspinservo.setPower(0.1);
            return false;
        }
    }

    public Action lify_arm_intake_action() {
        return new LiftArmIntake();
    }

    // outtake
    public class LiftArmOuttake implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            liftspinservo.setPower(-0.5);
            try {
                sleep(750);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            liftspinservo.setPower(0);
            return false;
        }
    }
    public Action lift_arm_outtake_action() {
        return new LiftArmOuttake();
    }

    // high rung
    public class HighRung implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            try {
                lift_arm_to_high_rung();
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            return false;
        }
    }

    public Action high_rung_action() {
        return new HighRung();
    }

    // below high rung
    public class LowRung implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            try {
                lift_arm_to_below_high_rung();
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            return false;
        }
    }

    public Action low_rung_action() {
        return new LowRung();
    }

}
