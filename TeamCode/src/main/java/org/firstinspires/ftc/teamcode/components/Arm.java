package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public abstract class Arm {

    protected DcMotor armMotor;
    protected Servo swingServo;
    protected CRServo spinServo;

    public Arm(HardwareMap hardwareMap, String motorName, String swingServoName, String spinServoName)
    {
        armMotor = hardwareMap.get(DcMotor.class, motorName);
        swingServo = hardwareMap.get(Servo.class, swingServoName);
        spinServo = hardwareMap.get(CRServo.class, spinServoName);
    }

    protected void moveArmMotor(int target, double power)
    {
        armMotor.setTargetPosition(target);
        armMotor.setPower(power);
    }

    public void setSwingServoPosition(double position)
    {
        swingServo.setPosition(position);
    }

    public int getArmMotorPosition()
    {
        return armMotor.getCurrentPosition();
    }

    abstract void addTelemetryData();

}
