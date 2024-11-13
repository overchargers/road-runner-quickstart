package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.support.InfluxDbLogger;

@TeleOp(name = "TestLogging (Blocks to Java)")
public class TestLogging extends LinearOpMode {
    /**
     * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
     * Comment Blocks show where to place Initialization code (runs once, after touching the
     * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
     * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
     * Stopped).
     */
    @Override
    public void runOpMode() {
        int count;
        double sine;
        InfluxDbLogger influxDbLogger = new InfluxDbLogger(hardwareMap);

        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            count = 0;
            while (opModeIsActive()) {
                // Put loop blocks here.

                sine = Math.sin((double)count / 180 * Math.PI);

                influxDbLogger.add("sine", sine);
                influxDbLogger.add("log", "count is " + count);
                influxDbLogger.write();

                if (gamepad1.a) {
                    telemetry.addData("Sine result", sine);
                }
                if (gamepad1.b) {
                    telemetry.addData("Sine result", "String value " + sine);
                }
                if (gamepad1.x) {
                    RobotLog.ii("DbgLog", "String value " + sine);
                    RobotLog.ee("DbgLog", "String value " + sine);
                }
                count = count + 1;
                telemetry.addData("InfluxDb connected", influxDbLogger.isConnected());
                telemetry.update();
            }
        }
    }
}

