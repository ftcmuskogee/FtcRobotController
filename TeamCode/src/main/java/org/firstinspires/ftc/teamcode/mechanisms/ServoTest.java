package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoTest extends LinearOpMode{
    public boolean servo_ing = false;
    public double wait = 0.8;
    public double increment = 0.001;
    public double P = 1;

    @Override
    public void runOpMode() {

        // CRServo BallKicker = hardwareMap.get(CRServo.class, "BKr"); // wait(0.601)!!!!!!!
        CRServo hood = hardwareMap.get(CRServo.class, "Hood"); // wait(0.800)!!!!!!!

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            telemetry.update();

            telemetry.addData("Servo - ", "No Data :(");

            if (gamepad1.dpad_left && increment - 0.001 >= 0.01) increment -= 0.001; //              Press (Dpad LEFT) to decrease increment
            else if (gamepad1.dpad_right && increment + 0.001 < 0.1) increment += 0.001; //          Press (Dpad RIGHT) to increase increment

            telemetry.addData("Inc", increment);

            if (gamepad1.y && gamepad1.x) wait = 0.8;

            if (gamepad1.yWasPressed() && wait + increment <= 1) wait += increment; //             Press (Y) to increase wait
            if (gamepad1.bWasPressed() && wait - increment > -1) wait -= increment; //                Press (B) to decrease wait

            if (gamepad1.xWasPressed() && P == 1) P = -1; else if (gamepad1.xWasPressed()) P = 1; // Press (X) to swap servo direction

            // Ball Kicker
            if (gamepad1.a && !servo_ing) { //                                                       Press (A) to move the servo for (wait) seconds
                resetRuntime();
                servo_ing = true;
            } else if (servo_ing && getRuntime() <= wait) {
                hood.setPower(P);
                telemetry.addData("Servo - ", "Spinning...");
            } else if (servo_ing) {
                hood.setPower(0);
                telemetry.addData("Servo - ","Stopped");
                servo_ing = false;
            }

            telemetry.addData("Set WAIT for", wait);
        }
    }
}
