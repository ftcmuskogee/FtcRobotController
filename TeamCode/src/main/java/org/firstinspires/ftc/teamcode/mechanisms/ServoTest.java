package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoTest extends LinearOpMode{
    public boolean Aservo_ing = false;
    public boolean Xservo_ing = false;
    public double wait = 0.8;
    public double increment = 0.001;
    public double P = 1;
    public boolean forward = true;

    @Override
    public void runOpMode() {

        CRServo hood = hardwareMap.get(CRServo.class, "Hood"); // wait(0.800)!!!!!!! (for one rotation)
        // Servo Gate = hardwareMap.get(Servo.class, "Gate");

        //Gate.setPosition(0);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            telemetry.update();

            telemetry.addData("Servo - ", "No Data :(");

            if (gamepad1.dpad_left && increment - 0.001 >= 0.01) increment -= 0.001; //              Press (Dpad LEFT) to decrease increment
            else if (gamepad1.dpad_right && increment + 0.001 < 0.1) increment += 0.001; //          Press (Dpad RIGHT) to increase increment

            telemetry.addData("Inc", increment);

            if (gamepad1.y && gamepad1.x) wait = 0.8; //                                             Hold (Y-TRIANGLE) and (X-SQUARE) to reset wait

            if (gamepad1.yWasPressed() && wait + increment <= 1) wait += increment; //               Press (Y-TRIANGLE) to increase wait
            if (gamepad1.bWasPressed() && wait - increment > 0) wait -= increment; //               Press (B-CIRCLE) to decrease wait

            // Ball Kicker REVERSE
            if (gamepad1.x && !Xservo_ing) { //                                                       Press (X-SQUARE) to move the servo for (wait) seconds
                resetRuntime();
                Xservo_ing = true;
            } else if (Xservo_ing && getRuntime() <= wait) {
                hood.setPower(-P);
                telemetry.addData("Servo - ", "Spinning...");
            } else if (Xservo_ing) {
                hood.setPower(0);
                telemetry.addData("Servo - ","Stopped");
                Xservo_ing = false;
            }

            // Ball Kicker FORWARD
            if (gamepad1.a && !Aservo_ing) { //                                                       Press (A-CROSS) to move the servo for (wait) seconds
                resetRuntime();
                Aservo_ing = true;
            } else if (Aservo_ing && getRuntime() <= wait) {
                hood.setPower(P);
                telemetry.addData("Servo - ", "Spinning...");
            } else if (Aservo_ing) {
                hood.setPower(0);
                telemetry.addData("Servo - ","Stopped");
                Aservo_ing = false;
            }

            telemetry.addData("Set WAIT for", wait);

            /*if (gamepad1.a) {
                Gate.setPosition(1);
            } else if (gamepad1.b) {
                Gate.setPosition(0);
            }*/
        }
    }
}
