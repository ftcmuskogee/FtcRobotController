package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Set", group="Servo")
public class ServoPositions extends LinearOpMode {
    private Servo servo = null;

    @Override
    public void runOpMode() throws InterruptedException {

        Servo = hardwareMap.get(Servo.class, "servo");
        Servo.setDirection(Servo.Direction.FORWARD);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // show current position when CIRCLE is pressed
            if (gamepad1.aWasPressed()) {
                telemetry.addData("0 value is...", Servo.getPosition());
                telemetry.update();
            }

            // show current position when SQUARE is pressed
            if (gamepad1.bWasPressed()) {
                telemetry.addData("1 value is...", Servo.getPosition());
                telemetry.update();
            }
        }
    }

}
