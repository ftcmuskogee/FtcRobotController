package org.firstinspires.ftc.teamcode;


import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

    @TeleOp
    public class servo extends OpMode {
        //    public CRServo servo;
        public Servo servo;
        @Override
        public void init() {
            servo =hardwareMap.get(Servo.class,"servo");
//        servo = hardwareMap.get(CRServo.class, "servo");
            servo.setPosition(0.0);
        }

        @Override
        public void loop() {
            if(gamepad2.yWasPressed()){   // WasPressed means holding the button down won't repeat the loop every tick, just every press.
                servo.setPosition(-0.5);
//            servo.setPower(1);
                sleep(0.5);
                servo.setPosition(0.0);
//            servo.setPower(-1);
            }
//        servo.setPower(0);
        }
    }
