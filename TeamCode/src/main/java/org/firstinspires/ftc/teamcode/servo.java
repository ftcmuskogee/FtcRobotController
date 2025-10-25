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
    }

    @Override
    public void loop() {
        if(gamepad2.y){;
            servo.setPosition(.5);
//            servo.setPower(1);
            sleep(500);
            servo.setPosition(0);
//            servo.setPower(-1);
        }
//        servo.setPower(0);
    }
}
