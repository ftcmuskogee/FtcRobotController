package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


@Autonomous
public class StateintegerPractice extends OpMode {

    int state;


    @Override
    public void init() {
        state = 0;
    }

    @Override
    public void loop() {
        telemetry.addData("Cur State", state);
        switch (state) {
            case 0:
                telemetry.addLine("To exit state, press A / x");
                if (gamepad1.a) {
                    state = 1;
                }
                break;
            case 1:
                telemetry.addLine("To exit state, press B / o");
                if (gamepad1.b) {
                    state = 2;
                }
                break;
            case 2:
                telemetry.addLine("To exit state, press X / [=]");
                if (gamepad1.x) {
                    state = 3;

                }
                break;
            default:
                telemetry.addLine("Auto Sate machine finished");
        }

    }
}


