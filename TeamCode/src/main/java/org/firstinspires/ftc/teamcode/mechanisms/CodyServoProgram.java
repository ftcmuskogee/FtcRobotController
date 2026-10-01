package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.CodyProgrammer;

@TeleOp
public class CodyServoProgram extends OpMode {
    CodyProgrammer bench = new CodyProgrammer();


    @Override
    public void init() {
        bench.init(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.cross) {
            bench.setServoPos(1.0);}
        else if (gamepad1.triangle){
            bench.setServoPos(-1.0);}

        if (gamepad1.square) {
            bench.setServoRot(0.0);
        } else {
            bench.setServoRot(0.55);
        }


    }
}