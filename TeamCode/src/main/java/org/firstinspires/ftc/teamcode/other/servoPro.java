package org.firstinspires.ftc.teamcode.other;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name =  "Servo Programmer", group = "e")
public class servoPro extends OpMode {
    Servo claw;
    @Override
    public void init() {
        claw = hardwareMap.get(Servo.class, "Swivel");
    }

    @Override
    public void loop() {
        telemetry.addData("Pos", claw.getPosition());
        if(gamepad2.left_bumper) {
        if (gamepad2.right_stick_x > 0) {
            claw.setPosition(0.2 * gamepad2.right_stick_x + 0.6);
        }
} else {
        if(gamepad2.a) {
            claw.setPosition(0.6);
        } else {
            claw.setPosition(0.72);
        }
}
    }
}
