package org.firstinspires.ftc.teamcode.other.standalone;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.action.claw;

/** This TeleOp allows you to find the position of a servo using a gamepad controller. */
@TeleOp(name =  "Servo Programmer", group = "e")
public class servoPro extends OpMode {
    Servo servo;
    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "Swivel");
    }

    @Override
    public void loop() {
        telemetry.addData("Pos", servo.getPosition());
        if (gamepad2.right_stick_x > 0) {
            servo.setPosition(0.2 * gamepad2.right_stick_x + 0.5);
        } else {
            if(gamepad2.a) {
                servo.setPosition(0.6);
            } else if (gamepad1.dpad_up) {
                servo.setPosition(servo.getPosition() + 0.01);
            } else if (gamepad1.dpad_down) {
                servo.setPosition(servo.getPosition() - 0.01);
            }
        }
    }
}
