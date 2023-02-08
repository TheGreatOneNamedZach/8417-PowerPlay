package org.firstinspires.ftc.teamcode.initialize;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Mechanum Drive", group = "e")
public class teleOp extends OpMode {

    public ElapsedTime timer = new ElapsedTime();
    public ElapsedTime clawTimer = new ElapsedTime();
    public ElapsedTime swivelTimer = new ElapsedTime();

    DcMotor elevator = null;
    Servo claw = null;
    Servo swivel = null;
    DigitalChannel digitalTouch;
    double motorPower; // Declares a double for telemetry and motor use
    boolean firstLoop = false;
    boolean isPressed = false;

    org.firstinspires.ftc.teamcode.action.distanceSensor distanceSensor;
    org.firstinspires.ftc.teamcode.action.mecanumDrive mecanumDrive;

    @Override
    public void init() {
        elevator = hardwareMap.get(DcMotor.class, "Elevator");
        claw = hardwareMap.get(Servo.class, "Claw");
        swivel = hardwareMap.get(Servo.class, "Swivel");
        digitalTouch = hardwareMap.get(DigitalChannel.class, "Touch Sensor");
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        distanceSensor.init(this);
        mecanumDrive.init(this);
    }

    @Override
    public void loop() {
        mecanumDrive.setPower(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
        mecanumDrive.slowMode(gamepad1.right_bumper);

        telemetry.addData("GP2LS", gamepad2.left_stick_y);
        motorPower = (gamepad2.left_stick_y * -0.75);
        if (!(isPressed && gamepad2.left_stick_y > 0)) {
            elevator.setPower(motorPower);
            telemetry.addData("Elevator", motorPower); // motor1 power
            telemetry.addData("Pos", elevator.getCurrentPosition());

            if (gamepad2.left_stick_y == 0 && !isPressed) { // If the joystick is at rest AND the limit switch is not pressed...
                elevator.setPower(0.025);

                /*
                if (!firstLoop) { // ...and the motor was NOT at rest in the last loop...
                    motor1Pos = elevator.getCurrentPosition(); // ...get the motor position.
                    firstLoop = true;
                /*
                This says the motor has been at rest before.
                This is so the motor position is not constantly reset every loop iteration
                while the motor is at rest.
                 */ /*
                } else if (elevator.getCurrentPosition() < motor1Pos) { // ...and the motor WAS at rest in the last loop...
                    slideControl(motor1Pos);
                } */
            } else if(gamepad2.left_stick_y == 0 && isPressed) {
                elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            } else { // If the joystick is NOT at rest..
                elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                firstLoop = false;
                elevator.setPower(motorPower);
                telemetry.addData("Power", motorPower);
            }
        }
        if (gamepad2.dpad_down) {
            elevator.setPower(0.6);
            elevator.setTargetPosition(1400);
            elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (gamepad2.dpad_left) {
            elevator.setPower(0.6);
            elevator.setTargetPosition(2260);
            elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (gamepad2.dpad_up) {
            elevator.setPower(0.6);
            elevator.setTargetPosition(3600);
            elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }


        clawControl(gamepad2.right_bumper);
        swivelControl(gamepad1.left_bumper);

        if (digitalTouch.getState()) {
            telemetry.addData("Digital Touch", "Is Not Pressed");
            isPressed = false;
        } else {
            telemetry.addData("Digital Touch", "Is Pressed");
            isPressed = true;
        }
        telemetry.addData("Swivel", turned);
    }

    boolean closed = false;
    public void clawControl(boolean bumpPress) {
        if (bumpPress && clawTimer.time() >= 0.75) {
            // If the driver has pressed the button AND 0.75 seconds have passed since the last time this has run...
            claw.setPosition(closed ? .64 : .42); // Set the claw to .64 if true or .42 if false
            closed = !closed; // Swap Boolean "closed" to the opposite value
            clawTimer.reset(); // Reset the 0.75 delay
        }
    }

    boolean turned = false;
    public void swivelControl(boolean bumpPress) {
        if (bumpPress && swivelTimer.time() >= 1.00) {
            // If the driver has pressed the button AND 0.75 seconds have passed since the last time this has run...
            swivel.setPosition(turned ? .60 : .72); // Set the claw to .64 if true or .42 if false
            turned = !turned; // Swap Boolean "closed" to the opposite value
            swivelTimer.reset(); // Reset the 0.75 delay
        }
    }
}