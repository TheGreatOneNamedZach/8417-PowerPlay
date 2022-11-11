package org.firstinspires.ftc.teamcode.motor;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name =  "Linear Claw", group = "e")
public class linearClaw extends OpMode {

    DcMotor elevator = null;
    Servo claw = null;
    DigitalChannel digitalTouch;
    double motorPower; // Declares a double for telemetry and motor use
    int motor1Pos = 0; // This saves the motor position when it is first at rest
    boolean firstLoop = false;
    boolean isPressed = false;

    @Override
    public void init() {

        elevator = hardwareMap.get(DcMotor.class, "Elevator");
        claw = hardwareMap.get(Servo.class, "Claw");
        digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    } //


    @Override
    public void loop() {

        motorPower = (gamepad2.left_stick_y * -0.75);
        elevator.setPower(motorPower); // Gamepad 1 left joystick Y axis controls motor1
        telemetry.addData("Elevator", motorPower); // motor1 power
        telemetry.addData("Pos", elevator.getCurrentPosition());

        if(!isPressed && gamepad2.left_stick_y == 0) { // If the joystick is at rest AND the slide is not at minimum position...
            if(!firstLoop) { // ...and the motor was NOT at rest in the last loop...
                motor1Pos = elevator.getCurrentPosition(); // ...get the motor position.
                firstLoop = true;
                /*
                This says the motor has been at rest before.
                This is so the motor postion is not constantly reset every loop iteration
                while the motor is at rest.
                 */
            } else { // ...and the motor WAS at rest in the last loop...
                elevator.setTargetPosition(motor1Pos); // Sets the target position
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                // Makes the positive direction the motor runs
                // towards the same direction you want it to go
                elevator.setPower(0.75);
            }
        } else { // If the joystick is NOT at rest...
            elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            firstLoop = false;
            if(isPressed && gamepad2.left_stick_y < 0) { // If the slide is at the minimum position...
                elevator.setPower(motorPower);
            }
        }

        clawControl(gamepad2.right_trigger);

        if (digitalTouch.getState()) {
            telemetry.addData("Digital Touch", "Is Not Pressed");
            boolean isPressed = false;
        } else {
            telemetry.addData("Digital Touch", "Is Pressed");
            boolean isPressed = true;
        }
    }

    public void clawControl(double a) { //"a" is the variable being passed in for the claw to close/open

        if(a != 0){
            claw.setPosition(1);
        }else{
            claw.setPosition(-1);
        }
    }
}





