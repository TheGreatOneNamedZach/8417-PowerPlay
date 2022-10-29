package org.firstinspires.ftc.teamcode.motor;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name =  "Linear Claw", group = "e")
public class linearClaw extends OpMode {

    DcMotor elevator = null;
    double motorPower; // Declares a double for telemetry and motor use
    int motor1Pos = 0; // This saves the motor position when it is first at rest
    boolean firstLoop = false;

    @Override
    public void init() {

        elevator = hardwareMap.get(DcMotor.class, "Elevator");
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {

        motorPower = (gamepad2.left_stick_y * -0.75);
        elevator.setPower(motorPower); // Gamepad 1 left joystick Y axis controls motor1
        telemetry.addData("Elevator", motorPower); // motor1 power
        telemetry.addData("Pos", elevator.getCurrentPosition());

        if(gamepad2.left_stick_y == 0) { // If the joystick is at rest...
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
            elevator.setPower(motorPower);
        }
    }
}
