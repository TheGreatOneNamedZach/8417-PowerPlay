package org.firstinspires.ftc.teamcode.action;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/** This is an interface for this year's claw. */
public class claw {
    // CONSTRUCT
    private static final ElapsedTime delay = new ElapsedTime();
    // DECLARE NULL
    private static Servo claw;
    // DECLARE CUSTOM
    private static double delayInSeconds = 0.75;
    private static final double closedPositionInTicks = 0.64;
    private static final double openPositionInTicks = 0.42;
    private static Boolean isClosed = false;

    // METHODS
    /** Initializes the claw.
     * @param opMode If you are constructing from an Auto or TeleOp, type in "this" without the quotation marks.
     */
    public void init(@NonNull OpMode opMode) {
        HardwareMap hardwareMap = opMode.hardwareMap;
        claw = hardwareMap.get(Servo.class, "Claw");
        isClosed = false;
    }

    /** Moves the claw to the opposite position of where it currently is.
     * This is useful during testing for visually indicating a change.
     * (e.g. If a color sensor sees red you can call toggleClaw() to visually indicate that the color sensor sees red.)
     * @apiNote By default, the claw as a delay of 0.75 seconds between movements. This can be changed.
     */
    public void toggleClaw() {
        if (delay.time() >= delayInSeconds) { // If the delay has finished...
            if (isClosed) { // Set the claw the opposite position
                open();
            } else {
                close();
            }
            delay.reset(); // Reset the 0.75 second delay
        }
    }

    /** Moves the claw to the opposite position of where it currently is when a button is pressed.
     * @param isPressed The button being pressed.
     * @apiNote By default, the claw as a delay of 0.75 seconds between movements. This can be changed.
     */
    public void toggleClawWithPress(@NonNull Boolean isPressed) {
        if (isPressed) { // If the button has been pressed...
            toggleClaw();
        }
    }

    /** Puts the claw in the open position.
     * @apiNote This will bypass the delay feature.
     */
    public void open() {
        claw.setPosition(openPositionInTicks);
        isClosed = false;
    }

    /** Puts the claw in the closed position.
     * @apiNote This will bypass the delay feature.
     */
    public void close() {
        claw.setPosition(closedPositionInTicks);
        isClosed = true;
    }

    /** Changes the minimum time the claw can move to a new position.
     * @param newDelay The delay in seconds.
     */
    public void setDelay(double newDelay) {
        delayInSeconds = newDelay;
    }
}
