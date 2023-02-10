package org.firstinspires.ftc.teamcode.action;


import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/** This is an interface for this year's distance sensor. */
public class linearSlide {
    // CONSTRUCT
    // DECLARE NULL
    DcMotor linearSlide;
    DigitalChannel touchSensor; // True when pressed
    Telemetry telemetry;
    // DECLARE CUSTOM
    private static double totalSpeed = 0.75; // Speed multiplier for the slide
    private static final double maxAutoSpeed = 0.6; // Maximum speed the linear slide can operate at AUTOMATICALLY. "Auto" does not stand for "autonomous"
    private static final double hoverSpeed = 0.012; // Speed at which the slide can hover
    private static double slidePower = 0.00;

    // METHODS
    /** Initializes the linear slide.
     * @param opMode If you are constructing from an Auto or TeleOp, type in "this" without the quotation marks.
     */
    public void init(@NonNull OpMode opMode) {
        HardwareMap hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;
        linearSlide = hardwareMap.get(DcMotor.class, "Linear Slide");
        touchSensor = hardwareMap.get(DigitalChannel.class, "Touch Sensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /** Moves the slides. This is best used with a joystick.
     * @apiNote Remember if you are using the "y" of a joystick reverse it!
     * @param power power to move the slides at. This is multiplied by the max speed.
     */
    public void setPower(double power) {
        slidePower = power * totalSpeed;

        if (!(touchSensorPressed() && slidePower < 0)) { // Everything BUT moving the slides down when fully retracted.
            if (slidePower == 0 && !touchSensorPressed()) { // If the slides are not moving AND the limit switch is not pressed...
                // This means the slide needs to hover at its position
                linearSlide.setPower(hoverSpeed);
            } else if(slidePower == 0) { // If the slides are not moving AND the limit switch is pressed...
                // This means the slides are fully retracted. This is the perfect opportunity to recalibrate the encoders
                linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            } else { // The slides must be moving
                linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                linearSlide.setPower(slidePower);
            }
        }
    }

    /** Fully retracts the slides
     * @apiNote The slides will decrease in power as they approach the target position.
     */
    public void retract() {
        goToPosition(0);
    }

    /** Sets the multiplier for the maximum speed the slides can travel at.
     * @apiNote This only affects setPower()! This will not affect the automatic movement of the slides.
     * @param speed The new speed multiplier. By default this is 0.75
     */
    public void setMaxSpeed(double speed) {
        totalSpeed = speed;
    }

    /** Goes to the designated junction when that button is pressed.
     * @apiNote The slides will decrease in power as they approach the target position.
     * @param low Low junction button
     * @param middle Middle junction button
     * @param high High junction button
     */
    public void goToJunctionsOnPress(@NonNull Boolean low, @NonNull Boolean middle, @NonNull Boolean high) {
        if (low) {
            goToPosition(1400);
        } else if (middle) {
            goToPosition(2260);
        } else if (high) {
            goToPosition(3000);
        }
    }

    /** Goes to the position and hovers there.
     * @apiNote The slides will decrease in power as they approach the target position.
     * @param ticks Encoder ticks
     */
    public void goToPosition(int ticks) {
        if(linearSlide.getCurrentPosition() >= ticks - 25 && ticks > 25 && linearSlide.getCurrentPosition() <= ticks + 25) {
            /*

                -------------- ticks + 25

                LINEAR SLIDE ~= ticks (At target position)

                -------------- ticks - 25



                Ticks is greater than 25 */
            linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            linearSlide.setPower(hoverSpeed);
        } else if (ticks > 25){
            /*  LINEAR SLIDE (Above target position)

                --------------
                ticks
                --------------

                LINEAR SLIDE (Below target position)

                Ticks is greater than 25 */
            linearSlide.setTargetPosition(ticks);
            linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linearSlide.setPower(Math.min(maxAutoSpeed, (3600 - linearSlide.getCurrentPosition()) * 0.01));
            /* Sets the power to be whichever is smaller:
            A.) A base power of the maximum allowed speed (maxAutoSpeed)
            B.) The deviation from its target position multiplied by 0.01
             */
        } else if (ticks == 0) {
            /*  LINEAR SLIDE (Above target position)

                -------------- ticks + 6
                ticks

                Ticks is 0 */
            linearSlide.setTargetPosition(ticks);
            linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linearSlide.setPower(Math.min(maxAutoSpeed, (linearSlide.getCurrentPosition()) * 0.005));
            /* Sets the power to be whichever is smaller:
            A.) A base power of the maximum allowed speed (maxAutoSpeed)
            B.) The deviation from its target position multiplied by 0.005
             */

            if(linearSlide.getCurrentPosition() <= 6 && touchSensorPressed()) {
                // This means the slide is fully retracted. This is the perfect opportunity to recalibrate the encoders
                linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                linearSlide.setPower(0);
            }
        }
    }

    /** Resets the encoder value for the slides. */
    public void resetEncoder() {
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /** Returns the current position of the slides
     * @return Returns as an int in ticks.
     */
    public int getCurrentPosition() {
        return linearSlide.getCurrentPosition();
    }

    /** Returns if the touch sensor is pressed or not.
     * @return Returns as a Boolean. True if pressed. False if not.
     */
    public Boolean touchSensorPressed() {
        return !touchSensor.getState();
    }

    public void telemetryOutput() {
        telemetry.addData("Power", slidePower);
        telemetry.addData("Touch Sensor", touchSensorPressed());
        telemetry.addData("Mode", linearSlide.getMode().toString());
        telemetry.addData("Ticks", linearSlide.getCurrentPosition());
    }
}
