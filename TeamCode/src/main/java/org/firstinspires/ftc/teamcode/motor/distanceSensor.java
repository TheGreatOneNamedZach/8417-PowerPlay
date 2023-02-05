package org.firstinspires.ftc.teamcode.motor;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.TypeConversion;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.vision.mb1043sensor;

import java.util.Objects;

public class distanceSensor {
    OpMode opMode;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    mb1043sensor mb1043sensor;
    Servo distanceServo;
    private static Boolean modeContinuous = false;
    private final double servoSpinAreaInDegrees = 180.00;
    private static String actionInProgress = "none";
    private static Boolean scanTowardsMaxPosition = true;
    private static int degreesToSkip = 0;

    /** Initializes the distance sensor */
    public void init(@NonNull OpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        mb1043sensor = hardwareMap.get(mb1043sensor.class, "Distance Sensor");
        distanceServo = hardwareMap.get(Servo.class, "Distance Servo");
    }

    /** Starts running the distance sensor until stop() is called. */
    public void startScanning(int degreesToSkip) {
        modeContinuous = true;
        actionInProgress = "contScan";
        distanceSensor.degreesToSkip = degreesToSkip;
    }

    /** Scans its area then scans back to its starting position. */
    public void sweepOnce(int degreesToSkip) {
        if(modeContinuous || (!Objects.equals(actionInProgress, "none"))) {return;} // Don't continue if it would be redundant or if an action is in progress
        actionInProgress = "sweepOnce"; // Since no other code is using the sensor, lets run this method
        distanceSensor.degreesToSkip = degreesToSkip;
    }

    /** Scans its area but does not go back to its starting position. */
    public void scanToMax(int degreesToSkip) {
        if(modeContinuous || (!Objects.equals(actionInProgress, "none"))) {return;} // Don't continue if it would be redundant or if an action is in progress
        actionInProgress = "scanOnce"; // Since no other code is using the sensor, lets run this method
        distanceSensor.degreesToSkip = degreesToSkip;
    }

    /** Scans its area and ends in the starting position. */
    public void scanToMin(int degreesToSkip) {
        degreesToSkip = degreesToSkip * -1;
        if(modeContinuous || (!Objects.equals(actionInProgress, "none"))) {return;} // Don't continue if it would be redundant or if an action is in progress
        actionInProgress = "scanOnce"; // Since no other code is using the sensor, lets run this method
        distanceSensor.degreesToSkip = degreesToSkip;
    }


    /** Returns the distance sensor to its starting position without skipping degrees. */
    public void returnToStart(Boolean scanWhileDoingSo) {
        if(distanceServo.getPosition() == 0) {actionInProgress = "none";return;}
        if(!scanWhileDoingSo) {actionInProgress = "returnToStart"; distanceServo.setPosition(0);return;} // If the user does not want to scan, just return to home ASAP
        actionInProgress = "scanOnce";
    }

    /** Returns the distance sensor to its starting position while skipping degrees. */
    public void returnToStart(Boolean scanWhileDoingSo, int degreesToSkip) {
        if(distanceServo.getPosition() == 0) {actionInProgress = "none";return;}
        if(!scanWhileDoingSo) {actionInProgress = "none"; distanceServo.setPosition(0);return;} // If the user does not want to scan, just return to home ASAP
        distanceSensor.degreesToSkip = degreesToSkip;
    }

    private void rotate(int degreesToSkip) {
        double currentPosition = distanceServo.getPosition();
        double nextPosition = currentPosition + degreesToServoPosition(degreesToSkip);
        distanceServo.setPosition(nextPosition);
    }

    private Boolean delayNotFinished() {
        // If the time since the last ping is greater than the minimum delay then return true else return false
        return !((System.currentTimeMillis() - mb1043sensor.getLastPingTime()) > mb1043sensor.minDelayInMilliseconds);
    }

    private double degreesToServoPosition(int degrees) {
        // This assumes 0 degrees is the position 0.00
        try {
            // There is actually a lot of things that could go wrong
            // with this so there is a try catch just in case. (e.g. divide by zero)
            return (double)degrees / servoSpinAreaInDegrees;
        } catch (Exception e) {
            telemetry.addData("ERROR", e);
            return 0;
        }
    }
    private double degreesToServoPosition(double degrees) {
        // This assumes 0 degrees is the position 0.00
        try {
            // There is actually a lot of things that could go wrong
            // with this so there is a try catch just in case. (e.g. divide by zero)
            return degrees / servoSpinAreaInDegrees;
        } catch (Exception e) {
            telemetry.addData("ERROR", e);
            return 0;
        }
    }

    private double servoPositionToDegrees(double servoPosition) {
        return servoPosition * servoSpinAreaInDegrees;
    }

    /** Scans the area following the directions previously specified.
     * If no directions have previously been specified it will scan every degree continuously.
     * @return Returns as a double[]. Returns the distance and current rotation.
     */
    public double[] scan() {
        double distance = mb1043sensor.getDistance();
        double currentRotation = servoPositionToDegrees(distanceServo.getPosition());
        double[] disAndRot = {distance, currentRotation};
        if(delayNotFinished()) { // Waits to rotate until the minimum delay has passed.
            actionInProgress = "none";
            return disAndRot;
        }
        // Stops rotating if it would leave the minimum or maximum boundaries
        // In addition, if it is currently executing "sweepOnce", it will finish the second half
        // In addition, if it is currently executing a "scanOnce", it will stop
        // Else, it will reverse direction
        if(((distanceServo.getPosition() + degreesToServoPosition(degreesToSkip)) < 0) || ((distanceServo.getPosition() + degreesToServoPosition(degreesToSkip)) > degreesToServoPosition(servoSpinAreaInDegrees))) {
            if(Objects.equals(actionInProgress, "sweepOnce") && distanceServo.getPosition() == degreesToServoPosition(servoSpinAreaInDegrees)) {
                actionInProgress = "none";
                scanToMin(degreesToSkip);
                return disAndRot;
            } else if (Objects.equals(actionInProgress, "sweepOnce") && distanceServo.getPosition() == 0) {
                actionInProgress = "none";
                scanToMax(degreesToSkip);
                return disAndRot;
            }
            if((Objects.equals(actionInProgress, "scanOnce") && distanceServo.getPosition() == degreesToServoPosition(servoSpinAreaInDegrees)) || (Objects.equals(actionInProgress, "scanOnce") && distanceServo.getPosition() == 0)) {
                actionInProgress = "none";
                return disAndRot;
            }
        }
        degreesToSkip = scanTowardsMaxPosition ? degreesToSkip : degreesToSkip * -1;
        rotate(degreesToSkip);
        distance = mb1043sensor.getDistance();
        currentRotation = servoPositionToDegrees(distanceServo.getPosition());
        return new double[]{distance, currentRotation};
    }
}
