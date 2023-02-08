package org.firstinspires.ftc.teamcode.action;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class colorSensor {
    /** Range of the sensor */
    public final double rangeInDegrees = 60.00;
    ColorSensor colorSensor;

    public void init(@NonNull OpMode opMode) {
        HardwareMap hardwareMap = opMode.hardwareMap;
        colorSensor = hardwareMap.get(ColorSensor.class, "Color Sensor");
    }

    /** Finds the distance. Minimum range of 5cm and maximum of 25cm.
     * @return Distance in cm.
     */
    private double getDistance() {
        return ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
    }
}
