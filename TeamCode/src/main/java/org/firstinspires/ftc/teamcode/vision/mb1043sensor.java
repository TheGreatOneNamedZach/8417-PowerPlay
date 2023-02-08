package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

// https://github.com/EricTownselAdams/ftc_app/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/I2CXLv2.java
@I2cDeviceType
@DeviceProperties(name = "MaxSonar I2CXL v2", description = "MaxSonar I2CXL Sensor from MaxBotix", xmlTag = "MaxSonarI2CXLv2")
public class mb1043sensor extends I2cDeviceSynchDevice<I2cDeviceSynch>
{
    private int lastDistance = -1;
    private long lastPingTime;
    private boolean waitingForNextPing = true;
    /** Range of the sensor */
    public final double rangeInDegrees = 90.00;
    /** Minimum update delay of sensor (how quickly it spit out new data) */
    public final double minDelayInMilliseconds = 100;

    @Override
    public Manufacturer getManufacturer(){
        return Manufacturer.Other;
    }

    @Override
    protected synchronized boolean doInitialize(){
        return true;
    }

    @Override
    public String getDeviceName(){
        return "MaxSonarI2CXLv2";
    }

    /** Constructs the MB1043 sensor as an I2C device. */
    public mb1043sensor(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned){
        super(deviceClient, deviceClientIsOwned);
        this.deviceClient.setI2cAddress(I2cAddr.create8bit(0xE0));
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    private void ping(){
        deviceClient.write8(0, 0x51, I2cWaitControl.WRITTEN);
    }

    /** Returns the distance to the closest object.
     * There is a 0.1 second delay between updates.
     * Calling this method quicker than the delay will return the last distance found.
     * @return Centimeters as int
     */
    public int getDistance(){
        long curTime = System.currentTimeMillis();
        if(((curTime - lastPingTime) > 99) && !waitingForNextPing){
            lastDistance = TypeConversion.byteArrayToShort(deviceClient.read(0x01, 2));
            waitingForNextPing = true;
        } if((System.currentTimeMillis() - lastPingTime) > 99){
            ping();
            lastPingTime = System.currentTimeMillis();
            waitingForNextPing = false;
        }
        return lastDistance;
    }
}