/*
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.MenuCode.TrcFtcLib.ftclib;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import org.firstinspires.ftc.teamcode.MenuCode.TrcCommonLib.trclib.TrcSensor;
import org.firstinspires.ftc.teamcode.MenuCode.TrcCommonLib.trclib.TrcTimer;
import org.firstinspires.ftc.teamcode.MenuCode.TrcCommonLib.trclib.TrcUtil;

/**
 * This class implements the ZX Distance sensor extending FtcI2cDevice.
 */
public class FtcZXDistanceSensor extends FtcI2cDevice implements TrcSensor.DataSource<FtcZXDistanceSensor.DataType>
{
    public enum DataType
    {
        GESTURE,
        GESTURE_SPEED,
        X,
        Z,
        LEFT_RANGING_DATA,
        RIGHT_RANGING_DATA
    }   //enum DataType

    public static final int DEF_I2CADDRESS          = 0x20;
    public static final int ALTERNATE_I2CADDRESS    = 0x22;

    //
    // GestureSense XZ01 Sensor I2C Register Map Version 1.
    //
    private static final int REG_STATUS             = 0x00;     //Sensor and Gesture Status
    private static final int REG_DRE                = 0x01;     //Data Ready Enable Bitmap
    private static final int REG_DRCFG              = 0x02;     //Data Ready Configuration
    private static final int REG_GESTURE            = 0x04;     //Last Detected Gesture
    private static final int REG_GSPEED             = 0x05;     //Last Detected Gesture Speed
    private static final int REG_DCM                = 0x06;     //Data Confidence Metric
    private static final int REG_XPOS               = 0x08;     //X Coordinate
    private static final int REG_ZPOS               = 0x0a;     //Z Coordinate
    private static final int REG_LRNG               = 0x0c;     //Left Emitter Ranging Data
    private static final int REG_RRNG               = 0x0e;     //Right Emitter Ranging Data
    private static final int REG_REGVER             = 0xfe;     //Register Map Version
    private static final int REG_MODEL              = 0xff;     //Sensor Model ID
    private static final int READ_LENGTH            = 0x10;     //number of registers in the read window

    //
    // Register 0x00 - STATUS:
    //  DAV - Position Data Available (RO).
    //      1 indicates that new position value is available in the coordinate registers.
    //      This bit automatically resets to zero after being read.
    //  OVF - Brightness value overflow (RO).
    //      Currently unused, reads 0.
    //  SWP - Swipe Gesture Available (RO).
    //      1 indicates that a swipe gesture has been detected.
    //      Gesture value is available in the gesture register.
    //      This bit automatically resets to zero after being read.
    //  HOVER - Hover Gesture Available (RO).
    //      1 indicates that a hover gesture has been detected.
    //      Gesture value is available in the gesture register.
    //      This bit automatically resets to zero after being readn
    //  HVG - Hover-Move Gesture Available (RO).
    //      1 indicates that a hover-and-move gesture has been detected.
    //      Gesture value is available in the gesture register.
    //      This bit automatically resets to zero after being read.
    //  EDGE - Edge Detection Event (RO).
    //      Currently unused, reads 0.
    //  HB - Heartbeat (RO).
    //      This bit will toggle every time the status register has been read.
    //
    public static final int STATUS_DAV              = (1 << 0);
    public static final int STATUS_OVF              = (1 << 1);
    public static final int STATUS_SWP              = (1 << 2);
    public static final int STATUS_HOVER            = (1 << 3);
    public static final int STATUS_HVG              = (1 << 4);
    public static final int STATUS_EDGE             = (1 << 5);
    public static final int STATUS_HB               = (1 << 7);
    public static final int STATUS_GESTURES         = (STATUS_SWP | STATUS_HOVER | STATUS_HVG);

    //
    // Register 0x01 - DRE (Data Ready Enable):
    //  A '1' in any of these bits will allow the DR pin to assert when the respective event or gesture occurs. The
    //  default value of this register is 0x00, meaning that nothing will cause the DR pin to assert. The value of
    //  this register does not prevent gestures or events from being detected. It only controls which gestures or
    //  events will cause the DR pin to assert.
    //  RNG - Ranging Data Available (RW).
    //      Enable 1 = assert DR when new ranging value is available.
    //  CRD - Coordinate Data Available (RW).
    //      Enable 1 = assert DR when new coordinate value is available.
    //  SWP - Swipe Gestures (RW).
    //      Enable 1 = assert DR when swipe gestures are detected.
    //  HOVER - Hover Gestures (RW).
    //      Enable 1 = assert DR when hover gestures are detected.
    //  HVG - Hover-Move Gestures (RW).
    //      Enable 1 = assert DR when "hover-move" gestures are detected.
    //  EDGE - Edge Detection Events (RW).
    //      Enable 1 = assert DR when edge detection occurs.
    //
    private static final int DRE_RNG                = (1 << 0);
    private static final int DRE_CRD                = (1 << 1);
    private static final int DRE_SWP                = (1 << 2);
    private static final int DRE_HOVER              = (1 << 3);
    private static final int DRE_HVG                = (1 << 4);
    private static final int DRE_EDGE               = (1 << 5);
    private static final int DRE_ALL                = (DRE_RNG | DRE_CRD | DRE_SWP | DRE_HOVER | DRE_HVG | DRE_EDGE);

    //
    // Register 0x02 - DRCFG (Data Ready Config):
    //  The default value of this register is 0x81.
    //  POLARITY - DR pin Polarity Select (RW).
    //      1 = DR pin is active-high.
    //      0 = DR pin is active-low.
    //  EDGE - DR pin Edge/Level Select (RW).
    //      1 = DR pin asserts for 1 pulse.
    //      0 = DR pin asserts until STATUS is read.
    //  FORCE - Force DR pin to assert, this bit auto-clears (RW).
    //      1 = Force DR pin to assert.
    //      0 = normal DR operation.
    //  EN - Enable DR (RW).
    //      1 = DR enabled.
    //      0 = DR always negated.
    //
    private static final int DRCFG_POLARITY         = (1 << 0);
    private static final int DRCFG_EDGE             = (1 << 1);
    private static final int DRCFG_FORCE            = (1 << 6);
    private static final int DRCFG_EN               = (1 << 7);

    //
    // Register 0x04 - Last Detected Gesture (RO).
    //  The most recent gesture appears in this register. The gesture value remains until a new gesture is detected.
    //  The gesture bits in the status register can be used to determine when to read a new value from this register.
    //  0x01 - Right Swipe.
    //  0x02 - Left Swipe
    //  0x03 - Up Swipe
    //  0x05 - Hover
    //  0x06 - Hover-Left
    //  0x07 - Hover-Right
    //  0x08 - Hover-Up
    //
    private static final int GESTURE_NONE           = 0x00;
    private static final int GESTURE_RIGHT_SWIPE    = 0x01;
    private static final int GESTURE_LEFT_SWIPE     = 0x02;
    private static final int GESTURE_UP_SWIPE       = 0x03;
    private static final int GESTURE_HOVER          = 0x05;
    private static final int GESTURE_HOVER_LEFT     = 0x06;
    private static final int GESTURE_HOVER_RIGHT    = 0x07;
    private static final int GESTURE_HOVER_UP       = 0x08;

    /**
     * Specifies the various detected gestures.
     */
    public enum Gesture
    {
        NONE(GESTURE_NONE),
        RIGHT_SWIPE(GESTURE_RIGHT_SWIPE),
        LEFT_SWIPE(GESTURE_LEFT_SWIPE),
        UP_SWIPE(GESTURE_UP_SWIPE),
        HOVER(GESTURE_HOVER),
        HOVER_LEFT(GESTURE_HOVER_LEFT),
        HOVER_RIGHT(GESTURE_HOVER_RIGHT),
        HOVER_UP(GESTURE_HOVER_UP);

        public final int value;

        /**
         * Constructor: Create an instance of the enum type.
         *
         * @param value specifies the enum ordinal value.
         */
        Gesture(int value)
        {
            this.value = value;
        }   //Gesture

        /**
         * This method returns the Gesture enum object matching the specified ordinal value.
         *
         * @param value specifies the ordinal value to match for.
         * @return Gesture enum object matching the ordinal value.
         */
        public static Gesture getGesture(int value)
        {
            for (Gesture g: Gesture.values())
            {
                if (value == g.value)
                {
                    return g;
                }
            }
            return NONE;
        }   //getGesture

    }   //enum Gesture

    //
    // Register 0x05 - Last Detected Gesture Speed (RO).
    //  The speed of the most recently detected gesture is stored here. The value remains until a new gesture is
    //  detected.
    //

    //
    // Register 0x06 - Data Confidence Metric (RO).
    //  Currently unused. Returns 0.
    //

    //
    // Register 0x08 - X Position (RO).
    //  The most recently calculated X position is stored in this register.
    //
    private static final int MAX_XPOSITION          = 240;

    //
    // Register 0x0a - Z Position (RO).
    //  The most recently calculated Z position is stored in this register.
    //
    private static final int MAX_ZPOSITION          = 240;

    //
    // Register 0x0c - Left Emitter Ranging Data (RO).
    //  The left emitter ranging value is stored in this register.
    //

    //
    // Register 0x0e - Right Emitter Ranging Data (RO).
    //  The right emitter ranging value is stored in this register.
    //

    //
    // Register 0xfe - Register Map Version (RO).
    //  This register is used to identify the register map version of attached sensor. All sensors share a register
    //  map. Sensors with the same register map have the same value arrangement.
    //  0x01 = Register Map v1.
    //
    public static final int REGISTERMAP_VERSION     = 0x01;

    //
    // Register 0xff - Sensor Model (RO).
    //  This register is used to identify the type of sensor attached.
    //  0x01 = XZ01.
    //
    public static final int MODEL_VERSION           = 0x01;

    private final int regMapVersion;
    private final int modelVersion;
    private final TrcSensor.SensorData<Gesture> gesture = new TrcSensor.SensorData<>(0.0, null);
    private final TrcSensor.SensorData<Double> gestureSpeed = new TrcSensor.SensorData<>(0.0, null);
    private final TrcSensor.SensorData<Double> xPos = new TrcSensor.SensorData<>(0.0, null);
    private final TrcSensor.SensorData<Double> zPos = new TrcSensor.SensorData<>(0.0, null);
    private final TrcSensor.SensorData<Double> leftRangingData = new TrcSensor.SensorData<>(0.0, null);
    private final TrcSensor.SensorData<Double> rightRangingData = new TrcSensor.SensorData<>(0.0, null);

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param hardwareMap specifies the global hardware map.
     * @param instanceName specifies the instance name.
     * @param i2cAddress specifies the I2C address of the device.
     * @param addressIs7Bit specifies true if the I2C address is a 7-bit address, false if it is 8-bit.
     */
    public FtcZXDistanceSensor(HardwareMap hardwareMap, String instanceName, int i2cAddress, boolean addressIs7Bit)
    {
        super(hardwareMap, instanceName, i2cAddress, addressIs7Bit);
        deviceSynch.setDeviceInfo(HardwareDevice.Manufacturer.Other, "ZX Distance Sensor");
        deviceSynch.setBufferedReadWindow(REG_STATUS, READ_LENGTH, I2cDeviceSynch.ReadMode.REPEAT, READ_LENGTH);

        byte[] data;

        data = syncRead(REG_REGVER, 1);
        regMapVersion = TrcUtil.bytesToInt(data[0]);

        data = syncRead(REG_MODEL, 1);
        modelVersion = TrcUtil.bytesToInt(data[0]);
    }   //FtcZXDistanceSensor

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param i2cAddress specifies the I2C address of the device.
     * @param addressIs7Bit specifies true if the I2C address is a 7-bit address, false if it is 8-bit.
     */
    public FtcZXDistanceSensor(String instanceName, int i2cAddress, boolean addressIs7Bit)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName, i2cAddress, addressIs7Bit);
    }   //FtcZXDistanceSensor

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public FtcZXDistanceSensor(String instanceName)
    {
        this(instanceName, DEF_I2CADDRESS, false);
    }   //FtcZXDistanceSensor

    /**
     * This method returns the data from the Status register.
     *
     * @return status register data.
     */
    public int getStatus()
    {
        byte[] data = readData(REG_STATUS, 1);
        int deviceStatus = TrcUtil.bytesToInt(data[0]);
        double timestamp = TrcTimer.getCurrentTime();

        if ((deviceStatus & STATUS_GESTURES) != 0)
        {
            data = readData(REG_GESTURE, 1);
            gesture.timestamp = timestamp;
            gesture.value = Gesture.getGesture(TrcUtil.bytesToInt(data[0]));

            data = readData(REG_GSPEED, 1);
            gestureSpeed.timestamp = timestamp;
            gestureSpeed.value = (double)TrcUtil.bytesToInt(data[0]);
        }

        if ((deviceStatus & STATUS_DAV) != 0)
        {
            data = readData(REG_XPOS, 1);
            xPos.timestamp = timestamp;
            xPos.value = (double)TrcUtil.bytesToInt(data[0]);

            data = readData(REG_ZPOS, 1);
            zPos.timestamp = timestamp;
            zPos.value = (double)TrcUtil.bytesToInt(data[0]);

            data = readData(REG_LRNG, 1);
            leftRangingData.timestamp = timestamp;
            leftRangingData.value = (double)TrcUtil.bytesToInt(data[0]);

            data = readData(REG_RRNG, 1);
            rightRangingData.timestamp = timestamp;
            rightRangingData.value = (double)TrcUtil.bytesToInt(data[0]);
        }

        return deviceStatus;
    }   //getStatus

    /**
     * This method returns the detected gesture type.
     *
     * @return detected gesture type.
     */
    public TrcSensor.SensorData<Gesture> getGesture()
    {
        getStatus();
        return new TrcSensor.SensorData<>(gesture.timestamp, gesture.value);
    }   //getGesture

    /**
     * This method returns the data from the Gesture Speed register.
     *
     * @return gesture speed.
     */
    public TrcSensor.SensorData<Double> getGestureSpeed()
    {
        getStatus();
        return new TrcSensor.SensorData<>(gestureSpeed.timestamp, gestureSpeed.value);
    }   //getGestureSpeed

    /**
     * This method returns the data from teh X Position register.
     *
     * @return X position.
     */
    public TrcSensor.SensorData<Double> getX()
    {
        getStatus();
        return new TrcSensor.SensorData<>(xPos.timestamp, xPos.value);
    }   //getX

    /**
     * This method returns the data from teh Z Position register.
     *
     * @return Z position.
     */
    public TrcSensor.SensorData<Double> getZ()
    {
        getStatus();
        return new TrcSensor.SensorData<>(zPos.timestamp, zPos.value);
    }   //getZ

    /**
     * This method returns the data from the Left Ranging Data register.
     *
     * @return left ranging data.
     */
    public TrcSensor.SensorData<Double> getLeftRangingData()
    {
        getStatus();
        return new TrcSensor.SensorData<>(leftRangingData.timestamp, leftRangingData.value);
    }   //getLeftRangingData

    /**
     * This method returns the data from the Right Ranging Data register.
     *
     * @return right ranging data.
     */
    public TrcSensor.SensorData<Double> getRightRangingData()
    {
        getStatus();
        return new TrcSensor.SensorData<>(rightRangingData.timestamp, rightRangingData.value);
    }   //getRightRangingData

    /**
     * This method returns the data from the Register Map Version register.
     *
     * @return register map version.
     */
    public int getRegMapVersion()
    {
        return regMapVersion;
    }   //getRegMapVersion

    /**
     * This method returns the data from the Model Version register.
     *
     * @return model version.
     */
    public int getModelVersion()
    {
        return modelVersion;
    }   //getModelVersion

    //
    // Implements TrcSensor.DataSource interface.
    //

    /**
     * This method returns the sensor data of the specified index.
     *
     * @param index specifies the data index.
     * @param dataType specifies the data type.
     * @return sensor data of the specified index and type.
     */
    @Override
    public TrcSensor.SensorData getRawData(int index, DataType dataType)
    {
        TrcSensor.SensorData data = null;

        switch (dataType)
        {
            case GESTURE:
                data = getGesture();
                break;

            case GESTURE_SPEED:
                data = getGestureSpeed();
                break;

            case X:
                data = getX();
                break;

            case Z:
                data = getZ();
                break;

            case LEFT_RANGING_DATA:
                data = getLeftRangingData();
                break;

            case RIGHT_RANGING_DATA:
                data = getRightRangingData();
                break;
        }

        return data;
    }   //getRawData

}   //class FtcZXDistanceSensor
