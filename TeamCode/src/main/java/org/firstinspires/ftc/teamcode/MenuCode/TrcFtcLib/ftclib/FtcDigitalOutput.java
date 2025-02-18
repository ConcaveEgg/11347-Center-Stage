/*
 * Copyright (c) 2017 Titan Robotics Club (http://www.titanrobotics.com)
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

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MenuCode.TrcCommonLib.trclib.TrcDigitalOutput;

/**
 * This class implements a platform dependent digital output extending TrcDigitalOutput. It provides
 * implementation of the abstract methods in TrcDigitalOutput.
 */
public class FtcDigitalOutput extends TrcDigitalOutput
{
    private final DigitalChannel digitalOutput;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param hardwareMap specifies the global hardware map.
     * @param instanceName specifies the instance name.
     */
    public FtcDigitalOutput(HardwareMap hardwareMap, String instanceName)
    {
        super(instanceName);
        digitalOutput = hardwareMap.get(DigitalChannel.class, instanceName);
        digitalOutput.setMode(DigitalChannel.Mode.OUTPUT);
    }   //FtcDigitalOutput

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public FtcDigitalOutput(String instanceName)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName);
    }   //FtcDigitalOutput

    //
    // Implements TrcDigitalOutput abstract methods.
    //

    /**
     * This method sets the state of the digital output port.
     */
    @Override
    public void setState(boolean state)
    {
        if (setOutputElapsedTimer != null) setOutputElapsedTimer.recordStartTime();
        digitalOutput.setState(state);
        if (setOutputElapsedTimer != null) setOutputElapsedTimer.recordEndTime();
    }   //setState

}   //class FtcDigitalOutput
