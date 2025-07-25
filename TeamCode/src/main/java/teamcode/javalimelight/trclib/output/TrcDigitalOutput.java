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

package teamcode.javalimelight.trclib.output;

import teamcode.javalimelight.trclib.robotcore.TrcDbgTrace;
import teamcode.javalimelight.trclib.timer.TrcElapsedTimer;

/**
 * This class implements a platform independent Digital Output port device.
 */
public abstract class TrcDigitalOutput
{
    private static final String moduleName = TrcDigitalOutput.class.getSimpleName();
    protected static TrcElapsedTimer setOutputElapsedTimer = null;

    /**
     * This method is provided by the platform dependent digital output port device to set the state of the output
     * port.
     *
     * @param state specifies state of the output port.
     */
    public abstract void setState(boolean state);

    private final String instanceName;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public TrcDigitalOutput(String instanceName)
    {
        this.instanceName = instanceName;
    }   //TrcDigitalOutput

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    @Override
    public String toString()
    {
        return instanceName;
    }   //toString

    /**
     * This method enables/disables the elapsed timers for performance monitoring.
     *
     * @param enabled specifies true to enable elapsed timers, false to disable.
     */
    public static void setElapsedTimerEnabled(boolean enabled)
    {
        if (enabled)
        {
            if (setOutputElapsedTimer == null)
            {
                setOutputElapsedTimer = new TrcElapsedTimer(moduleName + ".setOutput", 2.0);
            }
        }
        else
        {
            setOutputElapsedTimer = null;
        }
    }   //setElapsedTimerEnabled

    /**
     * This method prints the elapsed time info using the given tracer.
     *
     * @param tracer specifies the tracer to be used to print the info.
     */
    public static void printElapsedTime(TrcDbgTrace tracer)
    {
        if (setOutputElapsedTimer != null)
        {
            setOutputElapsedTimer.printElapsedTime(tracer);
        }
    }   //printElapsedTime

}   //class TrcDigitalOutput
