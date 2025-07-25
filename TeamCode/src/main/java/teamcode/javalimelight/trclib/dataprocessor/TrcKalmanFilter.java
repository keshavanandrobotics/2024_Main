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

package teamcode.javalimelight.trclib.dataprocessor;

/**
 * This class implements the Kalman filter. It is useful for filtering noise from the sensor data.
 */
public class TrcKalmanFilter extends TrcFilter
{
    private static final double DEF_KQ = 0.022;
    private static final double DEF_KR = 0.617;

    private final double kQ;
    private final double kR;
    private double prevP;
    private double prevXEst;
    private boolean initialized;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param kQ specifies the KQ constant.
     * @param kR specifies the KR constant.
     */
    public TrcKalmanFilter(String instanceName, double kQ, double kR)
    {
        super(instanceName);

        this.kQ = kQ;
        this.kR = kR;
        reset();
    }   //TrcKalmanFilter

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public TrcKalmanFilter(String instanceName)
    {
        this(instanceName, DEF_KQ, DEF_KR);
    }   //TrcKalmanFilter

    //
    // Implements TrcFilter abstract methods.
    //

    /**
     * This method resets the filter.
     */
    @Override
    public void reset()
    {
        prevP = 0.0;
        prevXEst = 0.0;
        initialized = false;
    }   //reset

    /**
     * This method returns the filtered data.
     *
     * @param data specifies the data value to be filtered.
     * @return filtered data.
     */
    @Override
    public double filterData(double data)
    {
        if (!initialized)
        {
            prevXEst = data;
            initialized = true;
        }

        double tempP = prevP + kQ;
        double k = tempP/(tempP + kR);
        double xEst = prevXEst + k*(data - prevXEst);

        prevP = (1 - k)*tempP;
        prevXEst = xEst;

        return prevXEst;
    }   //filterData

}   //class TrcKalmanFilter
