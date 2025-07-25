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

package teamcode.javalimelight.trclib.sensor;

import teamcode.javalimelight.trclib.dataprocessor.TrcDataIntegrator;
import teamcode.javalimelight.trclib.robotcore.TrcDbgTrace;
import teamcode.javalimelight.trclib.timer.TrcElapsedTimer;
import teamcode.javalimelight.trclib.dataprocessor.TrcFilter;

/**
 * This class implements a platform independent AnalogInput. Typically, this class is extended by a platform dependent
 * sensor class that produces value data. The sensor doesn't have to be connected to the AnalogInput port. It could be
 * connected to an I2C port as long as it produces a value data. The platform dependent sensor class must implement
 * the abstract methods required by this class. The abstract methods allow this class to get raw data from the sensor.
 * Depending on the options specified in the constructor, this class may create an integrator. If it needs data
 * integration, it can set the INTEGRATE or the DOUBLE_INTEGRATE options.
 */
public abstract class TrcAnalogInput extends TrcSensor<TrcAnalogInput.DataType>
{
    //
    // AnalogInput data type.
    //
    public enum DataType
    {
        RAW_DATA,
        INPUT_DATA,
        NORMALIZED_DATA,
        INTEGRATED_DATA,
        DOUBLE_INTEGRATED_DATA
    }   //enum DataType

    /**
     * This abstract method returns the raw data with the specified index and type.
     *
     * @param index specifies the data index.
     * @param dataType specifies the data type.
     * @return raw data with the specified type.
     */
    public abstract SensorData<Double> getRawData(int index, DataType dataType);
    //
    // AnalogInput options.
    //
    public static final int ANALOGINPUT_INTEGRATE       = (1);
    public static final int ANALOGINPUT_DOUBLE_INTEGRATE= (1 << 1);

    protected static TrcElapsedTimer getInputElapsedTimer = null;
    private final String instanceName;
    private TrcDataIntegrator<DataType> dataIntegrator = null;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param numAxes specifies the number of axes.
     * @param options specifies the AnalogInput options. Multiple options can be OR'd together.
     *                ANALOGINPUT_INTEGRATE - do integration on sensor data.
     *                ANALOGINPUT_DOUBLE_INTEGRATE - do double integration on sensor data.
     * @param filters specifies an array of filter objects, one for each axis, to filter sensor data. If no filter
     *                is used, this can be set to null.
     */
    public TrcAnalogInput(String instanceName, int numAxes, int options, TrcFilter[] filters)
    {
        super(instanceName, numAxes, filters);
        this.instanceName = instanceName;
        //
        // Create the data integrator. Data integrator needs data providers to provide processed data from the sensor.
        //
        if ((options & ANALOGINPUT_INTEGRATE) != 0)
        {
            dataIntegrator = new TrcDataIntegrator<>(
                    instanceName, this, DataType.INPUT_DATA, (options & ANALOGINPUT_DOUBLE_INTEGRATE) != 0);
        }
    }   //TrcAnalogInput

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param numAxes specifies the number of axes.
     * @param options specifies the AnalogInput options. Multiple options can be OR'd together.
     *                ANALOGINPUT_INTEGRATE - do integration on sensor data.
     *                ANALOGINPUT_DOUBLE_INTEGRATE - do double integration on sensor data.
     */
    public TrcAnalogInput(String instanceName, int numAxes, int options)
    {
        this(instanceName, numAxes, options, null);
    }   //TrcAnalogInput

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param numAxes specifies the number of axes supporting by the analog input device.
     */
    public TrcAnalogInput(String instanceName, int numAxes)
    {
        this(instanceName, numAxes, 0, null);
    }   //TrcAnalogInput

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
     * The method enables/disables the processing of sensor data. It is not automatically enabled when the
     * TrcAnalogInput object is created. You need to explicitly enable the it before data processing will
     * start. As part of enabling the sensor, calibrate() is also called. calibrate() may be overridden by
     * the platform dependent sensor if it is capable of doing its own. Otherwise, calibrate will call the
     * built-in calibrator to do the calibration. Enabling/disabling data processing for the sensor involves
     * enabling/disabling the integrator if it exists.
     *
     * @param enabled specifies true if enabling, false otherwise.
     */
    public void setEnabled(boolean enabled)
    {
        //
        // Enable/disable integrator.
        //
        if (dataIntegrator != null)
        {
            dataIntegrator.setEnabled(enabled);
        }
    }   //setEnabled

    /**
     * This method enables/disables the elapsed timers for performance monitoring.
     *
     * @param enabled specifies true to enable elapsed timers, false to disable.
     */
    public static void setElapsedTimerEnabled(boolean enabled)
    {
        if (enabled)
        {
            if (getInputElapsedTimer == null)
            {
                getInputElapsedTimer = new TrcElapsedTimer("TrcAnalogInput.getInput", 2.0);
            }
        }
        else
        {
            getInputElapsedTimer = null;
        }
    }   //setElapsedTimerEnabled

    /**
     * This method prints the elapsed time info using the given tracer.
     *
     * @param tracer specifies the tracer to be used to print the info.
     */
    public static void printElapsedTime(TrcDbgTrace tracer)
    {
        if (getInputElapsedTimer != null)
        {
            getInputElapsedTimer.printElapsedTime(tracer);
        }
    }   //printElapsedTime

    /**
     * This method inverts the sensor data. This is useful if the orientation of the sensor is such that the data
     * goes the wrong direction.
     *
     * @param inverted specifies true to invert sensor data, false otherwise.
     */
    public void setInverted(boolean inverted)
    {
        setInverted(0, inverted);
    }   //setInverted

    /**
     * This method sets the scale factor on the sensor data.
     *
     * @param scale specifies the scale factor.
     * @param offset specifies the offset to be subtracted from the scaled data.
     */
    public void setScaleAndOffset(double scale, double offset)
    {
        super.setScaleAndOffset(0, scale, offset);
    }   //setScaleAndOffset

    /**
     * This method sets the scale factor on the sensor data.
     *
     * @param scale specifies the scale factor.
     */
    public void setScale(double scale)
    {
        super.setScale(0, scale);
    }   //setScale

    /**
     * This method returns the processed sensor data of the specified index.
     *
     * @param index specifies the data index.
     * @return processed data.
     */
    public TrcSensor.SensorData<Double> getData(int index)
    {
        return getProcessedData(index, DataType.INPUT_DATA);
    }   //getData

    /**
     * This method returns the processed and normalized sensor data of the specified index.
     *
     * @param index specifies the data index.
     * @return processed normalized data.
     */
    public TrcSensor.SensorData<Double> getNormalizedData(int index)
    {
        return getProcessedData(index, DataType.NORMALIZED_DATA);
    }   //getNormalizedData

    /**
     * This method returns the integrated sensor data of the specified index.
     *
     * @param index specifies the data index.
     * @return integrated sensor data.
     */
    public TrcSensor.SensorData<Double> getIntegratedData(int index)
    {
        return dataIntegrator != null ? dataIntegrator.getIntegratedData(index) :
                                        getRawData(index, DataType.DOUBLE_INTEGRATED_DATA);
    }   //getIntegratedData

    /**
     * This method returns the double integrated sensor data of the specified index.
     *
     * @param index specifies the data index.
     * @return double integrated sensor data.
     */
    public TrcSensor.SensorData<Double> getDoubleIntegratedData(int index)
    {
        return dataIntegrator != null ? dataIntegrator.getDoubleIntegratedData(index) :
                                        getRawData(index, DataType.DOUBLE_INTEGRATED_DATA);
    }   //getDoubleIntegratedData

    //
    // The following methods can be overridden by a platform dependent AnalogInput class.
    //

    /**
     * This method resets the integrator of the specified index.
     *
     * @param index specifies the data index.
     */
    public void resetIntegrator(int index)
    {
        if (dataIntegrator != null)
        {
            dataIntegrator.reset(index);
        }
    }   //resetIntegrator

}   //class TrcAnalogInput
