/*
 * Copyright (c) 2021 Titan Robotics Club (http://www.titanrobotics.com)
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

import teamcode.javalimelight.trclib.robotcore.TrcEvent;

/**
 * This interface specifies a collection of methods to implement an asynchronous trigger. For example, a trigger can
 * be generated by a digital sensor, an analog sensor or any value source. For a digital sensor, a trigger is
 * generated when it changes from an inactive state to an active state or vise versa. For an analog sensor, a trigger
 * is generated when the sensor value crosses a given set of thresholds. This class is intended to be implemented by
 * different sensor trigger classes.
 */
public interface TrcTrigger
{
    public enum TriggerMode
    {
        OnActive,
        OnInactive,
        OnBoth
    }   //enum TriggerMode

    /**
     * This method arms the digital trigger. It enables the task that monitors the sensor state changes.
     *
     * @param triggerDelay specifies the delay for arming the trigger.
     * @param triggerMode specifies the trigger mode that will signal the event.
     * @param event specifies the event to signal when the trigger state changed.
     */
    void enableTrigger(double triggerDelay, TriggerMode triggerMode, TrcEvent event);

    /**
     * This method arms the digital trigger. It enables the task that monitors the sensor state changes.
     *
     * @param triggerMode specifies the trigger mode that will signal the event.
     * @param event specifies the event to signal when the trigger state changed.
     */
    default void enableTrigger(TriggerMode triggerMode, TrcEvent event)
    {
        enableTrigger(0.0, triggerMode, event);
    }   //enableTrigger

    /**
     * This method arms the digital trigger. It enables the task that monitors the sensor state changes.
     *
     * @param triggerDelay specifies the delay for arming the trigger.
     * @param triggerMode specifies the trigger mode that will trigger a callback.
     * @param callback specifies the callback handler to notify when the trigger state changed.
     */
    void enableTrigger(double triggerDelay, TriggerMode triggerMode, TrcEvent.Callback callback);

    /**
     * This method arms the digital trigger. It enables the task that monitors the sensor state changes.
     *
     * @param triggerMode specifies the trigger mode that will trigger a callback.
     * @param callback specifies the callback handler to notify when the trigger state changed.
     */
    default void enableTrigger(TriggerMode triggerMode, TrcEvent.Callback callback)
    {
        enableTrigger(0.0, triggerMode, callback);
    }   //enableTrigger

    /**
     * This method disarms the trigger. It disables the task that monitors the sensor value.
     */
    void disableTrigger();

    /**
     * This method checks if the trigger task is enabled.
     *
     * @return true if enabled, false otherwise.
     */
    boolean isEnabled();

    /**
     * This method reads the current analog sensor value. For digital sensor trigger, it will throw a RuntimeException.
     *
     * @return current sensor value.
     */
    double getSensorValue();

    /**
     * This method reads the current digital sensor state. For analog sensor trigger, it will throw a RuntimeException.
     *
     * @return current sensor state.
     */
    boolean getSensorState();

}   //interface TrcTrigger
