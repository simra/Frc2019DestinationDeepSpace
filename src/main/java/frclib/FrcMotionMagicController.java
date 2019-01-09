/*
 * Copyright (c) 2018 Titan Robotics Club (http://www.titanrobotics.com)
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

package frclib;

import com.ctre.phoenix.motorcontrol.ControlMode;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.can.TalonSRXPIDSetConfigUtil;
import com.ctre.phoenix.motorcontrol.can.TalonSRXPIDSetConfiguration;
import trclib.TrcDbgTrace;
import trclib.TrcEvent;
import trclib.TrcPidController;
import trclib.TrcRobot;
import trclib.TrcTaskMgr;
import trclib.TrcUtil;

/**
 * This class implements motion magic using the TalonSRX. This is mainly used for linear movement ONLY in the y axis.
 * It cannot be used in conjunction with x-axis movement or rotation. It uses the motion magic features of the TalonSRX
 * to run high resolution closed loop control using trapezoidal velocity profiles. This should result in more accurate
 * and more constrained motion than regular PID control. It is more constrained as you can set a real-world maximum
 * velocity and acceleration of the robot.
 */

public class FrcMotionMagicController
{
    private TrcPidController.PidCoefficients pidCoefficients;
    private TrcPidController.PidCoefficients turnPidCoefficients = new TrcPidController.PidCoefficients(0);
    private FrcCANTalon leftMaster, rightMaster;
    private double worldUnitsPerTick;
    private TrcEvent onFinishedEvent;
    private TrcTaskMgr.TaskObject motionMagicTaskObj;
    private boolean running = false;
    private boolean cancelled = false;
    private double timeoutTime;
    private boolean autoTimeout = false;
    private double fudgeFactor = 1.0;
    /**
     * Sensor units.
     */
    private double errorTolerance;
    /**
     * Sensor units.
     */
    private double targetPos;
    /**
     * Sensor units per 100ms.
     */
    private int maxVelocity;
    /**
     * Sensor units per 100ms per second. Yeah I know it's gross.
     */
    private int maxAcceleration;

    /**
     * Creates a motion magic controller with a default pid slot of 0.
     *
     * @param instanceName      The name of this instance.
     * @param worldUnitsPerTick The number of world units per encoder tick. This can be inches/tick, cm/tick, etc.
     *                          Whatever is used, MAKE SURE TO BE CONSISTENT.
     * @param pidCoefficients   The PIDF coefficients to use for the drivetrain.
     * @param maxVelocity       The maximum speed the robot should go during a move operation.
     *                          The robot may not reach this speed. This should be in world units per second.
     * @param maxAcceleration   The maximum acceleration of the robot during a move operation.
     *                          The robot may not reach this acceleration. This should be in world units per second per second.
     * @param errorTolerance    The tolerance of error, in world units. If the closed loop error is less than or equal to
     *                          the tolerance, the move operation will be finished.
     */
    public FrcMotionMagicController(String instanceName, double worldUnitsPerTick,
        TrcPidController.PidCoefficients pidCoefficients, double maxVelocity, double maxAcceleration,
        double errorTolerance)
    {
        this.worldUnitsPerTick = worldUnitsPerTick;
        this.pidCoefficients = pidCoefficients;
        // Convert to encoder units
        this.errorTolerance = Math.abs(errorTolerance) / worldUnitsPerTick;
        // Scale velocity and acceleration to encoder units and time frame of 100ms
        this.maxVelocity = TrcUtil.round(0.1 * maxVelocity / worldUnitsPerTick);
        // For some reason CTRE is dumb, so acceleration is ticks/100ms/1sec. God I hate these people.
        this.maxAcceleration = TrcUtil.round(0.1 * maxAcceleration / worldUnitsPerTick);

        this.motionMagicTaskObj = TrcTaskMgr.getInstance().createTask(instanceName, this::motionMagicTask);
    }

    /**
     * Start motion magic move operation.
     *
     * @param targetPos The target position in world units to move to.
     */
    public void drive(double targetPos)
    {
        drive(targetPos, null);
    }

    /**
     * Start motion magic move operation.
     *
     * @param targetPos       The target position in world units to move to.
     * @param onFinishedEvent The event to signal when done.
     */
    public void drive(double targetPos, TrcEvent onFinishedEvent)
    {
        double timeout = autoTimeout ? estimatePathDuration(targetPos) : Double.POSITIVE_INFINITY;
        drive(targetPos, onFinishedEvent, timeout);
    }

    /**
     * Start motion magic move operation.
     *
     * @param targetPos       The target position in world units to move to.
     * @param onFinishedEvent The event to signal when done.
     * @param timeout         Number of seconds after which to cancel the movement.
     */
    public void drive(double targetPos, TrcEvent onFinishedEvent, double timeout)
    {
        if (leftMaster == null || rightMaster == null)
        {
            throw new IllegalStateException("Cannot start before setting both left and right motors!");
        }

        this.timeoutTime = TrcUtil.getCurrentTime() + timeout;

        // Convert target to encoder units
        targetPos /= worldUnitsPerTick;
        this.targetPos = targetPos;

        if (onFinishedEvent != null)
        {
            onFinishedEvent.clear();
        }
        this.onFinishedEvent = onFinishedEvent;

        talonInit();

        // Set the target pos to the required position and the target angle to 0
        // This target angle is NOT absolute heading. The motors were just zeroed, so it should maintain a current heading.
        rightMaster.motor.set(ControlMode.MotionMagic, targetPos, DemandType.AuxPID, 0.0);
        leftMaster.motor.follow(rightMaster.motor, FollowerType.AuxOutput1);

        running = true;
        cancelled = false;
        setTaskEnabled(true);
    }

    /**
     * Configure auto timeout to be enabled or not. If enabled, unless otherwise specified, automatically compute an
     * upper bound of time required to calculate a timeout value. Fudge factor of 1.1.
     *
     * @param enabled If true, enable auto timeout. Otherwise, disable it.
     */
    public void setAutoTimeoutEnabled(boolean enabled)
    {
        setAutoTimeoutEnabled(enabled, 1.1);
    }

    /**
     * Configure auto timeout to be enabled or not. If enabled, unless otherwise specified, automatically compute an
     * upper bound of time required to calculate a timeout value.
     *
     * @param enabled     If true, enable auto timeout. Otherwise, disable it.
     * @param fudgeFactor Arbitrary value to multiply estimate by to get closer to "real world".
     */
    public void setAutoTimeoutEnabled(boolean enabled, double fudgeFactor)
    {
        autoTimeout = enabled;
        this.fudgeFactor = fudgeFactor;
    }

    /**
     * Sets the error tolerance. If the closed loop error is less than or equal to the tolerance,
     * the move operation will be finished.
     *
     * @param errorTolerance The new error tolerance, in world units.
     */
    public void setErrorTolerance(double errorTolerance)
    {
        this.errorTolerance = errorTolerance / worldUnitsPerTick;
    }

    /**
     * Cancel the current move operation.
     */
    public void cancel()
    {
        if (isRunning())
        {
            cancelled = true;
            if (onFinishedEvent != null)
            {
                // Ideally, this should be cancelled instead of signalled, but nobody else ever actually checks the
                // cancelled flag. Whatever.
                onFinishedEvent.set(true);
            }
            stop();
        }
    }

    /**
     * Is there a current move operation in progress?
     *
     * @return True if the motion magic is running, false otherwise.
     */
    public boolean isRunning()
    {
        return running;
    }

    /**
     * Was the last move operation cancelled prematurely?
     *
     * @return True if the last move operation was cancelled, false otherwise.
     */
    public boolean isCancelled()
    {
        return cancelled;
    }

    /**
     * Get the target position in world units.
     *
     * @return The target position in world units.
     */
    public double getTargetPos()
    {
        return targetPos * worldUnitsPerTick;
    }

    /**
     * Get the closed-loop error, in world units.
     *
     * @return The closed loop error in world units.
     */
    public double getError()
    {
        return worldUnitsPerTick * getRawError();
    }

    /**
     * Set the PIDF coefficients. This will only apply for the next time start() is called. It will NOT affect a run
     * already in progress.
     *
     * @param pidCoefficients The PID coefficients to set.
     */
    public void setPidCoefficients(TrcPidController.PidCoefficients pidCoefficients)
    {
        this.pidCoefficients = pidCoefficients;
    }

    /**
     * Set the PIDF coefficients for the turn correction. This will only apply for the next time start() is called.
     * It will NOT affect a run already in progress. The turn correction controller will apply adjustments to make the
     * robot drive straight.
     *
     * @param turnPidCoefficients The PID coefficients to set for the turn correction controller. If null, disable the
     *                            turn pid loop.
     */
    public void setTurnPidCoefficients(TrcPidController.PidCoefficients turnPidCoefficients)
    {
        this.turnPidCoefficients =
            turnPidCoefficients != null ? turnPidCoefficients : new TrcPidController.PidCoefficients(0.0);
    }

    /**
     * Sets the motors on the left side of the drive train.
     *
     * @param leftMotors List of motors on the left side of the drive train. The first motor in the list will be used
     *                   as the master motor, and all others will be set as slaves.
     */
    public void setLeftMotors(FrcCANTalon... leftMotors)
    {
        if (leftMotors.length == 0)
        {
            throw new IllegalArgumentException("Cannot pass empty array of motors!");
        }

        this.leftMaster = leftMotors[0];

        if (leftMotors.length > 1)
        {
            for (int i = 1; i < leftMotors.length; i++)
            {
                leftMotors[i].motor.set(ControlMode.Follower, leftMaster.motor.getDeviceID());
            }
        }
    }

    /**
     * Sets the motors on the right side of the drive train.
     *
     * @param rightMotors List of motors on the right side of the drive train. The first motor in the list will be used
     *                    as the master motor, and all others will be set as slaves.
     */
    public void setRightMotors(FrcCANTalon... rightMotors)
    {
        if (rightMotors.length == 0)
        {
            throw new IllegalArgumentException("Cannot pass empty array of motors!");
        }

        this.rightMaster = rightMotors[0];

        if (rightMotors.length > 1)
        {
            for (int i = 1; i < rightMotors.length; i++)
            {
                rightMotors[i].motor.set(ControlMode.Follower, rightMaster.motor.getDeviceID());
            }
        }
    }

    private double getRawError()
    {
        double leftError = targetPos - leftMaster.motor.getSelectedSensorPosition(0);
        double rightError = targetPos - rightMaster.motor.getSelectedSensorPosition(0);
        return TrcUtil.average(leftError, rightError);
    }

    private boolean isDone()
    {
        // TODO: This needs to be improved. What if it stalls just outside of tolerance range?
        // Maybe add a timeout. Or if it spends x seconds under a certain velocity. As it is now it's not very robust.
        return running && Math.abs(getRawError()) <= errorTolerance;
    }

    private void stop()
    {
        setTaskEnabled(false);
        onFinishedEvent = null;
        running = false;
        targetPos = 0.0;
        // Reset the selected feedback device to the cached value
        rightMaster.motor.configSelectedFeedbackSensor(rightMaster.getFeedbackDevice());
        // Stop the motors
        leftMaster.motor.neutralOutput();
        rightMaster.motor.neutralOutput();
    }

    private void talonInit()
    {
        // Set the pid coefficients for the primary and auxiliary controllers
        configureCoefficients(rightMaster, pidCoefficients, 0);
        configureCoefficients(rightMaster, turnPidCoefficients, 1);

        rightMaster.motor.configAllowableClosedloopError(0, TrcUtil.round(errorTolerance), 0);

        // Set the motion magic velocity and acceleration constraints
        rightMaster.motor.configMotionCruiseVelocity(maxVelocity, 0);
        rightMaster.motor.configMotionAcceleration(maxAcceleration, 0);

        // Add the left encoder as a remote sensor to the right controller
        rightMaster.motor
            .configRemoteFeedbackFilter(leftMaster.motor.getDeviceID(), RemoteSensorSource.TalonSRX_SelectedSensor, 0,
                0);

        // Configure the remote sensor and local sensor to be summed
        rightMaster.motor.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0, 0);
        rightMaster.motor.configSensorTerm(SensorTerm.Sum1, rightMaster.getFeedbackDevice(), 0);

        // Configure the remote sensor and local sensor to be subtracted
        // We operate in clockwise rotation so it's inverted polarity
        rightMaster.motor.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.RemoteSensor0, 0);
        rightMaster.motor.configSensorTerm(SensorTerm.Diff1, rightMaster.getFeedbackDevice(), 0);

        // Multiply the sum by 0.5. (basically average the sensor readings) Use the average in slot 0
        rightMaster.motor.configSelectedFeedbackSensor(FeedbackDevice.SensorSum, 0, 0);
        rightMaster.motor.configSelectedFeedbackCoefficient(0.5, 0, 0);

        // TODO: Maybe have to scale it by some amount so it fits into integers? (Possibly set a feedback coefficient)
        // Use the differences in sensor readings in slot 1
        // The old feedback sensor is restored after the motion magic finishes
        rightMaster.motor.configSelectedFeedbackSensor(FeedbackDevice.SensorDifference, 1, 0);

        // Configure the polarity of the controllers. Since we use clockwise rotation, we use inverted polarity.
        rightMaster.motor.configAuxPIDPolarity(true, 0);

        // Zero the encoders
        leftMaster.motor.setSelectedSensorPosition(0, 0, 0);
        rightMaster.motor.setSelectedSensorPosition(0, 0, 0);
    }

    /**
     * Estimates the duration of the path in seconds. Uses a trapezoidal velocity profile with the provided constraints.
     *
     * @param distance The distance moved in by the path.
     * @return The estimated number of seconds required to move this path. This *should* be an upper bound.
     */
    private double estimatePathDuration(double distance)
    {
        // For convenience, these calculations will be done in world units.
        double maxVelocity = this.maxVelocity * worldUnitsPerTick * 10; // convert to world units per second
        double maxAcceleration = this.maxAcceleration * worldUnitsPerTick * 10; // convert to world units per second

        // v/t = a
        // vt/2 = d
        // v = max vel, a = max acc, t = time, d = distance
        double rampUpDistance = Math.pow(maxVelocity, 2) / (2.0 * maxAcceleration);
        double estimate;
        if (rampUpDistance * 2 <= distance)
        {
            double cruiseDistance = distance - (rampUpDistance * 2.0);
            double cruiseTime = cruiseDistance / maxVelocity;
            double rampUpTime = maxVelocity / maxAcceleration;
            estimate = rampUpTime * 2 + cruiseTime;
        }
        else
        {
            double halfTime = Math.sqrt(distance / maxAcceleration);
            estimate = 2 * halfTime;
        }
        return estimate * fudgeFactor;
    }

    private void configureCoefficients(FrcCANTalon talon, TrcPidController.PidCoefficients pidCoefficients, int pidSlot)
    {
        talon.motor.config_kP(pidSlot, pidCoefficients.kP, 0);
        talon.motor.config_kI(pidSlot, pidCoefficients.kI, 0);
        talon.motor.config_kD(pidSlot, pidCoefficients.kD, 0);
        talon.motor.config_kF(pidSlot, pidCoefficients.kF, 0);
        talon.motor.config_IntegralZone(pidSlot, pidCoefficients.iZone, 0);
    }

    private void setTaskEnabled(boolean enabled)
    {
        if (enabled)
        {
            motionMagicTaskObj.registerTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        }
        else
        {
            motionMagicTaskObj.unregisterTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        }
    }

    private void motionMagicTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode mode)
    {
        if (isDone())
        {
            TrcDbgTrace.getGlobalTracer().traceInfo("FrcMotionMagicController.task", "Done!");
            if (onFinishedEvent != null)
            {
                onFinishedEvent.set(true);
            }
            stop();
        }
        else if (TrcUtil.getCurrentTime() >= timeoutTime)
        {
            cancel();
        }
    }
}