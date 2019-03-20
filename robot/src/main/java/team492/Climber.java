/*
 * Copyright (c) 2019 Titan Robotics Club (http://www.titanrobotics.com)
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

package team492;

import frclib.FrcCANTalon;
import frclib.FrcCANTalonLimitSwitch;
import trclib.TrcDigitalTrigger;
import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTaskMgr;

public class Climber
{
    private enum State
    {
        INIT_SUBSYSTEMS, MANUAL_CLIMB
    }

    public enum HabLevel
    {
        LEVEL_2(RobotInfo.CLIMBER_ELEVATOR_POS_LVL_2), LEVEL_3(RobotInfo.CLIMBER_ELEVATOR_POS_LVL_3);

        private double height;

        HabLevel(double height)
        {
            this.height = height;
        }

        public double getHeight()
        {
            return height;
        }
    }

    private FrcCANTalon actuator;
    private FrcCANTalon climberWheels;
    private Robot robot;
    private TrcTaskMgr.TaskObject climbTaskObj;
    private TrcStateMachine<State> sm;
    private HabLevel level;
    private TrcDigitalTrigger actuatorLowerLimitSwitchTrigger;
    private boolean calibrating = false;

    public Climber(Robot robot)
    {
        this.robot = robot;

        actuator = new FrcCANTalon("ClimberActuator", RobotInfo.CANID_CLIMB_ACTUATOR); // this should be 7, eventually
        actuator.motor.overrideLimitSwitchesEnable(true);
        actuator.configFwdLimitSwitchNormallyOpen(false);
        actuator.configRevLimitSwitchNormallyOpen(false);
        actuator.setBrakeModeEnabled(true);

        FrcCANTalonLimitSwitch actuatorLowerLimitSwitch = new FrcCANTalonLimitSwitch("ActuatorLowerLimit", actuator,
            false);
        actuatorLowerLimitSwitchTrigger = new TrcDigitalTrigger("TrcDigitalTrigger", actuatorLowerLimitSwitch,
            this::lowerLimitSwitchEvent);
        actuatorLowerLimitSwitchTrigger.setEnabled(true);

        climberWheels = new FrcCANTalon("ClimberWheels", RobotInfo.CANID_CLIMB_WHEELS);
        climberWheels.setInverted(true);
        climberWheels.setBrakeModeEnabled(true);

        climbTaskObj = TrcTaskMgr.getInstance().createTask("ClimberTask", this::climbTask);

        sm = new TrcStateMachine<>("ClimbStateMachine");
    }

    private void lowerLimitSwitchEvent(boolean triggered)
    {
        actuator.resetPosition(true);
        if (calibrating)
        {
            actuator.set(0.0);
            calibrating = false;
        }
    }

    public void zeroCalibrateActuator()
    {
        actuatorLowerLimitSwitchTrigger.setEnabled(true);
        actuator.set(RobotInfo.CLIMBER_ACTUATOR_CAL_POWER);
        calibrating = true;
    }

    public void setWheelPower(double power)
    {
        climberWheels.set(power);
    }

    public boolean getLowerLimitSwitch()
    {
        return actuator.isLowerLimitSwitchActive();
    }

    public boolean getUpperLimitSwitch()
    {
        return actuator.isUpperLimitSwitchActive();
    }

    public void setActuatorPower(double power)
    {
        actuator.set(power);
        calibrating = false;
    }

    public void climb(HabLevel level)
    {
        this.level = level;
        sm.start(State.INIT_SUBSYSTEMS);
        setEnabled(true);
        calibrating = false;
    }

    private void setEnabled(boolean enabled)
    {
        if (enabled)
        {
            climbTaskObj.registerTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        }
        else
        {
            climbTaskObj.unregisterTask(TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK);
        }
    }

    public double getActuatorPower()
    {
        return actuator.getPower();
    }

    public double getActuatorRawPos()
    {
        return actuator.getPosition();
    }

    public void cancel()
    {
        actuator.set(0.0);
        robot.elevator.setPower(0.0);
        robot.elevator.setManualOverrideEnabled(false);
        climberWheels.set(0.0);
        robot.pickup.setPitchPower(0.0);
        sm.stop();
        setEnabled(false);
    }

    public boolean isActive()
    {
        return sm.isEnabled();
    }

    private void climbTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        State state = sm.checkReadyAndGetState();
        if (state != null)
        {
            robot.dashboard.displayPrintf(9, "State: %s", state.toString());
            switch (state)
            {
                case INIT_SUBSYSTEMS:
                    TrcEvent pickupEvent = new TrcEvent("PickupEvent");
                    TrcEvent elevatorEvent = new TrcEvent("ElevatorEvent");

                    robot.setHalfBrakeModeEnabled(true);
                    robot.elevator.setPosition(level.getHeight(), elevatorEvent);
                    robot.elevator.getPidController().saveAndSetOutputLimit(0.5);
                    robot.pickup.getPitchPidController().saveAndSetOutputLimit(0.5);
                    robot.pickup.setPickupAngle(RobotInfo.CLIMBER_PICKUP_ANGLE, pickupEvent);
                    climberWheels.set(0.0);

                    robot.elevator.setManualOverrideEnabled(true);
                    robot.pickup.setManualOverrideEnabled(true);

                    sm.addEvent(elevatorEvent);
                    sm.addEvent(pickupEvent);
                    sm.waitForEvents(State.MANUAL_CLIMB, 3.0, true);
                    break;

                case MANUAL_CLIMB:
                    robot.elevator.getPidController().restoreOutputLimit();
                    robot.pickup.getPitchPidController().restoreOutputLimit();
                    double climbPower = robot.operatorStick.getYWithDeadband(true);
                    double syncGain = robot.operatorStick.getTwistWithDeadband(true);
                    double adjustment = syncGain * climbPower;
                    robot.elevator.setPower(-(climbPower - adjustment));
                    actuator.set(climbPower + adjustment);
                    robot.pickup.setPitchPower(RobotInfo.CLIMBER_PICKUP_HOLD_POWER);
                    if (robot.rightDriveStick.getRawButton(8))
                    {
                        robot.driveBase.arcadeDrive(0.0, 0.0);
                        climberWheels.set(robot.rightDriveStick.getYWithDeadband(true));
                    }
                    else
                    {
                        climberWheels.set(0.0);
                        robot.driveBase.arcadeDrive(robot.rightDriveStick.getYWithDeadband(true), 0.0);
                    }
                    break;
            }
        }
    }
}
