CRUISE CONTROL SYSTEM REQUIREMENT SPECIFICATION
1	OVERVIEW
This document describes a requirement specification for an automobile cruise control system.  The cruise control system provides functionality to control the speed of the vehicle automatically.
2	SYSTEM OVERVIEW
2.1	SYSTEM INPUTS
2.1.1	Cruise control buttons
Five buttons are installed onto the driver’s dashboard.
•	Cruise: A button to activate cruise control.
•	Cancel: A button to temporarily deactivate cruise control navigation. 
•	Set: A button to set vehicle speed when the cruise control system is enabled.
•	Res: A button to reset the vehicle to the previous set speed.
•	Inc: A button to increase the vehicle speed. A short press increases the speed by a given amount. Long pressing the button keeps increasing the target speed by a given amount over time.
•	Dec: A button to decrease the vehicle speed. A short press decreases the speed by a given amount. Long pressing the button keeps decreasing the target speed by a given amount over time.
2.1.2	Other inputs
•	Current vehicle speed
•	Throttle value from the accelerator pedal
•	Key position (Lock, Acc, ON, Start)
•	Gear position (Park, Neutral, Drive, Reverse)
2.2	CRUISE CONTROL MODE INDICATOR
Two indicator lights are installed onto the instrument panel.
•	CRUISE: Turned on when the cruise control system is activated.
•	SET: Turned on when the cruise control system is regulating the vehicle speed.
2.3	CRUISE CONTROL MODES
There are three modes for the cruise control system:
•	Disabled
The cruise control system is not enabled. 
•	Enabled
The cruise control system is operational and is ready to control the vehicle speed but the controls has not yet been taken over by the system.
•	Activated
The cruise control system is taking over the driver’s throttle control. The driver can increase the throttle value to a value greater than is specified by the cruise control system.

3		FUNCTIONAL REQUIREMENTS
3.1	ENABLING CRUISE CONTROL
Cruise control is enabled when the following conditions are met:
•	Vehicle speed is within the target speed range (40km/h – 100km/h).
•	Key position is ON.
•	Gear position is Drive.
•	Cruise button is pushed while the cruise control mode is disabled.
Dashboard image
 

3.2	DISABLING CRUISE CONTROL
Cruise control is disabled when one or more of the following are met:
•	Key position is set to any other position than ON.
•	When the vehicle is started. Cruise button is pushed while the cruise control mode is enabled or activated.
•	Gear position is not Drive
Dashboard image
 
3.3	ACTIVATING CRUISE CONTROL
Cruise control is activated when the following conditions are met:
•	Cruise control mode is enabled.
•	Set button is pushed.
•	Vehicle speed is in the target speed range (40-100 km/h)
•	Gear position is Drive.
 
The target speed is set to a current vehicle speed when the Set button is pushed while the cruise control mode is enabled.
Dashboard image
 
3.4	DEACTIVATING CRUISE CONTROL
Cruise control is disabled when one or more of the following conditions are met:
•	Brake pedal is pressed.
•	Cancel button is pushed.
•	Vehicle speed is not in the target speed range (40-100 km/h)
•	Gear position is not Drive.
Deactivating actions override any other button operations except for the disabling operation.
Dashboard image
 

3.5	TARGET SPEED INCREMENT
While the cruise control mode is activated, the driver can increase the target speed by pushing the Set+ button.  When the button is pressed and is released in less than 500ms, the target speed is increased by 1 km/h.
Dashboard image

 

3.6	TARGET SPEED DECREMENT
While the cruise control mode is activated, the driver can decrease the target speed by pushing the Set- button.  When the button is pressed and is released in less than 500ms, the target speed is decreased by 1 km/h.
Dashboard image
 
3.7	SUCCESSIVE TARGET SPEED INCREMENT
While the cruise control mode is activated, the driver can increase the target speed by pushing the Set+ button.   When the button is depressed and held for 500ms, the target speed begins increasing by 5km/h every second until the button is released.
Dashboard image
 
3.8	SUCCESSIVE TARGET SPEED DECREMENT
While the cruise control mode is activated, the driver can decrease the target speed by pushing the Set- button.  When the button is depressed and held for 500ms, the target speed begins decreasing by 5km/h every second until the button is released.

3.9	ADJUSTING TARGET SPEED WITH ACCELERATOR PEDAL
While the cruise control mode is activated, the driver can increase the target speed by using the accelerator pedal.
When the accelerator pedal is depressed by more than 15%, the cruise control system is temporarily disabled from controlling the engine throttle and the vehicle speed may increase or decrease.  When the accelerator pedal is depressed by less than 5%, the cruise control is returned to active mode with the target speed adjusted to the current vehicle speed.
3.10	RESUMING CRUISE CONTROL
Cruise control can be resumed after being activated provided the mode was not disabled. Cruise control is resumed to keep previously set target speed when the following conditions are met:
•	Cruise control mode is enabled.
•	There was a transition from activated to enabled at least one time and the cruise control system is not disabled since then.
•	Resume button is pushed.
•	Vehicle speed is in the target speed range (40-100 km/h)
•	Gear position is Drive.
Dashboard image
 
3.11	THROTTLE VALUE CALCULATION
The cruise control system calculates the throttle value using PID controller. Gain parameters are determined to satisfy the following conditions:
•	If the cruise control mode is activated when current vehicle speed is 40km/h and the target speed is 60km/h, the vehicle speed is maintained at 60km/h +- 5% within 20 sec.
•	If the cruise control mode is activated when current vehicle speed is 80km/h and the target speed is 60km/h, the vehicle speed is maintained at 60km/h +- 5% within 20 sec.
Cruise Control Indicator Light
Cruise control indicator light is displayed whenever the cruise control mode is enabled.  
When the ignition switch transitions from OFF to ON the cruise control indicator icon is displayed until either the ignition switch enters CRANK mode or 3 seconds have elapsed.
Dashboard image
 
3.12	CRUISE CONTROL SET INDICATOR LIGHT
Cruise control SET indicator light is displayed whenever the cruise control mode is activated.
Dashboard image
 
4	INTERFACE SPECIFICATION
4.1	SYSTEM INPUTS
Name	Data type	Units	Description
enbl	boolean		Cruise button
cncl	boolean		Cancel button
set	boolean		Set button
resume	boolean		Resume button
inc	boolean		Inc button
dec	boolean		Dec button
brakeP	single	kPa	Brake pressure
key	uint8		Key Position
gear	uint8		Shift Position
vehicle_speed	single	km/h	Vehicle speed
driver_throttle	single	%	Throttle from acceleration pedal
4.2	SYSTEM OUTPUTS
Name	Data type	Units	Description
reqDrv	Enum: reqMode		Driver’s request
status	boolean		Cruise control system status
mode	Enum: opMode		Operation mode of the cruise control system
cc_target_speed	single	km/h	Target speed
Target speed that controlled by the cruise control system
cc_throttle	single	%	Output throttle value
Target throttle value controlled by the cruise control system

4.3	ROM

Name	Value	Data type	Units	Description
Kd	-0.0249	single		
Ki	0.2	single		
Kp	20	single		
brake_threshold	5	single	kPa	Brake recognition threshold
Minimum brake pressure that is recognized as the brake pedal is depressed by cruise control system
target_dec_hold	5	single	km/h	Decremental velocity 
Decremental velocity amount per second when Dec button is hold more than 1 sec
target_dec_short	1	single	km/h	Decremental velocity amount when Dec button is pushed 1 time
target_inc_hold	5	single	km/h	Incremental velocity amount per second when Inc button is hold more than 1 sec
target_inc_short	1	single	km/h	Incremental velocity amount when Inc button is pushed 1 time
tsp_max 	100	single	km/h	Maximum target vehicle velocity that can be set by user as cruise control navigation
tsp_min 	40	single	km/h	Minimum target vehicle velocity that can be set by user as cruise control navigation
throttle_max	100	single	%	Maximum throttle amount
throttle_min	0	single	%	Minimum throttle amount

