//-----------------------------------------------------------------------------//                    
//                       MICROMOUSE - MATRIXBOT - 2022                         //
//-----------------------------------------------------------------------------//

//                Source file:      main_v4.cpp
//                Code version:     v4.6                  
//                Revision date:    29/04/22 - 04.32am 
//                Programmer:       Janitha Gamage
//                Contact:          djg.gamage@gmail.com

// This project contains the final code of a micromouse (maze solving robot) using the NXP LPC1768 Mbed board (based on an Arm Cortex-M3).
// The micromouse is an integrated system comprising of a range of components including sensors for input, a microcontroller board for processing
// and motors, LEDs and a LCD for output. 

//-----------------------------------------------------------------------------//
//-----------------------------------------------------------------------------// 

#include "mbed.h"
#include "TextLCDScroll.h"
#include "ultrasonic.h"

#include "config.h"
#include "pin_description.h"

// Use a union struct to store mapped maze data
union {
	unsigned char all;                            // Map access size is 1 byte
	struct {
		unsigned char north : 1;                  // 1 bit for North wall
		unsigned char east : 1;                   // 1 bit for East wall  
		unsigned char south : 1;                  // 1 bit for South wall
		unsigned char west : 1;                   // 1 bit for West wall   
		unsigned char direction : 4;              // 4 bits for map history  
	};
} mazeMap[MAZE_SIZE][MAZE_SIZE];                // A 16x16 struct is required for a 8x8 maze, stores a total of 256 values

												// Text LCD Class
TextLCDScroll lcd(LCD_RS_PIN, LCD_E_PIN, LCD_D0_PIN, LCD_D1_PIN, LCD_D2_PIN, LCD_D3_PIN, TextLCD::LCD16x2);             // Set the LCD pins, type and size

																														// Ultrasonic Sensor Class
ultrasonic mu(ULTRAOSNIC_TX_PIN, ULTRASONIC_RX_PIN, ULTRASONIC_UPDATE_INTERVAL, ULTRASONIC_TIMEOUT_INTERVAL, &dist);    // Set the trigger pin to D8 and the echo pin to D9
																														// have updates every .1 seconds and a timeout after 1
																														// second, and call dist when the distance changes

unsigned char   programMode(MAP_MAZE);                                          // Stores current program mode

unsigned int    rightMotorSetSteps(0);                                          // Stores number of steps to move for right motor
unsigned int    leftMotorSetSteps(0);                                           // Stores number of steps to move for right motor
unsigned int    rightMotorStepCount(0);                                         // Counter to store number of steps moved by right motor
unsigned int    leftMotorStepCount(0);                                          // Counter to store number of steps moved by left motor
unsigned int    sensorReadingsCount(0);                                         // Counter to store number of sensor readings

																				// Following variables are defined as volatile as they used in an interrupt
volatile unsigned char  rightMotorMode(FORWARD);                                // Stores mode of operation of right motor: FORWARD, REVERSE or STOP
volatile unsigned char  leftMotorMode(FORWARD);                                 // Stores mode of operation of left motor: FORWARD, REVERSE or STOP
volatile unsigned char  rightMotorTimer(TIMER_COUNT_FOR_LOW_SPEED);             // Timer counter to control when right motor is rotated
volatile unsigned char  leftMotorTimer(TIMER_COUNT_FOR_LOW_SPEED);              // Timer counter to control when left motor is rotated
volatile unsigned char  rightMotorSpeed(LOW);                                   // Flag to store speed of right motor
volatile unsigned char  leftMotorSpeed(LOW);                                    // Flag to store speed of left motor

volatile unsigned char  proximitySensorReadings[NUMBER_OF_SENSOR_READINGS];     // Array to store multiple proximity sensor readings
volatile unsigned char  steeringSensorReadings[NUMBER_OF_SENSOR_READINGS];      // Array to store multiple steering sensor readings
volatile unsigned char  sensorWaitingTime(TIMER_COUNT_FOR_SENSORS);             // Timer counter to control when sensors are read, proximity and steering sensors are read at different rates
volatile unsigned char  sensorReadingsCount(0);                                 // Counter to store number of sensor readings taken
volatile unsigned char  sensorReadingsCompare(0);                               // Counter to check number of sensor readings compared
volatile unsigned char  sensorReadingsMatch(0);                                 // Stores flag whether current sensor reading matches with that stored in array
volatile unsigned char  finalProximitySensorReading(0);                         // Stores final proximity sensor reading with most matches
volatile unsigned char  finalSteeringSensorReading(0);                          // Stores final steeering sensor reading with most matches
volatile unsigned char  foundCorrectSensorReading(FALSE);                       // Stores flag whether sensor reading with most matches has been found

DigitalIn   switchBegin(BEGIN_SWITCH_PIN);                                                                  // Switch to begin changing program mode
DigitalIn   switchSet(SET_SWITCH_PIN);                                                                      // Switch to set program mode
BusIn       proximitySensors(RIGHT_PROXIMITY_PIN, CENTER_PROXIMITY_PIN, LEFT_PROXIMITY_PIN);                // BusIn class to store proximity sensor readings; actual order: LEFT, CENTER, RIGHT
BusIn       steeringSensors(RIGHT_STEERING_PIN, LEFT_STEERING_PIN)                                          // BusIn class to store proximity sensor readings; actual order: LEFT, RIGHT

DigitalOut  motorLeftClock(MOTOR_LEFT_CLOCK_PIN);                               // Pin that generates left motor clock signal
DigitalOut  motorRightClock(MOTOR_RIGHT_CLOCK_PIN);                             // Pin that generates right motor clock signal
DigitalOut  motorLeftDirection(MOTOR_LEFT_DIR_PIN);                             // Pin that generates left motor direction signal
DigitalOut  motorRightDirection(MOTOR_RIGHT_DIR_PIN);                           // Pin that generates right motor direction signal

BusOut      onBoardLEDs(LED4, LED3, LED2, LED1);                                // BusOut class to display program mode using LEDs; actual order: LED1, LED2, LED3, LED4

Ticker      TM;                                                                 // Ticker to generate interrupts every 1ms

void isrSensorsandMotors();     // ISR - Reads sensors, performs steering correction and generate clocks for motors
void readSensors();             // Reads sensor values and prints them on a PC terminal using serial USB communication
void stopMotors();              // Stops rotation of motors
void slowStartMotors();         // Starts motors at a lower frequency to avoid skipping steps and slipping
void run1Square();              // Runs 1 square of the maze (600 steps)
void turnRight90();             // Performs a right turn
void turnLeft90();              // Performs a left turn
void turn180();                 // Perform a u-turn
void mapMazeUsingLHR();         // Maps and solver maze using Left hand rule
void dist();                    // Prints distance on LCD

//-----------------------------------------------------------------------------//
//                                MAIN FUNCTION                                //
//-----------------------------------------------------------------------------//
int main() {
	//Initialise
	mu.startUpdates();                          // Set ultrasonic sensor to receive updates

	TM.attach(&isrSensorsandMotors, INTERRUPT_TIME_DELAY);      // Set ticker to generate interrupts and call the function 'isrSensorsandMotors' every 1ms

	// Display text on LCD
	lcd.cls();                                                              // Clears LCD screen
	lcd.setSpeed(9);                                                        // Sets text scrolling speed
	lcd.setLine(0, ">>MatrixBot<<");                                     	// Prints text on first line
	lcd.setLine(1, "Micromouse based on Mbed NXP LPC1768 | Arm Cortex-M3"); // Prints text on second line   
	wait(7);                                                                // 7s delay until all scrolling text is displayed
	lcd.cls();
	lcd.setLine(1, "MOVING FORWARD");

	while (1) {
		mu.checkDistance();                         // Call 'checkDistance' as much as possible, as this is where the class determines if the
													// distance has changed and if 'dist' needs to be called to display new text.
		while (switchBegin == OFF) {                // Repeat while begin switch is not pressed, press begin switch to exit
			if (switchSet == ON) {                  // If set switch is pressed              
				wait(0.01);                         // 10ms delay to neglect initial bouncing                                  
				while (switchSet == ON);            // Wait until set switch is released
				wait(0.01);
				programMode++;                      // Change program mode
				if (programMode > LEES_ALGORITHM)   // If end of program mode list is reached,
					programMode = READ_SENSORS;     // jump back to the beginning of the list
			}
			onBoardLEDs = ProgramMode;              // Display ProgramMode using on-board LEDs
		}
		onBoardLEDs = 0;                            // Turn off on-board LEDs
		wait(0.5);

		// Call function based on selected program mode
		switch (ProgramMode) {
		case  READ_SENSORS:     readSensors();      break;     // Reads sensor values and prints them on a PC terminal using serial USB communication 
		case  RUN_1_SQUARE:     run1Square();       break;     // Runs 1 square of the maze (600 steps) 
		case  TURN_RIGHT_90:    turnRight90();      break;     // Turns right
		case  TURN_LEFT_90:     turnLeft90();       break;     // Turns left  
		case  TURN_180_UTURN:   turn180();          break;     // u-turn
		case  MAP_MAZE:         mapMazeUsingLHR();  break;     // left hand rule
		case  LEES_ALGORITHM:   leesRun();          break;     // fast run for minimum route
		}
	}
}

//-----------------------------------------------------------------------------//
//                         INTERRUPT SERVICE ROUTINE                           //
//-----------------------------------------------------------------------------//
void isrSensorsandMotors(){

	if (sensorsWaitingTime > 0)                                      // If sensor counter is greater than 0, reduce counter
		sensorsWaitingTime--;
	else
		sensorsWaitingTime = TIMER_COUNT_FOR_SENSORS;                // Set sensor counter time for reading sensors

																	 // Take proximity sensor readings for navigation
	if (sensorsWaitingTime == TIMER_COUNT_FOR_PROX_SENSORS){        // If time to read proximity sensors is reached 

																	 // Take multiple proximity sensor readings and store them in 'proximitySensorReadings' array
		for (sensorReadingsCount = 0; sensorReadingsCount < NUMBER_OF_SENSOR_READINGS; sensorReadingsCount++) {
			switch (proximitySensors) {
			case FFF: proximitySensorReadings[sensorReadingsCount] = FFF; break;								// Micromouse Proximity State: 111 
			case FFT: proximitySensorReadings[sensorReadingsCount] = TFF; break;								// Micromouse Proximity State: 110 
			case FTF: proximitySensorReadings[sensorReadingsCount] = FTF; break;								// Micromouse Proximity State: 101 
			case FTT: proximitySensorReadings[sensorReadingsCount] = TTF; break;								// Micromouse Proximity State: 100 
			case TFF: proximitySensorReadings[sensorReadingsCount] = FFT; break;								// Micromouse Proximity State: 011 
			case TFT: proximitySensorReadings[sensorReadingsCount] = TFT; break;								// Micromouse Proximity State: 010 
			case TTF: proximitySensorReadings[sensorReadingsCount] = FTT; break;								// Micromouse Proximity State: 001
			case TTT: proximitySensorReadings[sensorReadingsCount] = TTT;       								// Micromouse Proximity State: 000 
			}
		}

		// Initialise sensor majority voting
		foundCorrectSensorReading = FALSE;

		// Compare and find the sensor reading that has been repeated more than half of the times
		for (sensorReadingsCount = 0; sensorReadingsCount < NUMBER_OF_SENSOR_READINGS; sensorReadingsCount++) {          // Go through all sensor values stored in array
			if (foundCorrectSensorReading == TRUE)                                                                       // Check if correctSensorReading has been found, if not continue
				break;
			finalProximitySensorReading = proximitySensorReadings[sensorReadingsCount]                                  // Assign next value stored in array to temporary variable          
				sensorReadingsMatch = 0;                                                                                    // Set number of matches to zero
			for (sensorReadingsCompare = 0; sensorReadingsCompare < NUMBER_OF_SENSOR_READINGS; sensorReadingsCompare++) { // Go through all sensor values stored in array
				if (finalProximitySensorReading == proximitySensorReadings[sensorReadingsCompare])                      // If value stored in temporary variable matches that in array,
					sensorReadingsMatch++;                                                                              // increase counter for number of matches

				if (sensorReadingsMatch >(NUMBER_OF_SENSOR_READINGS / 2.0) {                                            // If number of matches is greater than half of the number of readings
					foundCorrectSensorReading = TRUE;                                                                   // stop comparison
					break;
				}
			}
		}
	}

	// Take steering sensor readings for steering correction
	if (sensorsWaitingTime == TIMER_COUNT_FOR_STEERING_SENSORS) {    // If time to read steering sensors is reached

																	 // Take multiple distance sensor readings and store them in 'steeringSensorReadings' array
		for (sensorReadingsCount = 0; sensorReadingsCount < NUMBER_OF_SENSOR_READINGS; sensorReadingsCount++) {
			switch (steeringSensors) {
			case LL: steeringSensorReadings[sensorReadingsCount] = LL; break;								// Micromouse Steering State: 11  
			case LH: steeringSensorReadings[sensorReadingsCount] = HL; break;								// Micromouse Steering State: 10
			case HL: steeringSensorReadings[sensorReadingsCount] = LH; break;								// Micromouse Steering State: 01
			case HH: steeringSensorReadings[sensorReadingsCount] = LL;      								// Micromouse Steering State: 00
			}
		}

		// Initialise sensor majority voting
		foundCorrectSensorReading = FALSE;

		// Compare and find the sensor reading that has been repeated more than half of the times
		for (sensorReadingsCount = 0; sensorReadingsCount < NUMBER_OF_SENSOR_READINGS; sensorReadingsCount++) {          // Go through all sensor values stored in array
			if (foundCorrectSensorReading == TRUE)															            // Check if correctSensorReading has been found, if not continue
				break;
			finalSteeringSensorReading = steeringSensorReadings[sensorReadingsCount]                                    // Assign next value stored in array to temporary variable      
				sensorReadingsMatch = 0;                                                                                    // Set number of matches to zero
			for (sensorReadingsCompare = 0; sensorReadingsCompare < NUMBER_OF_SENSOR_READINGS; sensorReadingsCompare++) { // Go through all sensor values stored in array
				if (finalSteeringSensorReading == steeringSensorReadings[sensorReadingsCompare])                        // If value stored in temporary variable matches that in array,
					sensorReadingsMatch++;                                                                              // increase counter for number of matches

				if (sensorReadingsMatch >(NUMBER_OF_SENSOR_READINGS / 2.0) {                                            // If number of matches is greater than half of the number of readings
					foundCorrectSensorReading = TRUE;                                                                   // stop comparison
					break;
				}
			}
		}

		// Read steering sensors and perform steering correction
		switch (finalProximitySensorReading) {
		case FFT:																				// When only right wall exists, follow it
			switch (finalSteeringSensorReading) {
			case LH:	leftMotorSpeed = LOW;	rightMotorSpeed = HIGH;		break;
			case HL:	leftMotorSpeed = HIGH;	rightMotorSpeed = LOW;		break;
			default:	leftMotorSpeed = LOW;	rightMotorSpeed = LOW;
			}
		case TFF:																				// When only left wall exists, follow it
			switch (finalSteeringSensorReading) {
			case LH:	leftMotorSpeed = HIGH;	rightMotorSpeed = LOW;		break;
			case HL:	leftMotorSpeed = LOW;	rightMotorSpeed = HIGH;		break;
			default:	leftMotorSpeed = LOW;	rightMotorSpeed = LOW;
			}
		case TFT:																				// When both walls exists, stay in center of path
			switch (finalSteeringSensorReading) {
			case LH:	leftMotorSpeed = HIGH;	rightMotorSpeed = LOW;		break;
			case HL:	leftMotorSpeed = LOW;	rightMotorSpeed = HIGH;		break;
			default:	leftMotorSpeed = LOW;	rightMotorSpeed = LOW;
			}
		default:    leftMotorSpeed = LOW;	rightMotorSpeed = LOW;
		}
	}

	// Right motor rotation
	if (rightMotorTimer == 0) {                                  // If timer to rotate motors is zero, then rotate motors
		if (rightMotorSpeed == LOW)                             // If speed is low, set higher counter
			rightMotorTimer = TIMER_COUNT_FOR_LOW_SPEED;
		else
			rightMotorTimer = TIMER_COUNT_FOR_HIGH_SPEED;       // If speed is high, set lower counter
		if (rightMotorMode != STOP) {                            // If motorMode is STOP, do not generate clocks
			if (rightMotorMode == FORWARD)                      // else generate clock pulse and direction signal accordingly
				motorRightDirection.write(POSITIVE);
			else if (rightMotorMode == REVERSE)
				motorRightDirection.write(NEGATIVE);
			motorRightClock = !motorRightClock;
			rightMotorStepCount++;                              // Increase right motor step count
		}
	}
	else
		rightMotorTimer--;

	// Left motor rotation   
	if (leftMotorTimer == 0) {                                   // If timer to rotate motors is zero, then rotate motors
		if (leftMotorSpeed == LOW)                              // If speed is low, set higher counter
			leftMotorTimer = TIMER_COUNT_FOR_LOW_SPEED;
		else
			leftMotorTimer = TIMER_COUNT_FOR_HIGH_SPEED;        // If speed is high, set lower counter
		if (leftMotorMode != STOP) {                             // If motorMode is STOP, do not generate clocks
			if (leftMotorMode == FORWARD)                       // else generate clock pulse and direction signal accordingly
				motorLeftDirection.write(NEGATIVE);
			else if (leftMotorMode == REVERSE)
				motorLeftDirection.write(POSITIVE);
			motorLeftClock = !motorLeftClock;
			leftMotorStepCount++;                               // Increase left motor step count
		}
	}
	else
		leftMotorTimer--;
}

//-----------------------------------------------------------------------------//
//                  PRINTS CURRENT DISTANCE IN CM ON LCD                       //
//-----------------------------------------------------------------------------//
void dist(int distance) {
	lcd.printf("Distance:%dcm\r\n", distance / 10);       // Print current distance on display
}

//-----------------------------------------------------------------------------//
//                READ AND PRINT SENSOR VALUES ON PC TERMINAL                  //
//-----------------------------------------------------------------------------//
void readSensors() {
	while (1) {
		COMPort.printf("\f");
		COMPort.printf("Sensor LeftProximity:%f \n", leftProxSensor);                           // Print sensor readings on terminal, one on each line                 
		COMPort.printf("Sensor CenterProximity:%f \n", centerProxSensor);
		COMPort.printf("Sensor RightProximity:%f \n", righttProxSensor);
		COMPort.printf("Sensor SteeringCorrection:%f \n", finalSteeringSensorReading);
		wait(0.5);
	}
}

//-----------------------------------------------------------------------------//
//                         STOPS ROTATION OF MOTORS                            //
//-----------------------------------------------------------------------------//
void stopMotors() {
	lcd.setLine(1, "STATIONARY");                       // Print text on LCD
	rightMotorMode = leftMotorMode = STOP;              // Set motor modes to STOP   
}

//-----------------------------------------------------------------------------//
//                 ROTATES STEPPER MOTORS SLOWLY INITALLY                      //
//-----------------------------------------------------------------------------//
void slowStartMotors() {                                 // Turn motors for 100 steps
	lcd.setLine(1, "SLOW START");                       // Print text on LCD
	rightMotorSpeed = leftMotorSpeed = LOW;             // Set motor speeds to low                         
	rightMotorMode = leftMotorMode = FORWARD;           // Set motor modes to move forward
	rightMotorCountSteps = 0;                           // Set right motor step count to zero
	rightMotorSetSteps = STEPS_FOR_SLOW_START;          // Set number of steps for slow start

	while (rightMotorCountSteps < rightMotorSetSteps);  // Wait until motors are rotated for 100 steps         
}

//-----------------------------------------------------------------------------//
//                   MOVES MOUSE 1 SQUARE FORWARD IN MAZE                      //
//-----------------------------------------------------------------------------//
void run1Square() {                                                 // Turn motors for 600 steps                                                          
	lcd.setLine(1, "RUN 1 SQUARE");                                 // Print text on LCD
	slowStartMotors();                                              // Start motors slowly
	rightMotorSpeed = leftMotorSpeed = HIGH;                        // Set motor speeds to high 
	rightMotorCountSteps = 0;                                       // Set right motor step count to zero
	rightMotorSetSteps = STEPS_FOR_1_SQUARE - STEPS_FOR_SLOW_START; // Set number of steps for 1 square

	while (rightMotorCountSteps < rightMotorSetSteps);              // Wait until motors are rotated for 600 steps
	stopMotors();
}

//-----------------------------------------------------------------------------//
//                      PERFORMS 90 DEGREE RIGHT TURN                          //
//-----------------------------------------------------------------------------//
void turnRight90() {                                    // Turn motors in opposite directions for 220 steps
	lcd.setLine(1, "TURN RIGHT");                       // Print text on LCD
	rightMotorSpeed = leftMotorSpeed = LOW;             // Set motor speeds to low   
	rightMotorCountSteps = 0;                           // Set right motor step count to zero
	rightMotorSetSteps = STEPS_FOR_RIGHT_90_TURN;       // Set number of steps for right turn
	rightMotorMode = BACKWARD;                          // Turn motors in opposite directions
	leftMotorMode = FORWARD;

	while (rightMotorCountSteps < rightMotorSetSteps);  // Wait until motors are rotated for 220 steps  
	stopMotors();                                       // Stop rotation of motors 
}

//-----------------------------------------------------------------------------//
//                      PERFORMS 90 DEGREE LEFT TURN                           //
//-----------------------------------------------------------------------------//
void turnLeft90() {                                     // Turn motors in opposite directions for 220 steps
	lcd.setLine(1, "TURN LEFT");                        // Print text on LCD
	rightMotorSpeed = leftMotorSpeed = LOW;             // Set motor speeds to low 
	leftMotorCountSteps = 0;                            // Set left motor step count to zero
	leftMotorSetSteps = STEPS_FOR_LEFT_90_TURN;         // Set number of steps for left turn  
	rightMotorMode = FORWARD;                           // Turn motors in opposite directions
	leftMotorMode = BACKWARD;

	while (leftMotorCountSteps < leftMotorSetSteps);    // Wait until motors are rotated for 220 steps
	stopMotors();                                       // Stop rotation of motors                            
}

//-----------------------------------------------------------------------------//
//                        PERFORMS 180 DEGREE U-TURN                           //
//-----------------------------------------------------------------------------//
void turn180() {                                        // Turn motors in opposite directions for 440 steps
	lcd.setLine(1, "TURN 180");                         // Print text on LCD
	rightMotorSpeed = leftMotorSpeed = LOW;             // Set motor speeds to low  
	rightMotorCountSteps = 0;                           // Set left motor step count to zero
	rightMotorSetSteps = STEPS_FOR_LEFT_180_TURN;       // Set number of steps for u-turn 
	rightMotorMode = BACKWARD;                          // Turn motors in opposite directions
	leftMotorMode = FORWARD;

	while (rightMotorCountSteps<rightMotorSetSteps);    // Wait until motors are rotated for 440 steps
	stopMotors();                                       // Stop rotation of motors
}

//-----------------------------------------------------------------------------//
//                MAPS MAZE AND SOLVES IT USING LEFT-HAND RULE                 //
//-----------------------------------------------------------------------------//
void mapMazeUsingLHR() {
	// Local variables for maze mapping                 
	unsigned char wallFront(NO_WALL), wallRight(IS_WALL), wallLeft(IS_WALL);            // Stores flags if walls exist walls
	unsigned char wallSense(NO_WALL);                                                   // Stores flag for detecting wall
	unsigned char readMapFront(NO_WALL), readMapRight(NO_WALL), readmapLeft(NO_WALL);   // Used to read the map from memory/ history
	unsigned char mouseXAxis(0), mouseYAxis(0);                                         // Stores axis of micromouse (X and Y)
	unsigned char mouseDirection(NORTH);                                                // Stores current direction of micromouse (NORTH, EAST, SOUTH, WEST)

	const unsigned char ToRightTable[] = { 0, EAST, SOUTH, 0, WEST, 0, 0, 0, NORTH };     // Table indicating to the right direction
	const unsigned char ToLeftTable[] = { 0, WEST, NORTH, 0, EAST, 0, 0, 0, SOUTH };      // Table indicating to the left direction  

	mazeMap[0][0].direction = NORTH;        // Initial starting direction is North

	while (beginSwitch == OFF) {            // Repeat while begin switch is not pressed, press begin switch to exit

											// Read stored wall information from memory
		switch (mouseDirection) {
		case NORTH: if (mouseYAxis < MAZE_SIZE - 1)   readMapFront = mazeMap[mouseYAxis + 1][mouseXAxis].direction;     // When micromouse is moving North
			if (mouseXAxis < MAZE_SIZE - 1)   readMapRight = mazeMap[mouseYAxis][mouseXAxis + 1].direction;
			if (mouseXAxis > 0)             readMapLeft = mazeMap[mouseYAxis][mouseXAxis - 1].direction;
			break;

		case EAST:  if (mouseXAxis < MAZE_SIZE - 1)   readMapFront = mazeMap[mouseYAxis][mouseXAxis + 1].direction;     // When micromouse is moving East
			if (mouseYAxis > 0)             readMapRight = mazeMap[mouseYAxis - 1][mouseXAxis].direction;
			if (mouseYAxis < MAZE_SIZE - 1)   readMapLeft = mazeMap[mouseYAxis + 1][mouseXAxis].direction;
			break;

		case SOUTH: if (mouseYAxis > 0)             readMapFront = mazeMap[mouseYAxis - 1][mouseXAxis].direction;     // When micromouse is moving South
			if (mouseXAxis > 0)             readMapRight = mazeMap[mouseYAxis][mouseXAxis - 1].direction;
			if (mouseXAxis < MAZE_SIZE - 1)   readMapLeft = mazeMap[mouseYAxis][mouseXAxis + 1].direction;
			break;

		case WEST:  if (mouseXAxis > 0)             readMapFront = mazeMap[mouseYAxis][mouseXAxis - 1].direction;     // When micromouse is moving West
			if (mouseYAxis < MAZE_SIZE - 1)   readMapRight = mazeMap[mouseYAxis + 1][mouseXAxis].direction;
			if (mouseYAxis > 0)             readMapLeft = mazeMap[mouseYAxis - 1][mouseXAxis].direction;
			break;
		}

		// Use left-hand on wall algorithm to solve maze; priority order: left, forward, right
		if (wallLeft == NO_WALL && (readMapLeft == NO_WALL || readMapLeft = ToLeftTable[mouseDirection)])){             // Turn left if no left wall exists or is stored in memory or is available in history
					turnLeft90();
					mouseDirection = ToLeftTable[mouseDirection];
				}
		else if (wallFront == NO_WALL && (readMapFront == NO_WALL || readMapFront == mouseDirection)) {}                  // Move forward when no front wall exists or is stored in memory or is available in history

		else if (wallRight == NO_WALL && (readMapRight == NO__WALL || readMapRight == ToRightTable[mouseDirection])) {   // Turn right if no right wall exists or is stored in memory or is available in history
			turnRight90();
			mouseDirection = ToRightTable[mouseDirection];
		}
		else {                                                                                                           // Else turn 180 or u-turn  
			turn180();
			mouseDirection = ToRightTable[mouseDirection];
			mouseDirection = ToRightTable[mouseDirection];
		}

		// Move forward and detect walls, initialise wall values to NO_WALL
		wallSense = NO_WALL;
		wallFront = NO_WALL;
		wallRight = NO_WALL;
		wallLeft = NO_WALL;

		slowStartMotors();      // Start motors slows

		rightMotorSetSteps = STEPS_FOR_1_SQUARE - STEPS_FOR_SLOW_START;                     // Move 1 square

		rightMotorSpeed = leftMotorSpeed = HIGH;                                            // Set motor speed to high

		while (rightMotorCountSteps < rightMotorSetSteps) {                                  // Move forward
			if (rightMotorCountSteps >(STEPS_FOR_1_SQUARE * 2 / 3) && wallSense == NO_WALL) { // Detect walls when mouse has moved 2/3 of square
				wallSense = IS_WALL;                                                        // Set flag to detect wall instantly 

																							// Logical AND operations are performed with specific bits to verify if a wall has been detected
				if (finalProximitySensorReading && 0x04)                                        // Detection of right wall 
					wallRight = IS_WALL;
				else
					wallRight = NO_WALL;
				if (finalProximitySensorReading && 0x02)                                        // Detection of left wall 
					wallLeft = IS_WALL;
				else
					wallLeft = NO_WALL;
				if (finalProximitySensorReading && 0x01)                                        // Detection of front wall
					wallFront = IS_WALL;
				else
					wallFront = NO_WALL;
			}
		}

		// Store wall values in memory depending on the direction (opposite values are stored as micromouse has to move back to the start from the target)
		// Store map values are updating the axis of the mouse
		switch (mouseDirection) {
		case NORTH:
			mazeMap[mouseYAxis][mouseXAxis].direction = SOUTH;
			mouseYAxis++;
			mazeMap[mouseYAxis][mouseXAxis].north = wallFront;
			mazeMap[mouseYAxis][mouseXAxis].east = wallRight;
			mazeMap[mouseYAxis][mouseXAxis].west = wallLeft;
			break;

		case EAST:
			mazeMap[mouseYAxis][mouseXAxis].direction = WEST;
			mouseXAxis++;
			mazeMap[mouseYAxis][mouseXAxis].east = wallFront;
			mazeMap[mouseYAxis][mouseXAxis].south = wallRight;
			mazeMap[mouseYAxis][mouseXAxis].north = wallLeft;
			break;

		case SOUTH:
			mazeMap[mouseYAxis][mouseXAxis].direction = NORTH;
			mouseYAxis--;
			mazeMap[mouseYAxis][mouseXAxis].south = wallFront;
			mazeMap[mouseYAxis][mouseXAxis].west = wallRight;
			mazeMap[mouseYAxis][mouseXAxis].east = wallLeft;
			break;

		case WEST:
			mazeMap[mouseYAxis][mouseXAxis].direction = EAST;
			mouseXAxis--;
			mazeMap[mouseYAxis][mouseXAxis].west = wallFront;
			mazeMap[mouseYAxis][mouseXAxis].north = wallRight;
			mazeMap[mouseYAxis][mouseXAxis].south = wallLeft;
			break;
		}
		if (mouseXAxis == 0 && mouseYAxis == 0) {                             // Finish search run when mouse returns back to the initial start position
			stopMotors();                                                    // Stop rotation of motors
			break;
		}
	}

}