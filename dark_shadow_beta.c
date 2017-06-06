/*
 * Author:        rohith.raju002@stud.fh-dortmund.de
 * Purpose:       Target Detection with Dorobo32 board.
 * Language:      C
 * Created Date:  10 April 2017
 * Last Modified: 2 May 2017
 */
#include <stdlib.h>
#include <signal.h>

#include "dorobo32.h"
#include "FreeRTOS.h"
#include "task.h"
#include "trace.h"
#include "adc.h"
#include "motor.h"
#include "fft.h"
#include "digital.h"
#include "queue.h"
/* Global Variables */
QueueHandle_t xQueue;
int switch1_value = 1; // Switch on the Right to detect Right Collision
int switch2_value = 1; // Switch on the Left to detect Left Collision
int fftSensor_value = 0; // FFT Sensor on Right to detect Target
int fftSensor1_value = 0;// FFT Sensor on Left to detect Target
int sharpSensor1_value = 3400;//Distance sensor on Right to detect Right Obstacle
int sharpSensor2_value = 3400;//Distance sensor on Left to detect Left Obstacle

static void sharpSensor();
static void sharpSensor1();
static void fftSensor(void *pvParameters);
static void switchSensor(void *pvParameters);
static void navigator(void *pvParameters);
void rightRotate(int speed);
void leftRotate(int speed);
void moveForward(int speed);
void moveBackward(int speed);
void shiftRight(int speed);
void shiftLeft(int speed);
void sensorOutput(int outputType);
/*
*	 Description:
*  Creates FreeRTOS tasks for concurrent processing.
*  Calls the scheduler.
*	 Returns:   integer   				Parameters: void
*/
int main() {
	dorobo_init();//initializing dorobo32
	xQueue = xQueueCreate(5, sizeof(int));

	xTaskCreate(navigator, "navigator", configMINIMAL_STACK_SIZE, NULL, 1,
			NULL);
	xTaskCreate(fftSensor, "fftSensor", configMINIMAL_STACK_SIZE, NULL, 1,
			NULL);
	xTaskCreate(sharpSensor, "sharpSensor", configMINIMAL_STACK_SIZE, NULL, 1,
			NULL);
	xTaskCreate(sharpSensor1, "sharpSensor1", configMINIMAL_STACK_SIZE, NULL, 1,
			NULL);

	xTaskCreate(switchSensor, "switchSensor", configMINIMAL_STACK_SIZE, NULL, 1,
			NULL);
	vTaskStartScheduler();//starting task scheduling

	return 0;
}
/*
*	 Description:
*  Static method that navigates the robot to the target with help of three motors.
*  Detection of obstacles using values from Distance, FFT, Collision sensors.
*	 Returns:   void   				Parameters: pointer
*  Manouvres: Arslan Manouvre-
*							For detection of target, the robot makes one rotation at a fixed point.
*							Collision Escape Manouvre-
*							In case of single side collision(Right/Left), robot rotates 90 degress in the direction
*							opposite to collision.Then moves forward about 500mm.
*							In case of head on collision(Front), robot moves backward about 500mm and rotates
* 						Right/Left depending on Distance sensor valus.
*							Collision Avoidance Manouvre-
*							In case of Right/Left Distance sensor values above threshold(3300),the robot moves diogonally
*							Left/Right respectively.
*/
static void navigator(void *pvParameters) {
		//Thread timier definitions
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 20;
	xLastWakeTime = xTaskGetTickCount();
	//Condition check counters to avoid unnecessary delays
	int mode = 100;
	int sharpSensor1_value_count = 0;
	int sharpSensor2_value_count = 0;
	int targetIncognitoCount = 0;
	int targetFoundCount = 0;
	int switch1On = 0;
	int switch2On = 0;
	int switch1n2On = 0;
	trace_init();//initializing logging
	motor_init();//initializing motors
	vTaskDelayUntil(&xLastWakeTime, xFrequency);
	TickType_t startTime = 0;
	TickType_t rotateEndTime = 0;

	//Navigation to Target initiated
		while (1) {
			//No target detected
			if (!(((fftSensor_value + fftSensor1_value) / 2) > 1400)) {

				startTime = xTaskGetTickCount();
				targetIncognitoCount = 0;
				do {
					sensorOutput(1);
					if (targetIncognitoCount == 0) {
						//vTaskDelay(500);
					}
					targetIncognitoCount = 1;
					rightRotate(20);//10.86seconds for on full rotation @ 20 speed.
					rotateEndTime = xTaskGetTickCount();
					//Detecting Target
					if (((fftSensor_value + fftSensor1_value) / 2) > 1400) {
						break;//detected
					}

	//Checking upperbound of Arslan Manouvre
					if ((rotateEndTime - startTime) > 4532) {
						//Changing Location-Avoiding nearby barrier-right side barrier
						if (sharpSensor1_value > sharpSensor2_value) {
							targetIncognitoCount = 0;
							//resetting Arslan manouvre upper time threshold values
							startTime = xTaskGetTickCount();
							rotateEndTime = 0;

							vTaskDelay(104);
							leftRotate(20);//17.31seconds for on full rotation @ 15 speed.
							vTaskDelay(560);
							//Detecting Target
							if (((fftSensor_value + fftSensor1_value) / 2) > 1400) {
								break; //detected
							}

							moveForward(100);//840mm at 3.69seconds @50 speed
							vTaskDelay(1648);
							//Detecting Target
							if (((fftSensor_value + fftSensor1_value) / 2) > 1400) {
								break;//detected
							}
							continue;//making Arslan manouvre again
						}
						//Changing Location-Avoiding nearby barrier-left side barrier
						if (sharpSensor2_value > sharpSensor1_value) {
							//resetting Arslan manouvre upper time threshold values
							startTime = xTaskGetTickCount();
							rotateEndTime = 0;
							vTaskDelay(104);
							rightRotate(20);
							vTaskDelay(560);
							//Detecting Target
							if (((fftSensor_value + fftSensor1_value) / 2) > 1400) {
								break; //detected
							}

							moveForward(100);

							vTaskDelay(1648);
							//Detecting Target
							if (((fftSensor_value + fftSensor1_value) / 2) > 1400) {
								break;//detected
							}
							continue;//making Arslan manouvre again
						}
						//Changing Location
						if (sharpSensor2_value == sharpSensor1_value) {
							//resetting Arslan manouvre upper time threshold values
							startTime = xTaskGetTickCount();
							rotateEndTime = 0;
							vTaskDelay(104);
							rightRotate(20);
							vTaskDelay(560);
							//Detecting Target
							if (((fftSensor_value + fftSensor1_value) / 2) > 1400) {
								break; //detected
							}

							moveForward(100);

							vTaskDelay(1648);
							//Detecting Target
							if (((fftSensor_value + fftSensor1_value) / 2) > 1400) {
								break;//detected
							}
							continue;//making Arslan manouvre again
						}
					}
					//Obstacle on Right
					sharpSensor1_value_count = 0;
					if (sharpSensor1_value > 3000) {
						//resetting Arslan manouvre upper time threshold values
						startTime = xTaskGetTickCount();
						rotateEndTime = 0;
						do {

							if (sharpSensor1_value_count == 0) {

								vTaskDelay(104);
							}
							targetIncognitoCount = 0;
							sharpSensor1_value_count = 1;
							shiftLeft(30);//Left shift of approx. 300mm @ 30 speed within 3.7seconds
							vTaskDelay(206);
						} while (sharpSensor1_value > 3300);
						//Detecting Target
						if (((fftSensor_value + fftSensor1_value) / 2) > 1400) {
							break;//detected
						}
						continue;//making Arslan manouvre again

					}
					//Obstacle on Left
					sharpSensor2_value_count = 0;
					if (sharpSensor2_value > 3000) {
						//resetting Arslan manouvre upper time threshold values
						startTime = xTaskGetTickCount();
						rotateEndTime = 0;
						do {
							if (sharpSensor2_value_count == 0) {
								vTaskDelay(104);
							}
							targetIncognitoCount = 0;
							sharpSensor2_value_count = 1;
							shiftRight(30);
							vTaskDelay(206);
						} while (sharpSensor2_value > 3300);
						//Detecting Target
						if (((fftSensor_value + fftSensor1_value) / 2) > 1400) {
							break;//detected
						}
						continue;//making Arslan manouvre again
					}
					//Collision on Right
					switch1On = 0;
					if (switch1_value == 0) {
						//resetting Arslan manouvre upper time threshold values
						startTime = xTaskGetTickCount();
						rotateEndTime = 0;
						do {

							if (switch1On == 0) {
								vTaskDelay(104);
							}
							leftRotate(20);
							vTaskDelay(560);
							switch1On = 1;

						} while (switch1_value == 0);

						//Detecting Target
						if (((fftSensor_value + fftSensor1_value) / 2) > 1400) {
							break;//detected
						}

						moveForward(100);
						vTaskDelay(412);
						//Detecting Target
						if (((fftSensor_value + fftSensor1_value) / 2) > 1400) {
							break;//detected
						}
						continue;//making Arslan manouvre again
					}
					//Collision on the Left
					switch2On = 0;
					if (switch2_value == 0) {
						//resetting Arslan manouvre upper time threshold values
						startTime = xTaskGetTickCount();
						rotateEndTime = 0;
						do {

							if (switch2On == 0) {
								vTaskDelay(104);
							}
							rightRotate(20);
							vTaskDelay(560);
							switch2On = 1;

						} while (switch2_value == 0);

						vTaskDelay(560);
						//Detecting Target
						if (((fftSensor_value + fftSensor1_value) / 2) > 1400) {
							break;//detected
						}
						moveForward(100);
						vTaskDelay(412);
						//Detecting Target
						if (((fftSensor_value + fftSensor1_value) / 2) > 1400) {
							break;//detected
						}
						continue;//making Arslan manouvre again
					}
					//Head on collision
					switch1n2On = 0;
					if ((switch2_value == 0) && (switch1_value == 0)) {
						//resetting Arslan manouvre upper time threshold values
						startTime = xTaskGetTickCount();
						rotateEndTime = 0;
						do {

							if (switch1n2On == 0) {
								vTaskDelay(104);
							}

							moveBackward(100);
							vTaskDelay(412);
							switch1n2On = 1;
						} while ((switch2_value != 0) && (switch1_value != 0));

						//Not Right Corner Scenario.Nearer Obstacle at left
						if((sharpSensor2_value > sharpSensor1_value)&&(switch1_value == 1)){
							rightRotate(20);
							vTaskDelay(560);
						}
						//Not Left Corner Scenario.Nearer Obstacle at right
						if((sharpSensor1_value > sharpSensor2_value)&&(switch2_value == 1)){
							leftRotate(20);
							vTaskDelay(560);
						}
						//Right Corner Scenario.Nearer Obstacle at Right
						if((sharpSensor2_value > sharpSensor1_value)&&(switch1_value == 0)){
							leftRotate(20);
							vTaskDelay(560);
							moveForward(100);
							vTaskDelay(412);
						}
						//Left Corner Scenario.Nearer Obstacle at left
						if((sharpSensor1_value > sharpSensor2_value)&&(switch2_value == 0)){
							rightRotate(20);
							vTaskDelay(560);
							moveForward(100);
							vTaskDelay(412);
						}
						//Left Corner Scenario.Nearer Obstacle at left
						if((sharpSensor1_value == sharpSensor2_value)){
							leftRotate(20);
							vTaskDelay(560);
						}
						//Detecting Target
						if (((fftSensor_value + fftSensor1_value) / 2) > 1400) {
							break;//detected
						}
						continue;//making Arslan manouvre again
					}
					tracef("Time Period of Arslan Manouvre:  %d", (rotateEndTime - startTime));
				} while (!(((fftSensor_value + fftSensor1_value) / 2) > 1400));

			}
			//Target Detected
			targetFoundCount = 0;
			while (((fftSensor_value + fftSensor1_value) / 2) > 1400) {
				sensorOutput(2);
				startTime = xTaskGetTickCount();
				if (targetFoundCount == 0) {
					leftRotate(20);
					vTaskDelay(280);
				}
				targetFoundCount = 1;

				moveForward(100);

				rotateEndTime = xTaskGetTickCount();
				//Target at Line of Sight
				if (!(((fftSensor_value + fftSensor1_value) / 2) > 1400)) {
					vTaskDelay(206);
					break;//Arslan Manouvre Initiated
				}
				//Straight hit on at target wall
				if ((switch2_value == 0) && (switch1_value == 0)) {
					while (1) {
						vTaskDelay(102);
						moveForward(0);//stopping nebucadnezzar

					}
				}
				//Obstacle on Left
				sharpSensor1_value_count = 0;
				if (sharpSensor1_value > 3300) {
					do {

						if (sharpSensor1_value_count == 0) {

							vTaskDelay(103);
						}

						shiftLeft(30);
						vTaskDelay(206);
						targetFoundCount = 0;
						sharpSensor1_value_count = 1;
					} while (sharpSensor1_value > 3300);
					//Target at Line of Sight
					if (!(((fftSensor_value + fftSensor1_value) / 2) > 1400)) {
						vTaskDelay(104);
						break;//Arslan Manouvre Initiated
					}
					vTaskDelay(104);
					continue;//Continuing straight navigation to Target

				}
				//Obstacle on Left
				sharpSensor2_value_count = 0;
				if (sharpSensor2_value > 3300) {
					do {
						if (sharpSensor2_value_count == 0) {
							vTaskDelay(104);
						}
						shiftRight(30);
						vTaskDelay(206);

						targetFoundCount = 0;
						sharpSensor2_value_count = 1;

					} while (sharpSensor2_value > 3300);
					//Target at Line of Sight
					if (!(((fftSensor_value + fftSensor1_value) / 2) > 1400)) {
						vTaskDelay(104);
						break;//Arslan Manouvre Initiated
					}
					vTaskDelay(104);
					continue;//Continuing straight navigation to Target
				}
				//Collision on the Right
				switch1On = 0;
				if (switch1_value == 0) {
					do {

						if (switch1On == 0) {
							vTaskDelay(104);
						}
						leftRotate(20);
						vTaskDelay(560);
						switch1On = 1;

					} while (switch1_value == 0);

					moveForward(100);
					vTaskDelay(412);
					//Target at Line of Sight
					if (!(((fftSensor_value + fftSensor1_value) / 2) > 1400)) {
						break;//Arslan Manouvre Initiated
					}


					continue;//Continuing straight navigation to Target
				}
				//Collision on the Left
				switch2On = 0;
				if (switch2_value == 0) {
					do {

						if (switch2On == 0) {
							vTaskDelay(104);
						}
						rightRotate(20);
						vTaskDelay(560);
						switch2On = 1;

					} while (switch2_value == 0);



					moveForward(100);
					vTaskDelay(412);
					//Target at Line of Sight
					if (!(((fftSensor_value + fftSensor1_value) / 2) > 1400)) {
						break;//Arslan Manouvre Initiated
					}
					continue;//Continuing straight navigation to Target
				}
			}
			vTaskDelay(104);
		}
}
/*
*	 Description:
*  Static method that loads FFT sensor values to the global variables from two FFT sensors .
*  DD_PIN_PD14 for the right FFT sensor.
*  DD_PIN_PD15 for the left FFT sensor.
*	 Returns:   void   				Parameters: pointer
*/
static void fftSensor(void *pvParameters) {
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 20;
	xLastWakeTime = xTaskGetTickCount();
	trace_init();
	ft_init();//initializing fast fourier transform
	vTaskDelayUntil(&xLastWakeTime, xFrequency);
	while (1) {
		traces("\n\nScanning Signal...");
		ft_start_sampling(DD_PIN_PD14);//starting sampling from the FFT sensor right.
		while (!(ft_is_sampling_finished())) {
			vTaskDelay(1);
		}
		int sensorR = ft_get_transform(DFT_FREQ125);// getting the sampled output.
		fftSensor_value = sensorR;

		ft_start_sampling(DD_PIN_PD15);//starting sampling from the FFT sensor left.
		while (!(ft_is_sampling_finished())) {
			vTaskDelay(1);
		}
		int sensorL = ft_get_transform(DFT_FREQ125);// getting the sampled output.
		fftSensor1_value = sensorL;
		vTaskDelay(100);

	}
}
/*
*	 Description:
*  Static method that loads switch sensor values to the global variables from two switch sensors .
*	 These values help in the collision escape manouvre.
*  DD_PIN_PC8 for the right Switch sensor.
*  DD_PIN_PC9 for the left Switch sensor.
*	 Returns:   void   				Parameters: pointer
*/
static void switchSensor(void *pvParameters) {
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 20;
	xLastWakeTime = xTaskGetTickCount();
	trace_init();
	digital_init();//initializing digital pins
	vTaskDelayUntil(&xLastWakeTime, xFrequency);
	while (1) {
		traces("Detecting Collision...");
		int switch1 = digital_get_pin(DD_PIN_PC8);//loading right switch sensor values.
		switch1_value = switch1;
		int switch2 = digital_get_pin(DD_PIN_PC9);//loading right switch sensor values.
		switch2_value = switch2;
		vTaskDelay(100);
	}
}
/*
*	 Description:
*  Static method that loads distance sensor value to the global variable from right distance sensor.
*	 These values help in the collision avoidance manouvre.
*  DA_ADC_CHANNEL0 for the right distance sensor.
*  Returns:   void   				Parameters: pointer
*/
static void sharpSensor(void *pvParameters) {
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 20;
	xLastWakeTime = xTaskGetTickCount();
	adc_init();//intializing analog to digital converter.
	vTaskDelayUntil(&xLastWakeTime, xFrequency);
	while (1) {
		int sharp1 = adc_get_value(DA_ADC_CHANNEL0);//loading values from the right distance sensor.
		sharpSensor1_value = sharp1;
		vTaskDelay(100);

	}

}
/*
*	 Description:
*  Static method that loads distance sensor value to the global variable from left distance sensor.
*	 These values help in the collision avoidance manouvre.
*  DA_ADC_CHANNEL1 for the left distance sensor.
*  Returns:   void   				Parameters: pointer
*/
static void sharpSensor1(void *pvParameters) {
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 20;
	xLastWakeTime = xTaskGetTickCount();
	adc_init();//intializing analog to digital converter.
	vTaskDelayUntil(&xLastWakeTime, xFrequency);
	while (1) {
		int sharp2 = adc_get_value(DA_ADC_CHANNEL1);//loading values from the right distance sensor.
		sharpSensor2_value = sharp2;
		vTaskDelay(100);

	}

}
/*
*	 Description:
*  Method that rotates the robot to the right direction at a fixed point.
*	 MOTOR-0 is the back motor.
*  MOTOR-1 is the right motor.
*	 MOTOR-2 is the left motor.
*  Returns:   void   				Parameters: integer;the speed of motor rotation.
*/
void rightRotate(int speed) {
	motor_set(DM_MOTOR0, speed);
	motor_set(DM_MOTOR1, speed);
	motor_set(DM_MOTOR2, speed);
}
/*
*	 Description:
*  Method that rotates the robot to the left direction at a fixed point.
*	 MOTOR-0 is the back motor.
*  MOTOR-1 is the right motor.
*	 MOTOR-2 is the left motor.
*  Returns:   void   				Parameters: integer;the speed of motor rotation.
*/
void leftRotate(int speed) {

	motor_set(DM_MOTOR0, (-1 * speed));
	motor_set(DM_MOTOR1, (-1 * speed));
	motor_set(DM_MOTOR2, (-1 * speed));
}
/*
*	 Description:
*  Method that moves the robot to the forward direction.
*	 MOTOR-0 is the back motor.
*  MOTOR-1 is the right motor.
*	 MOTOR-2 is the left motor.
*  Returns:   void   				Parameters: integer;the speed of motor rotation.
*/
void moveForward(int speed) {
	motor_set(DM_MOTOR0, 0);
	motor_set(DM_MOTOR1, (-1 * speed));
	motor_set(DM_MOTOR2, speed);
}
/*
*	 Description:
*  Method that moves the robot to the backward direction.
*	 MOTOR-0 is the back motor.
*  MOTOR-1 is the right motor.
*	 MOTOR-2 is the left motor.
*  Returns:   void   				Parameters: integer;the speed of motor rotation.
*/
void moveBackward(int speed) {
	motor_set(DM_MOTOR0, 0);
	motor_set(DM_MOTOR1, speed);
	motor_set(DM_MOTOR2, (-1 * speed));
}
/*
*	 Description:
*  Method that moves the robot to the diogonal left direction.
*	 MOTOR-0 is the back motor.
*  MOTOR-1 is the right motor.
*	 MOTOR-2 is the left motor.
*  Returns:   void   				Parameters: integer;the speed of motor rotation.
*/
void shiftLeft(int speed) {
	motor_set(DM_MOTOR0, speed);
	motor_set(DM_MOTOR1, (-1 * speed));
	motor_set(DM_MOTOR2, 0);
}
/*
*	 Description:
*  Method that moves the robot to the diogonal right direction.
*	 MOTOR-0 is the back motor.
*  MOTOR-1 is the right motor.
*	 MOTOR-2 is the left motor.
*  Returns:   void   				Parameters: integer;the speed of motor rotation.
*/
void shiftRight(int speed) {
	motor_set(DM_MOTOR0, (-1 * speed));
	motor_set(DM_MOTOR1, 0);
	motor_set(DM_MOTOR2, speed);
}
/*
*	 Description:
*  Method that does logging ofsensor values.
*  Returns:   void   				Parameters: integer;Arslan Manouvre Log(1) or Target Pursuit Log(2).
*/
void sensorOutput(int outputType) {
	if (outputType == 1) {
		tracef(
				"Arslan Manouvre Initiated....\n Acquiring Sensor values...\n FFT-RIGHT %d \n FFT-LEFT %d \n SHARP-1 %d \n SHARP-2 %d \n",
				fftSensor_value, fftSensor1_value, sharpSensor1_value,
				sharpSensor2_value);
	}
	if (outputType == 2) {
		tracef(
				" In pursuit of Target....\n Acquiring Sensor values...\n FFT-RIGHT %d \n FFT-LEFT %d \n SHARP-1 %d \n SHARP-2 %d \n",
				fftSensor_value, fftSensor1_value, sharpSensor1_value,
				sharpSensor2_value);
	}
}
