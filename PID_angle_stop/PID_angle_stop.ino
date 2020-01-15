//---------------------LIBRARIES---------------------------// 
/*
  Download the ESC library here: https://github.com/RB-ENantel/RC_ESC/ 
*/ 
#include "ESC.h"
//---------------------ESC SETTINGS-----------------------// 
#define SPEED_MIN (1000)                                  // Set the Minimum Speed Setting of the ESC
#define SPEED_MAX (2000)                                  // Set the Maximum Speed Setting of the ESC

ESC myESC (9, SPEED_MIN, SPEED_MAX, 500);                 // Initialize ESC (ESC PIN, Minimum Value, Maximum Value, Default Speed)
int oESC;                                                 // Variable for the speed sent to the ESC

/*
  The variables and ports defined below are to guide you when writing the code, but feel free to change or add/delete as nessecary. 
*/ 


//---------------------ENCODER PORTS----------------------// 
int val;
#define outputA 6                                         // Define Encoder output A to Digital Pin 6 on Arduino Uno 
#define outputB 7                                         // Define Encoder output B to Digital Pin 7 on Arduino Uno
#define NUMPULSES 1200  
int counter = 0;                                       // Define the number of pulses on Encoder resolution (enter twice the amount of pulses on datasheet)
int encoder0PinALast = LOW;                               // Past state of the encoder output
int n = LOW;
                                  

//---------------------TIMER VARIABLES---------------------//

long t_now = 0;
long t_last_print = 0;
long t_last_PID = 0;
int T_sample = 15;                                        // sample time in milliseconds (ms)
int T_print = 1000;                                       // sample print to monitor time in milliseconds (ms)


//---------------------PID VARIABLES----------------------//
// Define variables for the output of the encoder(sensed_output), angle output of the encoder(sensed_output converted to angle), and error w.r.t the setpoint

double sensed_output, error, sensed_angle;                




// Define variables for total error over time, previous error in the last sampling time interval, control signal, and limitations to the controller
double setpoint_angle =30; 
double setpoint = setpoint_angle * NUMPULSES/360;
long total_error =0;
double last_error=0;
long control_signal; 
int max_control = 1700;
int min_control = 1000; 




// ==================INSERT DESIRED SETPOINT ANGLE HERE================== //

// ==================INSERT CONTROL GAINS HERE=========== 
double Kp = 5;                                          // proportional gain
double Ki = 0.0265;                                        // integral gain in [ms^-1]
double Kd = 1888;                                          // derivative gain in [ms]
double Kp2 =1.6;
double Ki2 = 0.005;
double Kd2 = 20;
double output_Kp = 0;
double output_Ki = 0;
double output_Kd = 0;
double output = 0;
double angle = 0.0;

void setup() {
  /* 
    Setup function to initialize serial plotter/monitor, initialize origin of encoder, 
    arm ESC driver, and to initialize propeller by ramping up and down
  */ 
  pinMode (outputA, INPUT);
  pinMode (outputB, INPUT);// Propeller speed to ramp up and down to check functionality
  Serial.begin(9600);
  encoder0PinALast = digitalRead(outputA);                        // Reads the initial state of the outputA 
  myESC.arm();                                             // Send the Arm value so the ESC will be ready to take commands
  Serial.println("Setting up....");
  delay(7500);                                              // Wait for a while
  Serial.println("Testing Moter Function...");
  //rampUpDown();      
  Serial.println("Begin Main Loop!");
  Serial.println("============================");
  Serial.println("Press Anything to begin");
}

/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////MAIN LOOP///////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
void loop() {
  /* 
    Main loop of the project. 
    1. Read Encoder
    2. Implement PID control
    3. Send control signal to motor
    4. Print sensed angle with respect to the setpoint
  */
  while(Serial.available() < 1)
  {
  }

// Read the encoder 
encoder_read();

// Take the magnitude of the encoder output (accounting for CW or CCW rotation)
sensed_output = abs(counter);

// Implement PID control
PID_Control();

//Print Control Signal

// Write control signal to motor
myESC.speed(control_signal);  

// Print sensed angle and setpoint angle
print_results();
}







void rampUpDown() {

  /* 
    Function written to test functionality of brushless motor by accelerating and decelerating the rotation 
    of the motor. Speed of the motor does not accelerate to rated speed. 
  */


  for (oESC = SPEED_MIN; oESC <= 1150; oESC += 1) {        // iterate from minimum speed to a speed setting of 1150
    myESC.speed(oESC);                                     // write speed setting to motor
    delay(10);                                             // waits 10ms for the ESC to reach speed
  }
  delay(2000);                                             // wait a while
  for (oESC = 1150; oESC >= SPEED_MIN; oESC -= 1) {        // iterate from speed setting of 1150 to minimum speed
    myESC.speed(oESC);                                     // write speed setting to motor
    delay(10);                                             // waits 10ms for the ESC to reach speed  
   }
  delay(7000);                                             // Wait for a while before going into control loop
}







void encoder_read() {
  /* 
    STUDENTS TO ANSWER
    Function to count the number of pulses from the encoder output. 
    Must count at least 2 count per pulse, which would give you 1200 count per rotation.  
    *****DO NOT USE INTERUPTS TO READ ENCODER, IT COULD INTERFERE WITH THE CONTROLLER CALCULATIONS, USE IT AT YOUR OWN RISK*****  
  */ 
  // ==================INSERT ENCODER ALGORITHM HERE ==================== //
 n = digitalRead(outputA);
  if (encoder0PinALast != n ) {
    if (digitalRead(outputB) != n) {
      counter = counter -1;
    } else {
      counter = counter +1;
    }
  }
  encoder0PinALast = n;

  // ==================INSERT ENCODER ALGORITHM HERE ==================== //
}







void PID_Control() {
    // ==================INSERT CONTROL ALGORITHM HERE ==================== //

  /* 
    STUDENTS TO ANSWER
    Function to implement PID control. 
    Input: encoder position
    Output: motor speed
    Steps:
      1. Determine amount of time that has passed
      2. Determine 'if statement' to implement the use of a sampling time to implement control signal
        (i.e. create if statement to represent sampling time and to calculate the control signal every interval)
      3. Calculate the current error.
      4. Calculate control output assuming all gains of the PID are defined; a min and max limit must be placed on the output ESC speed signal
        (HINT: use finite difference approx for derivative, use rectangular approximation method to estimate area 
        under curve for the integral term)
  */




  t_now = millis();                  // returns the number of milliseconds passed since the Arduino started running the program
  if (t_now - t_last_PID >= 20){      // if the elapsed time is greater than the sampling time, time to send control signal
    t_last_PID = t_now;
    // ==================INSERT CONTROL ALGORITHM HERE ==================== //

    sensed_output = abs(counter);
    sensed_angle = sensed_output / 1200 * 360;
    error = setpoint_angle - sensed_angle;
    total_error += (error + last_error) / 2 * T_sample; // integrate the lastest error into total error for Pi
    output_Kp = Kp * error;
    output_Ki = Ki * total_error;
    output_Kd = Kd * (error - last_error) / T_sample;
    output = output_Kp + output_Ki + output_Kd;
    control_signal = output;
    last_error = error;

  /*
    if (output <min_control){
      output = min_control;
      control_signal = output;
      }
    else if (output>max_control){
      output = max_control;
      control_signal = output;
      }
    else {
    control_signal = output;
    }
   */
  

    // ==================INSERT CONTROL ALGORITHM HERE ====================== //

  }  
}

void PID_Fine_Control() {

  t_now = millis();                  // returns the number of milliseconds passed since the Arduino started running the program
  if (t_now - t_last_PID >= 20){      // if the elapsed time is greater than the sampling time, time to send control signal
    t_last_PID = t_now;
    // ==================INSERT CONTROL ALGORITHM HERE ==================== //

    sensed_output = abs(counter);
    sensed_angle = sensed_output / 1200 * 360;
    error = setpoint_angle - sensed_angle;

    if (error>5){
    total_error += (error + last_error) / 2 * T_sample; // integrate the lastest error into total error for Pi
    output_Kp = Kp * error;
    output_Ki = Ki * total_error;
    output_Kd = Kd * (error - last_error) / T_sample;
    output = output_Kp + output_Ki + output_Kd;
    }
    else {
    total_error += (error + last_error) / 2 * T_sample; // integrate the lastest error into total error for Pi
    output_Kp = Kp2 * error;
    output_Ki = Ki2 * total_error;
    output_Kd = Kd2 * (error - last_error) / T_sample;
    output = output_Kp + output_Ki + output_Kd;
      }
    if (output <min_control){
      output = min_control;
      control_signal = output;
      }
    else if (output>max_control){
      output = max_control;
      control_signal = output;
      }
    else {
    control_signal = output;
    }
  }  
} 


void print_results() {
  /* 
    STUDENTS TO ANSWER
    Function to print the sensed output/angle to the setpoint every 50 ms. Use Serial plotter on Arduino to graphically plot the 
    sensed angle with respect to the setpoint angle. 

    HINT: You might want to print sensed_output to verify that the encoder is reading correctly
    HINT: You might also want to print control_signal to ensure the PID is working properly before writing it to the motor 
  */

  t_now = millis();
  if (t_now - t_last_print >= T_print){
    t_last_print = t_now;

    // ==================INSERT PRINT ALGORITHM HERE ==================== //
Serial.print("Desired Angle: ");
Serial.print(setpoint_angle);
Serial.print("//Current Angle: ");
Serial.print(sensed_angle);
Serial.print("//Current Input: ");
Serial.print(control_signal);
Serial.print("====Ki: ");
Serial.print(output_Ki);
Serial.print("+Kp: ");
Serial.print(output_Kp);
Serial.print("+Kd: ");
Serial.println(output_Kd);
    // ==================INSERT PRINT ALGORITHM HERE ==================== //
    }
}
