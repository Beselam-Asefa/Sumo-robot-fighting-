
#include <motor_turn.h>
#include "Ultra.h"
#include "LSM303D.h"
#include "Accel_magnet.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Motor.h"
#include "Reflectance.h"
#include "IR.h"
#include "Beep.h"
#include "mqtt_sender.h"
#include <time.h>
#include <sys/time.h>
#include "serial1.h"
#include <zumo_config.h>



// sumo fight
Void zmain(void)
{

    Ultra_Start(); // start ultrasonic sensor
    uint8 button;
    button = SW1_Read(); // user button
    IR_Start();          //  start the infrared sensor
    IR_flush(); // clear IR r
    struct sensors_ ref;     // variable for reference value of the reference sensor
    struct sensors_ dig;     // variable for the digital value of the reference value
    int make_turn=0,after_hit_counter=0,sum=0;
    float time_differ; // variable for calculating the total time stamp of the competition
    reflectance_start();  // start the reflectance
    motor_start();       // start the motor
    bool ready_line=false;
    TickType_t start, end=0,hit_timer; // get timer
    struct accData_ data;  // variable for getting data from the accelerometer sensor
    reflectance_set_threshold(21000, 21000, 21000, 21000, 21000, 21000);  // reflectance sensor  threshold

    if(!LSM303D_Start())
    {
        printf("LSM303D failed to initialize!!! Program is Ending!!!\n");
        vTaskSuspend(NULL);
    }
    else
    {
        printf("Device Ok...\n");
    }

    while(SW1_Read() == 1 )
    {
        // wait here until the user button is pressed
        if(SW1_Read() == 0 )
        {
            break;
        }
    }

    while(SW1_Read() == 0)
    {
        // when the user button is pressed it fulfill the condition for the next loop
        ready_line=true;
    }

    while(ready_line)
    {
        reflectance_digital(&dig);
        sum = dig.l3 + dig.l2 + dig.l1 + dig.r1 + dig.r2 + dig.r3;

        if(sum<6)
        {
            //move until  detect the black line
            motor_forward(50,50);
        }

        if(sum==6)
        {
            // when detecting the line stop and wait for the IR signal
            motor_forward(0,0);
            print_mqtt("Zumo038/ready ","zumo");
            IR_wait();
            start = xTaskGetTickCount();
            print_mqtt("Zumo038/start ","%.2d ",start);
            motor_forward(200,700);
            break;
        }
    }


    for(;;)
    {

        int d = Ultra_GetDistance();
        reflectance_digital(&dig);
        sum = dig.l3 + dig.l2 + dig.l1 + dig.r1 + dig.r2 + dig.r3;

        while(sum==0 && d>30)
        {

            int d = Ultra_GetDistance();  // get the distance from another object
            reflectance_set_threshold(9000, 9000, 9000, 9000, 9000, 9000);  //  make threshold more sensitive to change
            reflectance_digital(&dig);  // get value from the reflectance sensors
            sum = dig.l3 + dig.l2 + dig.l1 + dig.r1 + dig.r2 + dig.r3;  // this add the values from the reflectance sensors
            LSM303D_Read_Acc(&data); // get data from the accelerometer

            if(sum==0 && d>30 && make_turn<30)
            {
                //this condition allow robot to run on the white field when there
                // is no robot in the 30cm range
                // when the make_turn counter is greater than the given value it takes the robot to the turn function and make to to scan
                // for opponent robot by rotating at a certain place
                motor_forward(255,20);
                make_turn++;
                after_hit_counter++;
            }

            if((data.accX>13000 || data.accY>13000) && after_hit_counter>10)
            {
                // this condition make the robot to detect a hit in positive X and Y direction which is above the given threshold
                // the variable after_hit_counter reduce multiple reading of accelerometer value during a single hit
                // it also remove unwanted accelerometer reading when the robot turn direction
                after_hit_counter=0;
                if(data.accX>13000 )
                {
                    hit_timer = xTaskGetTickCount();
                    print_mqtt("Zumo038/hit ","%.2d 0",hit_timer);
                }

                if(data.accY>13000 )
                {
                    hit_timer = xTaskGetTickCount();
                    print_mqtt("Zumo038/hit ","%.2d 270",hit_timer);
                }
            }

            if((data.accX< -10000 || data.accY < -10000) && after_hit_counter>10)
            {
                // this condition make the robot to detect a hit in negative X and Y direction which is above the given threshold
                // the variable after_hit_counter reduce multiple reading of accelerometer value during a single hit
                // it also remove unwanted accelerometer reading when the robot turn direction

                after_hit_counter=0;

                if(data.accX< -10000 )
                {
                    hit_timer = xTaskGetTickCount();
                    print_mqtt("Zumo038/hit ","%.2d 180",hit_timer);
                }

                if(data.accY <-10000 )
                {
                    hit_timer = xTaskGetTickCount();
                    print_mqtt("Zumo038/hit ","%.2d 90",hit_timer);
                }
            }

            if(sum>0 || make_turn >= 30)
            {
                //this condition make the robot to turn when the reflectance sensor detects the black line and the sum of
                // the reflectance value is greater than 0 or when the make_turn variable is greater than or equal to the given value

                //this loop allow the robot to move back until the sensor is out the black line
                while(true)
                {
                    reflectance_digital(&dig);
                    sum = dig.l3 + dig.l2 + dig.l1 + dig.r1 + dig.r2 + dig.r3;

                    if(sum>0)
                    {
                        motor_backward(255,150);

                    }
                    else
                    {
                        motor_tank_right(255,200);
                        break;
                    }
                }
                after_hit_counter=0;
                make_turn=0;
                break;
            }

            if(d<30)
            {
                // whenever the robot detect the opponent robot in the 30cm range
                // it breaks out from this loop and move to the next loop to be on attack mood
                break;
            }

            if(SW1_Read() == 0 && end==0)
            {
                // whenever the user button on the robot is pressed the robot calculate the time and
                // stop the motors
                end = xTaskGetTickCount(); // get ending time  time
                time_differ=(end-start)/1000;  // calculate the time
                print_mqtt("Zumo038/stop ","%.2d ",end); // print the ending time stamp
                print_mqtt("Zumo038/time ","%.2f ",time_differ); // print the total time since the robot start the competition
                motor_stop();
                break;
            }

        }

        while(sum==0 && d<30)
        {
            // this loop will run when the robot is inside the filed and if it detect another opponent robot
            // with in the 30cm range

            int d = Ultra_GetDistance();  // get the distance from another object
            reflectance_digital(&dig);    // get value from the reflectance sensors
            reflectance_set_threshold(21000, 21000, 21000, 21000, 21000, 21000);  // make threshold less sensitive to change
            sum = dig.l3 + dig.l2 + dig.l1 + dig.r1 + dig.r2 + dig.r3;  // this add the values from the reflectance sensors
            LSM303D_Read_Acc(&data);      // get data from the accelerometer

            if(sum==0 && d<30)
            {
                // in this condition the robot will be in attack mood and push the opponent robot
                // until the black line
                motor_forward(255,30);
                after_hit_counter++;
            }

            if (sum>0)
            {
                // when the robot detect the black line it move backward and turn direction
                motor_backward(255,400);
                motor_tank_right(255,350);
                after_hit_counter=0;
                break;
            }
            if((data.accX>13000 || data.accY>13000) && after_hit_counter>10)
            {
                // this condition make the robot to detect a hit in positive X and Y direction which is above the given threshold
                // the variable after_hit_counter reduce multiple reading of accelerometer value during a single hit
                // it also remove unwanted accelerometer reading when the robot turn direction
                after_hit_counter=0;
                if(data.accX>13000 )
                {
                    hit_timer = xTaskGetTickCount();
                    print_mqtt("Zumo038/hit ","%.2d 0",hit_timer);
                }

                if(data.accY>13000 )
                {
                    hit_timer = xTaskGetTickCount();
                    print_mqtt("Zumo038/hit ","%.2d 270",hit_timer);
                }
            }

            if((data.accX< -10000 || data.accY < -10000) && after_hit_counter>10)
            {
                // this condition make the robot to detect a hit in negative X and Y direction which is above the given threshold
                // the variable after_hit_counter reduce multiple reading of accelerometer value during a single hit
                // it also remove unwanted accelerometer reading when the robot turn direction

                after_hit_counter=0;

                if(data.accX< -10000 )
                {
                    hit_timer = xTaskGetTickCount();
                    print_mqtt("Zumo038/hit ","%.2d 180",hit_timer);
                }

                if(data.accY <-10000 )
                {
                    hit_timer = xTaskGetTickCount();
                    print_mqtt("Zumo038/hit ","%.2d 90",hit_timer);
                }
            }

            if(SW1_Read() == 0 && end==0)
            {
                // whenever the user button on the robot is pressed the robot calculate the time and
                // stop the motors
                end = xTaskGetTickCount(); // get ending time  time
                time_differ=(end-start)/1000;  // calculate the time
                print_mqtt("Zumo038/stop ","%.2d ",end); // print the ending time stamp
                print_mqtt("Zumo038/time ","%.2f ",time_differ); // print the total time since the robot start the competition
                motor_stop();
                break;
            }


        }
    }

}
