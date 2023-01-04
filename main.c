////Assignment 9 Orientation using Gyroscope
//// Prasad Pamu and Parth Patel
/* Code for execution of calibration only once, i have tried both ways please consider.*/

//#include <msp430fr6989.h>
//#include <i2c.h>
//#include <lcd.h>
//#include <uart.h>
//#include <math.h>
//#include <stdio.h>
//
//#define I2C_ADDRESS_MPU9150 0x68
//#define MEM_LEN 6000
//
//#pragma NOINIT()
//unsigned char FRAM_array[MEM_LEN] = { 0x00 };
//unsigned int gTick_0_5ms = 0;
//int res = -1;
//unsigned char buffer[10];
//unsigned int GyrobiasX[2];
//unsigned int GyrobiasY[2];
//unsigned int GyrobiasZ[2];
//unsigned int GyroBiasX;
//unsigned int GyroBiasY;
//unsigned int GyroBiasZ;
//unsigned int GyroBias[];
//float rawGyroX;
//float rawGyroY;
//float rawGyroZ;
//float GyroX;
//float GyroY;
//float GyroZ;
//float dt = 0;
//float Roll;
//float Yaw;
//float Pitch;
//unsigned char gbToggle = 1;
//unsigned char gbFirst = 0;
//unsigned int fram_index;
//unsigned int crc;
//unsigned int getCRC16(unsigned int GyroBias[], unsigned int len)
//{
//    unsigned int i;
//   CRCINIRES = 0xFFFF;
//   for(i=0;i<len;i++)
//       CRCDI = GyroBias[i];
// return CRCINIRES;
//}
//
//void main(void)
//{
//    WDTCTL = WDT_MDLY_0_5;
//
//    P1SEL0 &= ~(BIT0 + BIT1); // Bit 0 and 1 are cleared of port 1
//    P1SEL1 &= ~(BIT0 + BIT1);
//    P9SEL0 &= ~(BIT0 + BIT1); // Bit 0 and 1 are cleared of port 9
//    P9SEL1 &= ~(BIT0 + BIT1);
//    P1DIR |= BIT0; // setting bit 0 of port 1 acting as output
//    P1DIR &= ~BIT1; // clearing bit 1 of port 1 acting as input
//    P9DIR |= BIT7; // setting bit 7 of port 9 acting as output
//
//    P1OUT |= BIT1; // making a pullup resistor by setting P1out and P1ren register
//    P1REN |= BIT1;
//
//    P1IES |= BIT1; // falling edge at P1.1 generates interrupt
//    P1IE |= BIT1; // Interrupt enable for P1.1
//    PM5CTL0 &= ~LOCKLPM5;  // to activate I/O functions this needs to be cleared
//    SFRIE1 |= WDTIE; // WDT-interrupt enabled
//    UART_Init(SPEED_38400_SMCLK);
//    LCD_Init();     // Initializing LCD
//    I2C_Init(); // Initializing I2C
//    _EINT(); // global interrupt enable
//    TA0CTL = TASSEL__SMCLK | MC__CONTINUOUS;//Timer A: clock source:SMCLK, Continous Mode.
//
//    buffer[0] = 0x80; // this value needed to be written for the power management to reset the device
//
//    res = I2C_WriteBlock(I2C_ADDRESS_MPU9150, buffer, 1, 0x6B); // reseting the sensor hub
//
//    gTick_0_5ms = 40;      // wait for 20 ms
//    LPM0;
//    buffer[0] = 0x00;         // this value is needed to clear the sleep bit
//
//    res = I2C_WriteBlock(I2C_ADDRESS_MPU9150, buffer, 1, 0x6B); // clearing the sleep bit
//
//    gTick_0_5ms = 40;      // wait for 20 ms
//    LPM0;
//
//    GyrobiasX[0] = 0;
//    GyrobiasY[0] = 0;
//    GyrobiasZ[0] = 0;
//
//           Yaw = 0;
//    if ((getCRC16(&GyroBias[0])!=crc)))
//    calibrate();
//    while (1)// Angle calculation for yaw,roll and pitch, display according to button pressed
//       {
//        float Counter = 1;
//        res = I2C_ReadBlock(I2C_ADDRESS_MPU9150, buffer, 6, 0x43);
//
//           rawGyroX = (buffer[0] << 8) | buffer[1]; // combine the high byte first then low byte
//           rawGyroY = (buffer[2] << 8) | buffer[3]; // combine the high byte first then low byte
//           rawGyroZ = (buffer[4] << 8) | buffer[5]; // combine the high byte first then low byte
//
//           GyroX = (rawGyroX - GyroBiasX) / 131.0;// X value of gyro measurements
//           GyroY = (rawGyroY - GyroBiasY) / 131.0;// Y value of gyro measurements
//           GyroZ = (rawGyroZ - GyroBiasZ) / 131.0;// Z value of gyro measurements
//
//           dt = Counter - dt; // time since last measurement was taken
//
//           Pitch += -GyroX * dt;
//           Roll += -GyroY * dt;
//           Yaw += -GyroZ * dt;
//
//           gTick_0_5ms = 40;      // wait for 20 ms
//           LPM0;
//
//           if (gbToggle == 0)
//           {
//               LCD_displayShort((short) Pitch);
//               P1OUT |= BIT0;
//               P9OUT &= ~BIT7;
//           }
//
//           if (gbToggle == 1)
//           {
//               LCD_displayShort((short) Roll);
//               P1OUT &= ~BIT0;
//               P9OUT |= BIT7;
//           }
//
//           if (gbToggle == 2)
//           {
//               LCD_displayShort((short) Yaw);
//               P1OUT &= ~BIT0;
//               P9OUT &= ~BIT7;
//           }
//           Counter++;
//       }
//}
//void calibrate(void)
//{
//    while (1)// Calibration Just once and storing in FRAM so that its executed only once
//           {
//               res = I2C_ReadBlock(I2C_ADDRESS_MPU9150, buffer, 6, 0x43);
//
//               if (fram_index < MEM_LEN - 6)
//                   {
//                      FRAM_array[fram_index++] = buffer[0];
//                      FRAM_array[fram_index++] = buffer[1];
//                      FRAM_array[fram_index++] = buffer[2];
//                      FRAM_array[fram_index++] = buffer[3];
//                      FRAM_array[fram_index++] = buffer[4];
//                      FRAM_array[fram_index++] = buffer[5];
//
//                      gTick_0_5ms = 40;      // wait for 20 ms
//                      LPM0;
//                  }
//                   else
//                  {
//                      fram_index = 0;
//                      while (fram_index < MEM_LEN)
//                      {
//
//                          unsigned char HX = FRAM_array[fram_index++];
//                          unsigned char LX = FRAM_array[fram_index++];
//                          unsigned char HY = FRAM_array[fram_index++];
//                          unsigned char LY = FRAM_array[fram_index++];
//                          unsigned char HZ = FRAM_array[fram_index++];
//                          unsigned char LZ = FRAM_array[fram_index++];
//
//                          GyrobiasX[1] = (HX << 8) | LX; // combine the high byte first then low byte
//                          GyrobiasY[1] = (HY << 8) | LY; // combine the high byte first then low byte
//                          GyrobiasZ[1] = (HZ << 8) | LZ; // combine the high byte first then low byte
//
//                          GyrobiasX[0] = GyrobiasX[0] + GyrobiasX[1];
//                          GyrobiasY[0] = GyrobiasY[0] + GyrobiasY[1];
//                          GyrobiasZ[0] = GyrobiasZ[0] + GyrobiasZ[1];
//
//                      }
//
//                      GyroBias[0] = GyrobiasX[0] / 1000;
//                      GyroBias[1] = GyrobiasY[0] / 1000;
//                      GyroBias[2] = GyrobiasZ[0] / 1000;
//                      break;
//                  }
//               crc = getCRC16(&GyroBias[0]);
//              }
//              Yaw = 0;
//}
//#pragma vector=PORT1_VECTOR         // interrupt when button is pressed
//__interrupt void Port1(void)
//{
//    P1IFG &= ~BIT1; // clear P1.1 interrupt flag in the end
//        if (gbToggle >= 2)// when the buttons are pressed again and again
//        {
//            gbToggle = 0;
//        }
//        else
//        {
//            gbToggle = gbToggle + 1;
//        }
//}
//
//#pragma vector=WDT_VECTOR
//__interrupt void tick(void)
//{
//    if (gTick_0_5ms > 0)
//    {
//        gTick_0_5ms--;
//        if (gTick_0_5ms == 0)
//            LPM0_EXIT;
//    }
//}



/*I have tried  the normal way as well*/
#include <msp430fr6989.h>
#include <i2c.h>
#include <lcd.h>
#include <uart.h>
#include <math.h>
#include <stdio.h>

#define I2C_ADDRESS_MPU9150 0x68
#define MEM_LEN 6000

#pragma PERSISTENT(FRAM_array)
unsigned char FRAM_array[MEM_LEN] = { 0x00 };
unsigned int gTick_0_5ms = 0;
int res = -1;
unsigned char buffer[10];
float GyrobiasX[2];
float GyrobiasY[2];
float GyrobiasZ[2];
float GyroBiasX;
float GyroBiasY;
float GyroBiasZ;
float rawGyroX;
float rawGyroY;
float rawGyroZ;
float GyroX;
float GyroY;
float GyroZ;
float dt = 0;
float Roll;
float Yaw;
float Pitch;
unsigned char gbToggle = 1;
unsigned int fram_index;

void main(void)
{
    WDTCTL = WDT_MDLY_0_5;

    P1SEL0 &= ~(BIT0 + BIT1); // Bit 0 and 1 are cleared of port 1
    P1SEL1 &= ~(BIT0 + BIT1);
    P9SEL0 &= ~(BIT0 + BIT1); // Bit 0 and 1 are cleared of port 9
    P9SEL1 &= ~(BIT0 + BIT1);
    P1DIR |= BIT0; // setting bit 0 of port 1 acting as output
    P1DIR &= ~BIT1; // clearing bit 1 of port 1 acting as input
    P9DIR |= BIT7; // setting bit 7 of port 9 acting as output

    P1OUT |= BIT1; // making a pullup resistor by setting P1out and P1ren register
    P1REN |= BIT1;

    P1IES |= BIT1; // falling edge at P1.1 generates interrupt
    P1IE |= BIT1; // Interrupt enable for P1.1
    PM5CTL0 &= ~LOCKLPM5;  // to activate I/O functions this needs to be cleared
    SFRIE1 |= WDTIE; // WDT-interrupt enabled
    UART_Init(SPEED_38400_SMCLK);
    LCD_Init();     // Initializing LCD
    I2C_Init(); // Initializing I2C
    _EINT(); // global interrupt enable

    buffer[0] = 0x80; // this value needed to be written for the power management to reset the device

    res = I2C_WriteBlock(I2C_ADDRESS_MPU9150, buffer, 1, 0x6B); // reseting the sensor hub

    gTick_0_5ms = 40;      // wait for 20 ms
    LPM0;
    buffer[0] = 0x00;         // this value is needed to clear the sleep bit

    res = I2C_WriteBlock(I2C_ADDRESS_MPU9150, buffer, 1, 0x6B); // clearing the sleep bit

    gTick_0_5ms = 40;      // wait for 20 ms
    LPM0;

    GyrobiasX[0] = 0;
    GyrobiasY[0] = 0;
    GyrobiasZ[0] = 0;
    while (1)// Calibration Just once and storing in FRAM so that its executed only once
        {
            res = I2C_ReadBlock(I2C_ADDRESS_MPU9150, buffer, 6, 0x43);

            if (fram_index < MEM_LEN - 6)
                {
                   FRAM_array[fram_index++] = buffer[0];
                   FRAM_array[fram_index++] = buffer[1];
                   FRAM_array[fram_index++] = buffer[2];
                   FRAM_array[fram_index++] = buffer[3];
                   FRAM_array[fram_index++] = buffer[4];
                   FRAM_array[fram_index++] = buffer[5];

                   gTick_0_5ms = 40;      // wait for 20 ms
                   LPM0;
               }
                else
               {
                   fram_index = 0;
                   while (fram_index < MEM_LEN)
                   {

                       unsigned char HX = FRAM_array[fram_index++];
                       unsigned char LX = FRAM_array[fram_index++];
                       unsigned char HY = FRAM_array[fram_index++];
                       unsigned char LY = FRAM_array[fram_index++];
                       unsigned char HZ = FRAM_array[fram_index++];
                       unsigned char LZ = FRAM_array[fram_index++];

                       GyrobiasX[1] = (HX << 8) | LX; // combine the high byte first then low byte
                       GyrobiasY[1] = (HY << 8) | LY; // combine the high byte first then low byte
                       GyrobiasZ[1] = (HZ << 8) | LZ; // combine the high byte first then low byte

                       GyrobiasX[0] = GyrobiasX[0] + GyrobiasX[1];
                       GyrobiasY[0] = GyrobiasY[0] + GyrobiasY[1];
                       GyrobiasZ[0] = GyrobiasZ[0] + GyrobiasZ[1];

                   }

                   GyroBiasX = GyrobiasX[0] / 1000;
                   GyroBiasY = GyrobiasY[0] / 1000;
                   GyroBiasZ = GyrobiasZ[0] / 1000;
                   break;
               }
           }
           Yaw = 0;
    while (1)// Angle calculation for yaw,roll and pitch, display according to button pressed
       {
        float Counter = 1;
        res = I2C_ReadBlock(I2C_ADDRESS_MPU9150, buffer, 6, 0x43);

           rawGyroX = (buffer[0] << 8) | buffer[1]; // combine the high byte first then low byte
           rawGyroY = (buffer[2] << 8) | buffer[3]; // combine the high byte first then low byte
           rawGyroZ = (buffer[4] << 8) | buffer[5]; // combine the high byte first then low byte

           GyroX = (rawGyroX - GyroBiasX) / 131.0;// X value of gyro measurements
           GyroY = (rawGyroY - GyroBiasY) / 131.0;// Y value of gyro measurements
           GyroZ = (rawGyroZ - GyroBiasZ) / 131.0;// Z value of gyro measurements

           dt = Counter - dt; // time since last measurement was taken

           Pitch += -GyroX * dt;
           Roll += -GyroY * dt;
           Yaw += -GyroZ * dt;

           gTick_0_5ms = 40;      // wait for 20 ms
           LPM0;

           if (gbToggle == 0)
           {
               LCD_displayShort((short) Pitch);
               P1OUT |= BIT0;
               P9OUT &= ~BIT7;
           }

           if (gbToggle == 1)
           {
               LCD_displayShort((short) Roll);
               P1OUT &= ~BIT0;
               P9OUT |= BIT7;
           }

           if (gbToggle == 2)
           {
               LCD_displayShort((short) Yaw);
               P1OUT &= ~BIT0;
               P9OUT &= ~BIT7;
           }
           Counter++;
       }
}
#pragma vector=PORT1_VECTOR         // interrupt when button is pressed
__interrupt void Port1(void)
{
    P1IFG &= ~BIT1; // clear P1.1 interrupt flag in the end

        if (gbToggle >= 2)// when the buttons are pressed again and again
        {
            gbToggle = 0;
        }
        else
        {
            gbToggle = gbToggle + 1;
        }
}

#pragma vector=WDT_VECTOR
__interrupt void tick(void)
{
    if (gTick_0_5ms > 0)
    {
        gTick_0_5ms--;
        if (gTick_0_5ms == 0)
            LPM0_EXIT;
    }
}


