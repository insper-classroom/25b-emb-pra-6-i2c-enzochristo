#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include "pico/stdlib.h"
#include <stdio.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "mpu6050.h"
#include "mouse.h"
#include "math.h"


#include "Fusion.h"
#define SAMPLE_PERIOD (0.01f) // replace this with actual sample period

const int MPU_ADDRESS = 0x68;
const int I2C_SDA_GPIO = 4;
const int I2C_SCL_GPIO = 5;

#define UART_TX_PIN 0
#define UART_RX_PIN 1

QueueHandle_t xQueuePos;

static void mpu6050_reset() {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);  // False - finished with bus

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }

    // Now temperature from reg 0x41 for 2 bytes
    // The register is auto incrementing on each read
    val = 0x41;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 2, false);  // False - finished with bus

    *temp = buffer[0] << 8 | buffer[1];
}

void mpu6050_task(void *p) {
    // configuracao do I2C
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);

    mpu6050_reset();
    int16_t acceleration[3], gyro[3], temp;

    FusionAhrs ahrs;
  FusionAhrsInitialise(&ahrs);
  mouse_t mouse;
  int gain = 2.5;
  
  while (true) { 

    mpu6050_read_raw(acceleration, gyro, &temp);
    FusionVector gyroscope = {
        .axis.x = gyro[0] / 131.0f, // Conversão para graus/s
        .axis.y = gyro[1] / 131.0f,
        .axis.z = gyro[2] / 131.0f,
    };

    FusionVector accelerometer = {
        .axis.x = acceleration[0] / 16384.0f, // Conversão para g
        .axis.y = acceleration[1] / 16384.0f,
        .axis.z = acceleration[2] / 16384.0f,
    };     
    
    FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);

    const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

    // nos passamos a velocidade angular, pense no caso em que estamos mudando na tela
    // e do nada paramos de mexer. Ele vai detectar isso por conta da velocidade e nao pela posicao.
    
    //eixo x
    printf("%d", gyroscope.axis.x);
    if (((gyroscope.axis.x  < -4) && (gyroscope.axis.x > -6)) ){
        // movimentando o eixo x, ou seja, roll
        mouse.val = 0;
    }else{
        mouse.axis = 0; 
        mouse.val = gyroscope.axis.x * gain;
        xQueueSend(xQueuePos, &mouse, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    // eixo y
    if (((gyroscope.axis.y) > -2) && (gyroscope.axis.y) < 0)
    {
        mouse.val = 0;
    }else{
        mouse.axis = 1; 
        mouse.val = gyroscope.axis.y * gain;
        xQueueSend(xQueuePos, &mouse, 0);
        vTaskDelay(pdMS_TO_TICKS(10));

    }

    // detectando o click
    if (acceleration[0] > 3e3){
        mouse.axis = 2;
        mouse.val = 0.1;
        xQueueSend(xQueuePos, &mouse, 0);

    }
    
    // printf("velocidade x: %0.1f, velocidade y: %0.1f, velocidade z: %0.1f\n", gyroscope.axis.x, gyroscope.axis.y, gyroscope.axis.z);

    // printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f\n", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);
    // roll eixo x e pitch y
    // printf("acceleration x : %d, acceleration y : %d, acceleration z: %d\n", acceleration[0], acceleration[1], acceleration[2]);
    // printf("eixo : %d, velocidade: %d\n", mouse.axis, mouse.val);

}
}

void uart_task(void *p){
    mouse_t mouse;
    uart_init(uart0, 115200);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    
    while(1){

        if(xQueueReceive(xQueuePos, &mouse, portMAX_DELAY)){
            // printf("%d , %d\n", mouse.axis, mouse.val);

            // byte de sincronismo
            uart_putc_raw(uart0, 0xFF);

            // byte representando eixo
            uart_putc_raw(uart0, mouse.axis);

            // LSB
            uart_putc_raw(uart0, mouse.val);
            // MSB
            uart_putc_raw(uart0, mouse.val >> 8);

            // stop byte
            // uart_putc_raw(uart0, -1);
        }
        
                
    }
}




int main() {
    stdio_init_all();
    xQueuePos = xQueueCreate(10, sizeof(mouse_t));    

    xTaskCreate(mpu6050_task, "mpu6050_Task 1", 8192, NULL, 1, NULL);
    xTaskCreate(uart_task, "Task 4", 1024, NULL, 1, NULL);


    vTaskStartScheduler();

    while (true)
        ;
}
