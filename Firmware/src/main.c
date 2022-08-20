// Author: Joshua Southerland (2015)

// #define USE_CROSS_STUDIO_DEBUG

// STM32F405RG (see stm32f4xx.h to change target device)
// you have to uncomment one of the options, I chose:
//#define STM32F427_437xx

// assert_param undefined reference errors are solved by defining USE_STDPERIPH_DRIVER
//#define USE_STDPERIPH_DRIVER

// for startup file, make sure  STARTUP_FROM_RESET is defined

// also have to specify the external crystal speed
//#define HSE_VALUE=16000000

#include "stm32f4xx.h"

#include "wallaby.h"

 

int main()
{
    int32_t bemf_vals[4] = {0,0,0,0};
    int32_t bemf_vals_filt[4] = {0,0,0,0};

    init();

    // set up DMA/SPI buffers
    init_rx_buffer();
    init_tx_buffer();

    // set up pid structs
    pid_struct pid_structs[4];

    {        uint8_t i;
        for (i = 0; i < 4; ++i) init_pid_struct(&(pid_structs[i]), i);
    }

    update_dig_pin_configs();
    config_adc_in_from_regs();

    //debug_printf("starting\n");
    int low_volt_alarmed = 0;

    setupIMU();
    
    // Loop until button is pressed
    uint32_t count = 0;
    while (1)
    {
        count += 1;
        {
            // only sample motor backemf 1/4 of the time
            const uint8_t bemf_update_time =  (count % 4 == 1);
            if (bemf_update_time)
            {
                // idle breaking
                MOT0_DIR1_PORT->BSRRH |= MOT0_DIR1; 
                MOT0_DIR2_PORT->BSRRH |= MOT0_DIR2;
                MOT1_DIR1_PORT->BSRRH |= MOT1_DIR1; 
                MOT1_DIR2_PORT->BSRRH |= MOT1_DIR2;
                MOT2_DIR1_PORT->BSRRH |= MOT2_DIR1; 
                MOT2_DIR2_PORT->BSRRH |= MOT2_DIR2;
                MOT3_DIR1_PORT->BSRRH |= MOT3_DIR1; 
                MOT3_DIR2_PORT->BSRRH |= MOT3_DIR2;
            }
            // let the motor coast
            // delay_us(700);
            // now I squeeze in most sensor updates instead of just sleeping
            uint32_t before = usCount;
            update_dig_pins();
            int16_t batt = adc_update();
            //readAccel();
            //readMag();
            //readGyro();   
            if(count%2==0)readIMU();

            if (batt < 636) // about 5.75 volts
            {
                configDigitalOutPin(LED1_PIN, LED1_PORT);
                low_volt_alarmed = 1;
                if (count % 50 < 10) // low duty cycle to save battery
                {
                    LED1_PORT->BSRRL |= LED1_PIN; // ON
                }
                else
                {
                    LED1_PORT->BSRRH |= LED1_PIN; // OFF
                }
            }
            else if (low_volt_alarmed)
            {
                // make sure led is off coming out of the low voltage alarm
                configDigitalOutPin(LED1_PIN, LED1_PORT);
                LED1_PORT->BSRRH |= LED1_PIN; // OFF
                low_volt_alarmed = 0;
            }

            if (adc_dirty)
            {
                adc_dirty = 0;
                config_adc_in_from_regs();
            }

            if (dig_dirty)
            {
                dig_dirty = 0;
                update_dig_pin_configs();
            }

            uint32_t sensor_update_time = usCount - before;
            const uint16_t us_delay_needed = 700;
            const uint8_t got_time_to_burn = sensor_update_time < us_delay_needed;

            if (got_time_to_burn)
            {
                // sleep the remainder of the coasting period before sampling bemf
                delay_us(us_delay_needed-sensor_update_time);
            }

            if (bemf_update_time)
            {
                update_bemfs(bemf_vals, bemf_vals_filt);

                // set the motor directions back up
                update_motor_modes();

                uint8_t channel;
                for (channel = 0; channel < 4; ++channel)
                {
                    uint8_t shift = 2*channel;
                    uint8_t motor_mode = (aTxBuffer[REG_RW_MOT_MODES] & (0b11 << shift)) >> shift;
                    motor_update(bemf_vals[channel], bemf_vals_filt[channel], &pid_structs[channel], channel, motor_mode);
                }

            }
            else
            {
                // updating sensors doesnt' take as long as all of the adc for backemf updates
                // sleep a bit so that our time through the loop is consistent for PID control
                delay_us(222);
            }

            //aTxBuffer[REG_RW_MOT_SRV_ALLSTOP] = 0;//FIXME: remove
        }
    } 

    // set all motor pwms to 0
    aTxBuffer[REG_RW_MOT_0_PWM_H] = 0;
    aTxBuffer[REG_RW_MOT_0_PWM_L] = 0;
    aTxBuffer[REG_RW_MOT_1_PWM_H] = 0;
    aTxBuffer[REG_RW_MOT_1_PWM_L] = 0;
    aTxBuffer[REG_RW_MOT_2_PWM_H] = 0;
    aTxBuffer[REG_RW_MOT_2_PWM_L] = 0;
    aTxBuffer[REG_RW_MOT_3_PWM_H] = 0;
    aTxBuffer[REG_RW_MOT_3_PWM_L] = 0;

    // set all motors to idle state
    idle_motor_dirs();

    spi2_dma_cleanup();

    //debug_printf("done\n");
}
