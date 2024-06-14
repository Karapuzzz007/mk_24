#include "ring_buf/Ring_buf.hpp"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/usart.h>

uint8_t str[20]{'a'}; //?

//Ring_buffer buf; 

void indicate_adc_val(uint32_t val){
if (val > 2048) gpio_set(GPIOE, GPIO11);
else gpio_clear(GPIOE, GPIO11);
}

uint32_t start_wait_and_read_adc(){
        adc_start_conversion_regular(ADC1);
        while (!adc_eoc(ADC1));
        uint32_t res = adc_read_regular(ADC1);
        return res;
}

void blocking_delay_parrots () {
    for (volatile uint32_t i = 0; i < 200000; i++); 
}

void setup_ADC () {
    	rcc_periph_clock_enable(RCC_ADC12);
        adc_power_off(ADC1);
        rcc_periph_reset_pulse(RST_ADC12);

        rcc_periph_clock_enable(RCC_GPIOA);
        gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0);

        adc_set_clk_prescale(ADC1, ADC_CCR_CKMODE_DIV2);
        adc_set_single_conversion_mode(ADC1);
        adc_set_sample_time(ADC1, 1, ADC_SMPR_SMP_7DOT5CYC);
        uint8_t channel_array[] = { 1 }; /* ADC1_IN1 (PA0) */
        adc_set_regular_sequence(ADC1, 1, channel_array);
        adc_power_on(ADC1);

        adc_start_conversion_regular(ADC1);
        while (!adc_eoc(ADC1));
}

void setup_USART (){
    // Интерфейс U(S)ART с внешним миром
rcc_periph_clock_enable(RCC_GPIOA);                           // Разморозка порта ввода/вывода
//rcc_periph_clock_enable(RCC_GPIOE);

gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2 | GPIO3);  // Режим альтернативной функции
gpio_set_af(GPIOA,GPIO_AF7, GPIO2 | GPIO3);                           // Альтернативная функция (выбор по номеру) PA9 --- Tx, PA10 --- Rx.

//gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO9|GPIO11);

rcc_periph_clock_enable(RCC_USART2);                      // Разморозка ПУ

usart_set_baudrate(USART2, 19200);                       // Скорость передачи
usart_set_databits(USART2, 8);                            // Размер посылки
usart_set_stopbits(USART2, USART_STOPBITS_1);             // Количество стоп-битов
usart_set_parity(USART2, USART_PARITY_NONE);              // Контроль четности

usart_set_mode(USART2, USART_MODE_TX_RX);                 // Режим работы ПУ
usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);   // Управление процессом передачи сообщений

//usart_enable_rx_interrupt(USART2);
//nvic_enable_irq(NVIC_USART2_EXTI26_IRQ);

usart_enable(USART2);                                     // Включение ПУ
}

void setup () {
    setup_ADC();
    setup_USART();

    rcc_periph_clock_enable(RCC_GPIOE);
    gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO9 | GPIO11 | GPIO15);
}

void loop(){
uint32_t res = start_wait_and_read_adc();
indicate_adc_val(res);

uint8_t res_str[7];

blocking_delay_parrots();
gpio_toggle(GPIOE, GPIO9);
usart_send_blocking(USART2, res);
}

int main (){

    setup();

   while(true){

    loop();

   } 
}
