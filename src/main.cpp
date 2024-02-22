#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

void setup (){
    // Интерфейс U(S)ART с внешним миром
rcc_periph_clock_enable(RCC_GPIOA);                           // Разморозка порта ввода/вывода

gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2 | GPIO3);  // Режим альтернативной функции
gpio_set_af(GPIOA,GPIO_AF7, GPIO2 | GPIO3);                           // Альтернативная функция (выбор по номеру) PA9 --- Tx, PA10 --- Rx.

rcc_periph_clock_enable(RCC_USART2);                      // Разморозка ПУ

usart_set_baudrate(USART2, 19200);                       // Скорость передачи
usart_set_databits(USART2, 8);                            // Размер посылки
usart_set_stopbits(USART2, USART_STOPBITS_1);             // Количество стоп-битов
usart_set_parity(USART2, USART_PARITY_NONE);              // Контроль четности

usart_set_mode(USART2, USART_MODE_TX_RX);                 // Режим работы ПУ
usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);   // Управление процессом передачи сообщений

usart_enable(USART2);                                     // Включение ПУ

}

void loop(){
usart_send_blocking (USART2 , 0x55);
}
int main (){
    setup();
   while(true){
    loop();

   } 
}
 