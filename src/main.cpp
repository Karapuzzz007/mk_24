#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

void setup(){
// Интерфейс U(S)ART с внешним миром
rcc_periph_clock_enable(RCC_GPIOA);                           // Разморозка порта ввода/вывода

gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9 | GPIO10);  // Режим альтернативной функции
gpio_set_af(GPIOA,GPIO_AF7, GPIO9 | GPIO10);                           // Альтернативная функция (выбор по номеру) PA9 --- Tx, PA10 --- Rx.

rcc_periph_clock_enable(RCC_USART1);                      // Разморозка ПУ

usart_set_baudrate(USART1, 115200);                       // Скорость передачи
usart_set_databits(USART1, 8);                            // Размер посылки
usart_set_stopbits(USART1, USART_STOPBITS_1);             // Количество стоп-битов
usart_set_parity(USART1, USART_PARITY_NONE);              // Контроль четности

usart_set_mode(USART1, USART_MODE_TX_RX);                 // Режим работы ПУ
usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);   // Управление процессом передачи сообщений

usart_enable(USART1);                                     // Включение ПУ

}

void loop(){
usart_send_blocking (USART1,0x55);
}
//--------------------------------------------------------------------------------
int main () {

setup();
while (true){
loop();
}
}