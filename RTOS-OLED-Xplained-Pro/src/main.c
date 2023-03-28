#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

// Botao 1 do OLED 
#define BUT1_PIO PIOD
#define BUT1_PIO_ID ID_PIOD
#define BUT1_PIO_IDX 28
#define BUT1_PIO_IDX_MASK (1u<< BUT1_PIO_IDX)

// Botao 2 do OLED 
#define BUT2_PIO PIOC
#define BUT2_PIO_ID ID_PIOC
#define BUT2_PIO_IDX 31
#define BUT2_PIO_IDX_MASK (1u<< BUT2_PIO_IDX)

// Botao 3 do OLED
#define BUT3_PIO PIOA
#define BUT3_PIO_ID ID_PIOA
#define BUT3_PIO_IDX 19
#define BUT3_PIO_IDX_MASK (1u<< BUT3_PIO_IDX)

/** RTOS  */
#define TASK_OLED_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY            (tskIDLE_PRIORITY)

#define TASK_MODO_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_MODO_STACK_PRIORITY            (tskIDLE_PRIORITY)

#define TASK_MOTOR_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_MOTOR_STACK_PRIORITY            (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/** prototypes */
void but1_callback(void);
void but2_callback(void);
void but3_callback(void);
static void BUT_init(void);

//queue que os botoes mandam seu valor para task oled que guarda o valor do botao em uma lista e desenha * na tela
//caso valor digitado nao bata com lista a priori usar RTT e semaforo para bloquear por 5 segundos.
QueueHandle_t xQueueSenha;
/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

void but1_callback(void) {
	int but = 1;
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
  	xQueueSendFromISR(xQueueSenha, &but, &xHigherPriorityTaskWoken);
}
void but2_callback(void) {
	int but = 2;
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
  	xQueueSendFromISR(xQueueSenha, &but, &xHigherPriorityTaskWoken);

}
void but3_callback(void) {
	int but = 3;
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
  	xQueueSendFromISR(xQueueSenha, &but, &xHigherPriorityTaskWoken);
}
/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_oled(void *pvParameters) {
	gfx_mono_ssd1306_init();
	int senha[4] = {1,2,3,1};
	int contador = 0;
	int ultimo_digito;
	char display_string[4];
	char cofre[7];
	int errada = 0;
	int aberto = 0;
	for (;;)  {
		if(xQueueReceive(xQueueSenha,&(ultimo_digito),1000)){
			if(aberto && (ultimo_digito==1)){
				aberto = 0;
			}
			if((senha[contador] == ultimo_digito)){
				contador++;
			}else{
				//dar timeout e zerar contador
				errada = 1;
				contador = 0;
			}
			//zerando display
			gfx_mono_draw_filled_rect(0,0,128,32,GFX_PIXEL_CLR);
			//escrevendo se cofre esta aberto ou fechado
			if(contador <= 3){
				sprintf(cofre,"fechado");
			}else{
				sprintf(cofre,"aberto");
				aberto = 1;
				contador = 0;
			}

			if(errada){
				sprintf(cofre,"senha errada");
				errada = 0;
				//bloqueia cofre por 4 segundos
			}
			//desenhando string de * relativa ao contador
			for(int i=0;i<contador;i++){
				gfx_mono_draw_char('*',30+5*i,15,&sysfont);
			}
			gfx_mono_draw_string(cofre, 0, 0, &sysfont);
		}

	}
}


/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}

static void BUT_init(void) {
	//desativando watchdog
	WDT->WDT_CR=WDT_MR_WDDIS;

	/* configura prioridade */
	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_SetPriority(BUT1_PIO_ID, 4);

	/* conf but1 como entrada */
	pio_configure(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT1_PIO, BUT1_PIO_IDX_MASK, 60);
	pio_enable_interrupt(BUT1_PIO, BUT1_PIO_IDX_MASK);
	pio_handler_set(BUT1_PIO, BUT1_PIO_ID, BUT1_PIO_IDX_MASK, PIO_IT_FALL_EDGE , but1_callback);


		/* configura prioridade */
	NVIC_EnableIRQ(BUT2_PIO_ID);
	NVIC_SetPriority(BUT2_PIO_ID, 4);

	/* conf but2 como entrada */
	pio_configure(BUT2_PIO, PIO_INPUT, BUT2_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT2_PIO, BUT2_PIO_IDX_MASK, 60);
	pio_enable_interrupt(BUT2_PIO, BUT2_PIO_IDX_MASK);
	pio_handler_set(BUT2_PIO, BUT2_PIO_ID, BUT2_PIO_IDX_MASK, PIO_IT_FALL_EDGE , but2_callback);


		/* configura prioridade */
	NVIC_EnableIRQ(BUT3_PIO_ID);
	NVIC_SetPriority(BUT3_PIO_ID, 4);

	/* conf but3 como entrada */
	pio_configure(BUT3_PIO, PIO_INPUT, BUT3_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT3_PIO, BUT3_PIO_IDX_MASK, 60);
	pio_enable_interrupt(BUT3_PIO, BUT3_PIO_IDX_MASK);
	pio_handler_set(BUT3_PIO, BUT3_PIO_ID, BUT3_PIO_IDX_MASK, PIO_IT_FALL_EDGE , but3_callback);
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/


int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();
	configure_console();
	BUT_init();

	/* Create task to control oled */
	if (xTaskCreate(task_oled, "oled", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
	  printf("Failed to create oled task\r\n");
	}

	xQueueSenha = xQueueCreate(100, sizeof(int));
	if (xQueueSenha == NULL)
		printf("falha em criar a queue xQueueSenha \n");

	vTaskStartScheduler();

  /* RTOS n�o deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
