
/**
 * \file
 *
 * \brief SAM
 *
 * Copyright (C) 2013-2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#include <asf.h>
#include <stdio.h>
#include <conf_demo.h>
#include "oled1.h"
#include <math.h>

#define NR_STATES 6
#define NR_EVENTS 4

#define INIT_SYSTEM 0
#define LE_SENSOR 1
#define CALC_MEDIA 2
#define MOSTRA_DISPLAY 3
#define ESPERA_TAXA 4
#define RESET_APPICATION 5

#define MANTEM_ESTADO 0 
#define PROXIMO_ESTADO 1
#define ESTADO_ANTERIOR 2
#define HARD_RESET 3

#define TAM_BUFFER 50

typedef struct
{
	void (*ptrFunc) (void);
	uint8_t NextState;
} FSM_STATE_TABLE;

//prototipos dos estados
void init();
void le_sensor();
void calcula_media();
void mostra_display();
void ocioso();
void hard_reset();

// tabela de estados
const FSM_STATE_TABLE StateTable [NR_STATES][NR_EVENTS] =
{
      init, INIT_SYSTEM,                             init, LE_SENSOR,                       init, LE_SENSOR,                      init, RESET_APPICATION, 
      le_sensor, LE_SENSOR,                          le_sensor, CALC_MEDIA, 				le_sensor, CALC_MEDIA, 				  le_sensor, RESET_APPICATION, 
      calcula_media, CALC_MEDIA,                     calcula_media, MOSTRA_DISPLAY,   		calcula_media, MOSTRA_DISPLAY,		  calcula_media, RESET_APPICATION,
      mostra_display, MOSTRA_DISPLAY,                mostra_display, ESPERA_TAXA,			mostra_display, ESPERA_TAXA,		  mostra_display, RESET_APPICATION,
      ocioso, ESPERA_TAXA,                 			 ocioso, LE_SENSOR,						ocioso, MOSTRA_DISPLAY,   			  ocioso, RESET_APPICATION,   
	  hard_reset, INIT_SYSTEM,                       hard_reset, INIT_SYSTEM,               hard_reset, INIT_SYSTEM,              hard_reset, RESET_APPICATION,             
};

int evento = 0;
struct usart_module usart_instance;
struct usart_config usart_conf;

static OLED1_CREATE_INSTANCE(oled1, OLED1_EXT_HEADER);

void configure_rtc_count(void);
struct rtc_module rtc_instance;

void configure_adc(void);
struct adc_module adc_instance;

uint8_t temperatura_atual = 20, temp_media = 25, temp_max = 40, temp_min = 10 ;
uint16_t conversao_temperatura;
int x, y; 
char c[50];
char mensagem [20];

volatile uint8_t buffer_temp[TAM_BUFFER], i_buffer = 0, f_buffer = 0, cont_buffer = 0;

 

uint8_t page_data[EEPROM_PAGE_SIZE];

enum state {TEMP_ATUAL = 0 , TEMP_MEDIA , TEMP_MAX, TEMP_MIN}estado; //estados para o mostrador do display
	

void configure_eeprom(void);

// void configure_extint_channel(void);
// void configure_extint_callbacks(void);
// void extint_detection_callback(void);
// 
// void configure_extint_channel(void)
// {
// 	//! [setup_1]
// 	struct extint_chan_conf config_extint_chan;
// 	//! [setup_1]
// 	//! [setup_2]
// 	extint_chan_get_config_defaults(&config_extint_chan);
// 	//! [setup_2]
// 
// 	//! [setup_3]
// 	config_extint_chan.gpio_pin           = OLED1_BUTTON3_ID;
// 	config_extint_chan.gpio_pin_mux       = BUTTON_0_EIC_MUX;
// 	config_extint_chan.gpio_pin_pull      = EXTINT_PULL_UP;
// 	config_extint_chan.detection_criteria = EXTINT_DETECT_BOTH;
// 	//! [setup_3]
// 	//! [setup_4]
// 	extint_chan_set_config(BUTTON_0_EIC_LINE, &config_extint_chan);
// 	//! [setup_4]
// }


//! [setup]
void configure_eeprom(void)
{
	/* Setup EEPROM emulator service */
//! [init_eeprom_service]
	enum status_code error_code = eeprom_emulator_init();
//! [init_eeprom_service]

//! [check_init_ok]
	if (error_code == STATUS_ERR_NO_MEMORY) {
		while (true) {
			/* No EEPROM section has been set in the device's fuses */
		}
	}
//! [check_init_ok]
//! [check_re-init]
	else if (error_code != STATUS_OK) {
		/* Erase the emulated EEPROM memory (assume it is unformatted or
		 * irrecoverably corrupt) */
		printf("Erro de memoria!!\n");
		eeprom_emulator_erase_memory();
		eeprom_emulator_init();
	}
//! [check_re-init]
}

#if (SAMD || SAMR21)
void SYSCTRL_Handler(void)
{
	if (SYSCTRL->INTFLAG.reg & SYSCTRL_INTFLAG_BOD33DET) {
		SYSCTRL->INTFLAG.reg |= SYSCTRL_INTFLAG_BOD33DET;
		eeprom_emulator_commit_page_buffer();
	}
}
#endif
static void configure_bod(void)
{
#if (SAMD || SAMR21)
	struct bod_config config_bod33;
	bod_get_config_defaults(&config_bod33);
	config_bod33.action = BOD_ACTION_INTERRUPT;
	/* BOD33 threshold level is about 3.2V */
	config_bod33.level = 48;
	bod_set_config(BOD_BOD33, &config_bod33);
	bod_enable(BOD_BOD33);

	SYSCTRL->INTENSET.reg |= SYSCTRL_INTENCLR_BOD33DET;
	system_interrupt_enable(SYSTEM_INTERRUPT_MODULE_SYSCTRL);
#endif

}
//! [setup]


void configure_rtc_count(void)
{
	struct rtc_count_config config_rtc_count;

	rtc_count_get_config_defaults(&config_rtc_count);

	config_rtc_count.prescaler           = RTC_COUNT_PRESCALER_DIV_32;
	config_rtc_count.mode                = RTC_COUNT_MODE_16BIT;
	#ifdef FEATURE_RTC_CONTINUOUSLY_UPDATED
	config_rtc_count.continuously_update = true;
	#endif
	config_rtc_count.compare_values[0]   = 1000;
	rtc_count_init(&rtc_instance, RTC, &config_rtc_count);
	rtc_count_enable(&rtc_instance);
}

void configure_adc(void)
{
	struct adc_config config_adc;
	adc_get_config_defaults(&config_adc);
	config_adc.resolution = ADC_RESOLUTION_12BIT; 
	config_adc.positive_input = ADC_POSITIVE_INPUT_TEMP;
	adc_init(&adc_instance, ADC, &config_adc);
	adc_enable(&adc_instance);
}

void debounce(){
	volatile uint16_t i;
	for(i = 0; i < 30000; i++);
}

void init(){
	configure_rtc_count();
	rtc_count_set_period(&rtc_instance, 2000);
	configure_adc();
	oled1_init(&oled1);
	gfx_mono_init();
	estado  = TEMP_ATUAL;
	
	// teste de mem??
	configure_eeprom();
	configure_bod();
	
// 	page_data[0] = 10;
// 	eeprom_emulator_write_page(0, page_data);
// 	eeprom_emulator_commit_page_buffer();
	
	eeprom_emulator_read_page(0, page_data);
	temperatura_atual = page_data[0]; //a cada reset a temperatura ?ncrementada 10 unidades
	temp_media = page_data[1];
	temp_max = page_data[2];
	temp_min = page_data[3];
	cont_buffer = page_data [4];
	i_buffer = page_data [5];
	f_buffer = page_data [6];
	
	int c = 0, i;
	i = i_buffer;
	for(c = 7; c < cont_buffer; c++) {
		buffer_temp [i] = page_data [c];
		i = (i + 1)%TAM_BUFFER;
	}
	printf("Estou no Init!!\r\n"); //testes
	
	evento = PROXIMO_ESTADO;
}

void le_sensor(){
	port_pin_toggle_output_level(LED_0_PIN); //teste
	adc_start_conversion(&adc_instance);
	
	do {
		/* Aguarda a conversao e guarda o resultado em temperatura_atual */
	} while (adc_read(&adc_instance, &conversao_temperatura) == STATUS_BUSY); 
	//printf("CONVERSAO = %d\n", conversao_temperatura);
	temperatura_atual =  ((float)conversao_temperatura*3.3/(4096))/0.01;  // conversao se necessario
	//printf("Lendo do sensor !!\r\n");
	
	evento = PROXIMO_ESTADO;
}

void calcula_media(){
	
        // coloca o dado lido no buffer para calculo da media
        if(cont_buffer < TAM_BUFFER){
	        i_buffer = 0;
	        buffer_temp[f_buffer] = temperatura_atual;
	        f_buffer = (f_buffer + 1)%TAM_BUFFER;
	        cont_buffer++;
	    }else{
	        buffer_temp[f_buffer] = temperatura_atual;
	        i_buffer = (i_buffer + 1) % TAM_BUFFER;
	        f_buffer = (f_buffer + 1) % TAM_BUFFER;
        }
	
	
	//calcula media, max, min e atual e grava na memoria
	int j = TAM_BUFFER, somatorio = 0, i = i_buffer;
	
	//printf("Calculando media e gravando na memoria !!\r\n");
	if (temperatura_atual > temp_max){
		temp_max = temperatura_atual;
	}else if (temperatura_atual < temp_min){
		temp_min = temperatura_atual;
	}

	
	while (j--){
		
		somatorio = buffer_temp [i] + somatorio; 
		i = (i + 1)%TAM_BUFFER; 
	}
	temp_media = somatorio/cont_buffer;
	
	// gravacao na memoria fisica
	int c = 0 ;
	
	page_data[0] = temperatura_atual;
	page_data[1] = temp_media;
	page_data[2] = temp_max;
	page_data[3] = temp_min;
	page_data[4] = cont_buffer;
	page_data[5] = i_buffer;
	page_data[6] = f_buffer;
	printf("temp_media: %d !\n", temp_media);
	
	i = i_buffer;
	
	//grava? do buffer na memoria
	for(c = 7; c < cont_buffer; c++) {
		page_data[c] = buffer_temp [i];
		i = (i + 1)%TAM_BUFFER;
	} 
	eeprom_emulator_write_page(0, page_data);
	eeprom_emulator_commit_page_buffer();
	
	evento = PROXIMO_ESTADO;
}

//\fn (void) mostra_display
void mostra_display(){
	///mostra no display as informa?s media, max, min e atual
	printf("Mostrando no display !!\r\n");
	
	switch (estado){ // estados para as informa?s mostradas no display
		case TEMP_ATUAL:
			strcpy(mensagem, "Temperatura  Atual:");
			itoa ((int)temperatura_atual , c, 10);
			printf ("temp atual %d\n", (int)temperatura_atual);						
		break;
								
		case TEMP_MEDIA:
			strcpy(mensagem, "Temperatura  Media:");
			itoa ((int)temp_media, c, 10);
			printf ("temp media %d\n", (int)temp_media);
		break;
					
		case TEMP_MAX:
			strcpy(mensagem, "Temperatura Maxima:");
			itoa ((int)temp_max, c, 10);
			printf ("temp max %d\n", (int)temp_max);
		break;
					
		case TEMP_MIN:
			strcpy(mensagem, "Temperatura Minima:");
			itoa ((int)temp_min, c, 10);
			printf ("temp min %d\n", (int)temp_min);
		break;
	}
	
	x = 0;
 	y = 0;
 	gfx_mono_draw_string(mensagem, x, y, &sysfont);


	x = 54;
	y = 10;
	gfx_mono_draw_string("    ", x, y, &sysfont);

 	x = 54;
 	y = 10;
 	gfx_mono_draw_string(c, x, y, &sysfont);
	
	evento = PROXIMO_ESTADO;
}


//\fn (void) ocioso
///usa o tempo entre leituras do sensor para verificar o estado dos botoes do display
void ocioso(){
	
	/** caso o botao da Samr21 seja pressionado a tela reseta */
	if (rtc_count_is_compare_match(&rtc_instance, RTC_COUNT_COMPARE_0)) {
		rtc_count_clear_compare_match(&rtc_instance, RTC_COUNT_COMPARE_0);
		evento = PROXIMO_ESTADO;
		/**OLED1_BUTTON1_ID, botao esquerdo do oled decrementa o estado e muda o dado do display */ 
	}else if(oled1_get_button_state(&oled1, OLED1_BUTTON1_ID)){
		estado = (estado - 1) % 4;
		evento = ESTADO_ANTERIOR;
		debounce();
		/**OLED1_BUTTON3_ID, botao direito do oled incrementa o estado e muda o dado do display */
	}else if (oled1_get_button_state(&oled1, OLED1_BUTTON3_ID)){
		estado = (estado + 1) % 4;
		evento = ESTADO_ANTERIOR;
		debounce();
		/**OLED1_BUTTON2_ID, botao do meio do oled chama a funcao que zera as variaveis*/
	}else if (oled1_get_button_state(&oled1, OLED1_BUTTON2_ID)){
		evento = HARD_RESET;
		debounce();
	}else{
		evento = MANTEM_ESTADO;
	}
}

//\fn (void) hard_reset 
void hard_reset(){
	printf("HARD RESET DA APLICACAO\n");
	
	/** @var temperatura_atual 
	* temperatura atualmente medida*/ 
	temperatura_atual = 0;
	/** @var temp_max
	* temperatura maxima */ 
	temp_max = 0;
	/** @var temp_min
	* temperatura minima*/ 
	temp_min = 255;
	/** @var temp_media
	* temperatura media*/ 
	temp_media = 0;
	/** @var (volatile uint8_t) cont_buffer 
	*contador do buffer*/ 
	cont_buffer = 0;
	/** @var (volatile uint8_t) i_buffer 
	*inicio do buffer*/ 
	i_buffer = 0;
	/** @var (volatile uint8_t) f_buffer
	* final do buffer*/ 
	f_buffer = 0;
	/** @var c 
	*contador_percorre o for */ 
	int c = 0 ;
	
	/** \defgroup DADOS MODIFICADOS dados modificados 
	 * @{
	 * /
	// zera a memoria fisica utilizada
	/** @var (uint8_t) page_data [0]
	* temperatura atual zera*/ 
	page_data[0] = 0; 
	/** @var (uint8_t) page_data[1]
	*temperatura media zera*/
	page_data[1] = 0; 
	/** @var (uint8_t) page_data[2]
	*temperatura maxima zera*/
	page_data[2] = 0; 
	/** @var (uint8_t) page_data[3]
	*temperatura minima*/
	page_data[3] = 255; 
	/** @var (uint8_t) page_data[4] 
	*contador do buffer zera */
	page_data[4] = 0; 
	/** @var (uint8_t) page_data[5] 
	*inico do buffer zera*/
	page_data[5] = 0;
	/** @var (uint8_t) page_data[6] 
	*final do buffer zera*/
	page_data[6] = 0;
	
	/** @var (uint8_t) page_data[c] 
	*dados do buffer zerados*/
	for(c = 7; c < TAM_BUFFER; c++) {
		page_data[c] = 0;
	}
	/** @}*/
	
	eeprom_emulator_write_page(0, page_data);
	eeprom_emulator_commit_page_buffer();
	evento = PROXIMO_ESTADO;
}

//\fn (int) main 	do programa
int main (void){
	int i;
	
	/// Inicializacao do sistema
	system_init();

	// Temporario
	/** Uso da comunicacao serial*/
	/** \defgroup Pinagem da placa Pinos da placa
	 * @{
	 * /
	/** @*/ 
	usart_get_config_defaults(&usart_conf);
	usart_conf.baudrate    = 9600;
	usart_conf.mux_setting = EDBG_CDC_SERCOM_MUX_SETTING;
	usart_conf.pinmux_pad0 = EDBG_CDC_SERCOM_PINMUX_PAD0;
	usart_conf.pinmux_pad1 = EDBG_CDC_SERCOM_PINMUX_PAD1;
	usart_conf.pinmux_pad2 = EDBG_CDC_SERCOM_PINMUX_PAD2;
	usart_conf.pinmux_pad3 = EDBG_CDC_SERCOM_PINMUX_PAD3;
	stdio_serial_init(&usart_instance, EDBG_CDC_MODULE, &usart_conf);
	/** @}*/
	usart_enable(&usart_instance);
	
	
	uint8_t currentState = INIT_SYSTEM;
	while (1) {
		if (StateTable[currentState][evento].ptrFunc != NULL)
			StateTable[currentState][evento].ptrFunc();
	
		currentState = StateTable[currentState][evento].NextState;	
	}
}

