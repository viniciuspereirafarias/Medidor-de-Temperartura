/**
 * \file
 *
 * \brief SAM ADC Quick Start
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

#define NR_STATES 5
#define NR_EVENTS 2

#define INIT_SYSTEM 0
#define LE_SENSOR 1
#define CALC_MEDIA 2
#define MOSTRA_DISPLAY 3
#define ESPERA_TAXA 4

#define MANTEM_ESTADO 0 
#define PROXIMO_ESTADO 1

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

// tabela de estados
const FSM_STATE_TABLE StateTable [NR_STATES][NR_EVENTS] =
{
      init, INIT_SYSTEM,                             init, LE_SENSOR, 
      le_sensor, LE_SENSOR,                          le_sensor, CALC_MEDIA, 
      calcula_media, CALC_MEDIA,                     calcula_media, MOSTRA_DISPLAY,   
      mostra_display, MOSTRA_DISPLAY,                mostra_display, ESPERA_TAXA,
      ocioso, ESPERA_TAXA,                 			 ocioso, LE_SENSOR,
};

int evento = 0;
struct usart_module usart_instance;
struct usart_config usart_conf;

void configure_rtc_count(void);
struct rtc_module rtc_instance;

void configure_adc(void);
struct adc_module adc_instance;

float temperatura_atual;
uint16_t conversao_temperatura;

void configure_rtc_count(void)
{
	struct rtc_count_config config_rtc_count;

	rtc_count_get_config_defaults(&config_rtc_count);

	config_rtc_count.prescaler           = RTC_COUNT_PRESCALER_DIV_1;
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
	adc_init(&adc_instance, ADC, &config_adc);
	adc_enable(&adc_instance);
}

void init(){
	configure_rtc_count();
	rtc_count_set_period(&rtc_instance, 2000);
	configure_adc();
	printf("Estou no Init!!\r\n");
	
	evento = PROXIMO_ESTADO;
}

void le_sensor(){
	port_pin_toggle_output_level(LED_0_PIN); //teste
	adc_start_conversion(&adc_instance);
	
	do {
		/* Aguarda a conversao e guarda o resultado em temperatura_atual */
	} while (adc_read(&adc_instance, &conversao_temperatura) == STATUS_BUSY); 
	temperatura_atual = conversao_temperatura; // conversao se necessario
	printf("Lendo do sensor !!\r\n");
	
	evento = PROXIMO_ESTADO;
}

void calcula_media(){
	//calcula media, max, min e atual e grava na memoria
	printf("Calculando media e gravando na memoria !!\r\n");
	
	evento = PROXIMO_ESTADO;
}

void mostra_display(){
	//mostra no display as informa絥s media, max, min e atual
	printf("Mostrando no display !!\r\n");
	
	evento = PROXIMO_ESTADO;
}

void ocioso(){
	printf("Estou ocioso por 2 segundos !!\r\n");
	if (rtc_count_is_compare_match(&rtc_instance, RTC_COUNT_COMPARE_0)) {
		rtc_count_clear_compare_match(&rtc_instance, RTC_COUNT_COMPARE_0);
		evento = PROXIMO_ESTADO;
	}else{
		evento = MANTEM_ESTADO;
	}
}

int main (void){
	
	
	// Inicializacao do sistema
	system_init();

	// Temporario
	usart_get_config_defaults(&usart_conf);
	usart_conf.baudrate    = 9600;
	usart_conf.mux_setting = EDBG_CDC_SERCOM_MUX_SETTING;
	usart_conf.pinmux_pad0 = EDBG_CDC_SERCOM_PINMUX_PAD0;
	usart_conf.pinmux_pad1 = EDBG_CDC_SERCOM_PINMUX_PAD1;
	usart_conf.pinmux_pad2 = EDBG_CDC_SERCOM_PINMUX_PAD2;
	usart_conf.pinmux_pad3 = EDBG_CDC_SERCOM_PINMUX_PAD3;
	stdio_serial_init(&usart_instance, EDBG_CDC_MODULE, &usart_conf);
	
	usart_enable(&usart_instance);
	
	printf("Oi !!\r\n");
	
	while (1) {
		uint8_t currentState = INIT_SYSTEM;
		
		if (StateTable[currentState][evento].ptrFunc != NULL)
			StateTable[currentState][evento].ptrFunc();

		currentState = StateTable[currentState][evento].NextState;
	}
}

