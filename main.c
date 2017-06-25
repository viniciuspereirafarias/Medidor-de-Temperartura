
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

/** @file */

/**
 * \defgroup Estados_da_Aplicacao Estados da aplicacao
 * @{
 */


/*! \def NR_STATES
    \brief Constante que guarda o numero de estados da aplicacao. Nesse caso, 6 estados.
*/
#define NR_STATES 6


/*! \def NR_EVENTS
    \brief Constante que guarda o numero de maximo de transicoes  da aplicacao. Nesse caso, 4 transicoes.
*/
#define NR_EVENTS 4


/*! \def INIT_SYSTEM
    \brief Estado responsavel pela inicializacao dos parâmetros e módulos essenciais a aplicaçao.
*/ 
#define INIT_SYSTEM 0


/*! \def LE_SENSOR
    \brief Estado que fica lendo a temperatura coletada do sensor
*/ 
#define LE_SENSOR 1


/*! \def CALC_MEDIA
    \brief Estado que calcula a temperatura media, temperatura maxima e temperatura minima, dada a temperatura coletada pelo sensor no estado anterior.
*/ 
#define CALC_MEDIA 2


/*! \def MOSTRA_DISPLAY
    \brief Estado que atualiza as informacoes da tela do display mostrando as informacoes.
    
    Details: A tela é atualizada seguindo a maquina de estados interna ao display, a qual é controlada pelos botões
    1 e 3 do display OLED.
*/ 
#define MOSTRA_DISPLAY 3


/*! \def ESPERA_TAXA
    \brief Estado que fica aguardando ate o termino de 2 segundos, quando a aplicacao lera o sensor novamente.
    
    Details: A tela é atualizada seguindo a maquina de estados interna ao display, a qual é controlada pelos botões
    1 e 3 do display OLED.
*/ 
#define ESPERA_TAXA 4


/*! \def RESET_APPICATION
    \brief Estado que reseta todas as informacoes da aplicacao, incluindo a memoria fisica.
*/ 
#define RESET_APPICATION 5


/*! \def MANTEM_ESTADO
    \brief Constante que é usada como evento para a transicao para o proprio estado.
*/ 
#define MANTEM_ESTADO 0 


/*! \def PROXIMO_ESTADO
    \brief Constante que é usada como evento para a transicao para o proprio estado.
    \brief Constante que é usada como evento para a transicao para o proximo estado.
*/ 
#define PROXIMO_ESTADO 1

/*! \def ESTADO_ANTERIOR
    \brief Constante que é usada como evento para a transicao para o proximo estado.
    \brief Constante que é usada como evento para a transicao para o estado anterior.
*/ 
#define ESTADO_ANTERIOR 2

/*! \def HARD_RESET
    \brief Constante que é usada como evento para a transicao para o estado reset.
*/ 
#define HARD_RESET 3

/** @} */ // fim do grupo ESTADOS_APLICACAO


/**
 * \defgroup Funcoes_estados Funcoes dos estados
 * @{
 */

/*! \fn init()
    \brief Funcao que representa o estado de inicializacao das variaveis e modulos necessarios para a aplicacao.
*/ 
void init();

/*! \fn le_sensor()
    \brief Funcao que representa o estado de leitura do sensor de temperatura da aplicacao.
*/ 
void le_sensor();

/*! \fn calcula_media()
    \brief Funcao que representa o estado de processamento para a obtencao dos valores necessarios para a aplicacao.
*/ 
void calcula_media();

/*! \fn mostra_display()
    \brief Funcao que representa o estado de atualizacao do display.
*/ 
void mostra_display();

/*! \fn ocioso()
    \brief Funcao que representa o estado de aguardo ate os dois segundos.
*/ 
void ocioso();

/*! \fn hard_reset()
    \brief Funcao que representa o estado de reset total da aplicacao.
*/ 
void hard_reset();

/** @} */ // fim do grupo FUNCOES DOS ESTADOS

/**
 * \defgroup TRANSICAO_ESTADOS Transicao entre os estados da aplicacao
 * @{
 */

 /*! \struct FSM_STATE_TABLE
    \brief Estrutura usada para transicao entre estados.
	
	Essa estrutura guarda um parâmetro para a função do determinado estado e um sinal para possiveis transiçoes
	desse estado para outros estados.
*/ 
typedef struct
{
	// @var prtFunc ponteiro para a funcao do estado atual
	void (*ptrFunc) (void);
	
	// @var  NextState variavel para transicao de estados
	uint8_t NextState;
} FSM_STATE_TABLE;

/**
* Matriz que contem a relacao entre os estados e possiveis transicoes para a aplicacao
*/
const FSM_STATE_TABLE StateTable [NR_STATES][NR_EVENTS] =
{
      init, INIT_SYSTEM,                             init, LE_SENSOR,                       init, LE_SENSOR,                      init, RESET_APPICATION, 
      le_sensor, LE_SENSOR,                          le_sensor, CALC_MEDIA, 				le_sensor, CALC_MEDIA, 				  le_sensor, RESET_APPICATION, 
      calcula_media, CALC_MEDIA,                     calcula_media, MOSTRA_DISPLAY,   		calcula_media, MOSTRA_DISPLAY,		  calcula_media, RESET_APPICATION,
      mostra_display, MOSTRA_DISPLAY,                mostra_display, ESPERA_TAXA,			mostra_display, ESPERA_TAXA,		  mostra_display, RESET_APPICATION,
      ocioso, ESPERA_TAXA,                 			 ocioso, LE_SENSOR,						ocioso, MOSTRA_DISPLAY,   			  ocioso, RESET_APPICATION,   
	  hard_reset, INIT_SYSTEM,                       hard_reset, INIT_SYSTEM,               hard_reset, INIT_SYSTEM,              hard_reset, RESET_APPICATION,             
};

/** @var evento 
* \brief Variavel utilizada para transicoes entre estados da aplicacao 
* Assim, cada funcao de estado seta essa variavel com um dos estados possiveis
* para a proxima transicao.
*/
int evento = 0;

/** @} */ // fim do grupo TRANSICAO_ESTADOS

/**
 * \defgroup Modulos Modulos da aplicacao
 * @{
 */

 /**
 * \defgroup USART USART
 * @{
 */
/** @struct usart_instance 
\brief Estrutura usada para usar a interface serial da aplicacao*/
struct usart_module usart_instance;
/** @struct usart_conf 
\brief estrutura para a configuracao do modulo usart 
para a comunicacao serial */
struct usart_config usart_conf;

/** @} */ // fim do grupo USART

/**
 * \defgroup OLED OLED
 * @{
 */
/** \def OLED1_CREATE_INSTANCE
 \brief Cria uma instancia para o uso do display OLED e atribui ele a 
 um EXT_HEADER (EXT 1 nesse caso)*/
static OLED1_CREATE_INSTANCE(oled1, OLED1_EXT_HEADER);

/** \var x
	\brief Variavel que marca a posicao x de escrita no display
*/
int x;

/** \var y
	\brief Variavel que marca a posicao y de escrita no display
*/
int y; 

/** \var c
	\brief Variavel que recebe o valor de temperaturas convertidas para string (itoa)
*/
char c[50];

/** \var mensagem
	\brief Variavel que recebe as mensagens temperatura atual, temperatura media, etc para a escrita no display
*/
char mensagem [20];

/** \enum state
*	\brief Enumerador para transicao entre estados internos do display OLED
*	Essa variavel controla qual sera a informacao mostrada no display OLED 
*	por uma maquina de estados 	propria e eh mudada de acordo com os botoes do 
*   display OLED.
*/
enum state {TEMP_ATUAL = 0 , TEMP_MEDIA , TEMP_MAX, TEMP_MIN}estado; //estados para o mostrador do display
	

/** @} */ // fim do grupo OLED


/**
 * \defgroup RTC RTC
 * @{
 */

/*! \fn configure_rtc_count()
    \brief Funcao que configura o RTC (timer) para um tempo de 2 segundos.
*/
void configure_rtc_count(void);

/** @struct rtc_instance 
\brief estrutura responsavel pela representacao do timer criado.
*/
struct rtc_module rtc_instance;

/** @} */ // fim do grupo RTC

/**
 * \defgroup ADC ADC 
 * @{
 */

/*! \def ADC_SAMPLES
    \brief Constante define quantos valores o ADC ira coletar a cada operacao. Depois
	sera feita uma media desses valores.
*/
#define ADC_SAMPLES 128

/*! \fn configure_adc()
    \brief Funcao que configura o ADC para o uso da aplicacao. 
*/
void configure_adc(void);

/** @struct adc_instance 
\brief Estrutura responsavel pela representacao do ADC configurado.
*/
struct adc_module adc_instance;

/** \fn configure_adc_callbacks
	\brief Funcao que configura as interrupcoes para o ADC
*/
void configure_adc_callbacks(void);

/** \fn adc_complete_callback
	\brief Rotina de tratamento de interrupcao que eh chamada quando o ADC termina uma conversao.
*/
void adc_complete_callback(struct adc_module *const module);

/** \var adc_result_buffer
	\brief Variavel que recebe as conversoes do ADC
*/
uint16_t adc_result_buffer[ADC_SAMPLES];

/** \var adc_read_done
	\brief Variavel que indica que o ADC terminou uma conversao
*/
volatile bool adc_read_done = false;

/** @} */ // fim do grupo ADC

/**
 * \defgroup EEPROM EEPROM 
 * @{
 */
 
/*! \fn configure_eeprom()
    \brief Funcao que configura a memoria EEPROM para uso. 
*/
void configure_eeprom(void);

/** \var page_data 
	\brief Buffer auxiliar para escrita e leitura da memoria EEPROM
*/
uint8_t page_data[EEPROM_PAGE_SIZE];


/** @} */ // fim do grupo EEPROM

/** @} */ // fim do grupo MODULOS


/**
 * \defgroup TEMPERATURA_INFO Informacoes temperatura
 * @{
 */
/** \var temperatura_atual 
	\brief Variavel que guarda a temperatura atual medida
*/
uint8_t temperatura_atual;

/** \var temp_media 
	\brief Variavel que guarda a temperatura media das temperaturas atuais medidas
*/
uint8_t temp_media;

/** \var temp_max 
	\brief Variavel que guarda a temperatura maxima medida
*/
uint8_t temp_max;

/** \var temp_min 
	\brief Variavel que guarda a temperatura minima medida
*/
uint8_t temp_min;

/** \var conversao_temperatura 
	\brief Variavel que guarda a media dos valores lidos pelo ADC
*/
uint16_t conversao_temperatura;

/*! \def TAM_BUFFER
    \brief Constante que marca o tamanho do buffer que sera usado para calculo da media.
*/ 
#define TAM_BUFFER 50

/** \var buffer_temp 
	\brief Buffer que guarda as ultimas temperaturas atuais para calculo da media
*/
volatile uint8_t buffer_temp[TAM_BUFFER];

/** \var i_buffer 
	\brief Guarda o inicio do buffer_temp
*/
volatile uint8_t i_buffer = 0;

/** \var f_buffer 
	\brief Guarda o final do buffer_temp
*/
volatile uint8_t f_buffer = 0;

/** \var cont_buffer 
	\brief Guarda o numero de posicoes preenchidas do buffer_temp
*/
volatile uint8_t cont_buffer = 0;

/** @} */ // fim do grupo TEMPERATURA_INFO


/** \fn adc_complete_callback
*
* Essa funcao eh um handler da interrupcao acionada pelo ADC
* Desse modo, ela eh chamada toda vez que o ADC termina uma conversao.
* Ela torna a variavel adc_read_done = true e permite que o restante do 
* software continue executando sem problemas.
* 
* \param module Representa a estrutura do ADC que foi inicializada anteriormente
*
*/
void adc_complete_callback(struct adc_module *const module)
{
	adc_read_done = true;
}

/** \fn configure_adc
*
* Funcao que inicializa os parametros do ADC como default. Depois, define a referencia do ADC
* como 1.7 V, define o PRESCALER como dividido por 8, define uma resolucao de 16 bits e seta
* o pino PB02 (ADC[10]) como entrada positiva. A entrada negativa eh aterrada e o modo de operacao 
* eh definido como single-ended.
* Depois, ativa o ADC.
*/
void configure_adc(void)
{
	struct adc_config config_adc;
	adc_get_config_defaults(&config_adc);
	#if (!SAML21) && (!SAML22) && (!SAMC21) && (!SAMR30)
	config_adc.gain_factor     = ADC_GAIN_FACTOR_DIV2;
	#endif
	config_adc.clock_prescaler = ADC_CLOCK_PRESCALER_DIV8;
	config_adc.reference       = ADC_REFERENCE_INT1V;
	#if (SAMC21)
	config_adc.positive_input  = ADC_POSITIVE_INPUT_PIN5;
	#else
	config_adc.positive_input  = ADC_POSITIVE_INPUT_PIN10;
	#endif
	config_adc.resolution      = ADC_RESOLUTION_16BIT;
	
	#if (SAMC21)
	adc_init(&adc_instance, ADC1, &config_adc);
	#else
	adc_init(&adc_instance, ADC, &config_adc);
	#endif
	
	adc_enable(&adc_instance);
}

/** \fn configure_adc_callbacks
*
* Habilita as interrupcoes geradas pelo ADC dado o modulo inicializado anteriormente. 
*
*/
void configure_adc_callbacks(void)
{
	adc_register_callback(&adc_instance, adc_complete_callback, ADC_CALLBACK_READ_BUFFER);
	adc_enable_callback(&adc_instance, ADC_CALLBACK_READ_BUFFER);
}

/** \fn configure_eeprom
*
* Inicializa o emulador da EEPROM e verifica se ha erros. Dois tipos de erros podem ocorrer:
* 1) Sem area de memoria: Nesse caso, a aplicacao entre em loop infinito e nao faz mais nada. 
* 2) Memoria corrompida: Nesse caso, a aplicacao reseta a memoria e a reinicia. 
* Caso nenhum dos erros ocorra, a memoria comeca a funcionar corretamente.
*/
void configure_eeprom(void)
{
	/* Setup EEPROM emulator service */
	enum status_code error_code = eeprom_emulator_init();

	if (error_code == STATUS_ERR_NO_MEMORY) {
		while (true) {
			/* No EEPROM section has been set in the device's fuses */
		}
	}
	else if (error_code != STATUS_OK) {
		/* Erase the emulated EEPROM memory (assume it is unformatted or
		 * irrecoverably corrupt) */
		printf("Erro de memoria!!\n");
		eeprom_emulator_erase_memory();
		eeprom_emulator_init();
	}
}

/** \fn SYSCTRL_Handler
*
* Interrupcao para a atualizacao da pagina na memoria nao-volatil acionada pelo BOD.
*/
#if (SAMD || SAMR21)
void SYSCTRL_Handler(void)
{
	if (SYSCTRL->INTFLAG.reg & SYSCTRL_INTFLAG_BOD33DET) {
		SYSCTRL->INTFLAG.reg |= SYSCTRL_INTFLAG_BOD33DET;
		eeprom_emulator_commit_page_buffer();
	}
}
#endif

/** \fn configure_bod
*	\brief Configuracao do BOD
*	Configuracao do BOD e estabelecimento de thresholds para o uso da memoria EEPROM. 
*/
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

/** \fn configure_rtc_count
*
*	Configuracao RTC com prescaler de 32 (divide por 32 o clock) e modo de contagem de 16 bits.
* 	Ainda, o periodo do timer eh setado como 2 segundos. Alem disso, essa funcao inicializa o timer
*   e torna ele ativo.	
*/

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

/** \fn debounce
*	\brief Debounce para os botoes
*	Rotina que gasta um pequeno tempo para o ajuste do debounce dos botoes do display OLED.
*/
void debounce(){
	volatile uint16_t i;
	for(i = 0; i < 30000; i++);
}

/** \fn init
*	Estado inicial da aplicacao. O RTC eh configurado para um periodo de 2 segundos, 
*   o display OLED eh inicializado, o ADC eh inicializado, as funcoes de escrita no display,
*   sao inicializadas, a maquina de estado de escrita no display eh inicializada, a memoria
*	EEPROM eh inicializada, sao ativadas as interrupcoes para a aplicacao, 
* 	e os valores das temperaturas atual, media, maxima e minima sao recuperados da memoria
*   EEPROM. Alem disso, o buffer_temp eh zerado e um evento para a transicao para o 
*   estado le_sensor eh gerado.
*/
void init(){
	configure_rtc_count();
	rtc_count_set_period(&rtc_instance, 2000);
	oled1_init(&oled1);
	gfx_mono_init();
	estado  = TEMP_ATUAL;
	
	configure_eeprom();
	configure_bod();
	
	configure_adc();
	configure_adc_callbacks();
	
	system_interrupt_enable_global();
		
	eeprom_emulator_read_page(0, page_data);
	temperatura_atual = page_data[0]; //a cada reset a temperatura ?ncrementada 10 unidades
	temp_media = page_data[1];
	temp_max = page_data[2];
	temp_min = page_data[3];
	cont_buffer = 0;
	i_buffer = 0;
	f_buffer = 0;
	
	printf("Estou no Init!!\r\n"); //testes
	
	evento = PROXIMO_ESTADO;
}

void le_sensor(){
	int i, soma = 0;
	
	port_pin_toggle_output_level(LED_0_PIN); //teste
	
	adc_read_buffer_job(&adc_instance, adc_result_buffer, ADC_SAMPLES);
	while (adc_read_done == false);
	/* Wait for asynchronous ADC read to complete */
	adc_read_done = false;
	
	for(i = 0; i< ADC_SAMPLES; i++){
		soma = soma + adc_result_buffer[i];
	}
	soma = soma/ADC_SAMPLES;
	printf("CONVERSAO = %d\n", soma);
	
	temperatura_atual = (((float)((1.7)*soma)/65536)/0.01);
	
	printf("Temperatura = %d\n", temperatura_atual);
		
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
	int j = cont_buffer, somatorio = 0, i = i_buffer;
	
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
	j = 0 ;
	
	page_data[0] = temperatura_atual;
	page_data[1] = temp_media;
	page_data[2] = temp_max;
	page_data[3] = temp_min;
	
	eeprom_emulator_write_page(0, page_data);
	eeprom_emulator_commit_page_buffer();
	
	evento = PROXIMO_ESTADO;
}

void mostra_display(){
	//mostra no display as informacoes media, max, min e atual
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


//\fn ocioso
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

//\fn hard_reset 
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
	/** @var cont_buffer 
	*contador do buffer*/ 
	cont_buffer = 0;
	/** @var i_buffer 
	*inicio do buffer*/ 
	i_buffer = 0;
	/** @var f_buffer
	* final do buffer*/ 
	f_buffer = 0;
		
	/** \defgroup DADOS MODIFICADOS dados modificados 
	 * @{
	 * /
	// zera a memoria fisica utilizada
	/** @var page_data [0]
	* temperatura atual zera*/ 
	page_data[0] = 0; 
	/** @var page_data[1]
	*temperatura media zera*/
	page_data[1] = 0; 
	/** @var page_data[2]
	*temperatura maxima zera*/
	page_data[2] = 0; 
	/** @varpage_data[3]
	*temperatura minima*/
	page_data[3] = 255; 
	
	eeprom_emulator_write_page(0, page_data);
	eeprom_emulator_commit_page_buffer();
	evento = PROXIMO_ESTADO;
}

//\fn main do programa
int main (void){	
	/// Inicializacao do sistema
	system_init();

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