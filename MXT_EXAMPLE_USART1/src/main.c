/**
 * \file
 *
 * \brief Example of usage of the maXTouch component with USART
 *
 * This example shows how to receive touch data from a maXTouch device
 * using the maXTouch component, and display them in a terminal window by using
 * the USART driver.
 *
 * Copyright (c) 2014-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */

/**
 * \mainpage
 *
 * \section intro Introduction
 * This simple example reads data from the maXTouch device and sends it over
 * USART as ASCII formatted text.
 *
 * \section files Main files:
 * - example_usart.c: maXTouch component USART example file
 * - conf_mxt.h: configuration of the maXTouch component
 * - conf_board.h: configuration of board
 * - conf_clock.h: configuration of system clock
 * - conf_example.h: configuration of example
 * - conf_sleepmgr.h: configuration of sleep manager
 * - conf_twim.h: configuration of TWI driver
 * - conf_usart_serial.h: configuration of USART driver
 *
 * \section apiinfo maXTouch low level component API
 * The maXTouch component API can be found \ref mxt_group "here".
 *
 * \section deviceinfo Device Info
 * All UC3 and Xmega devices with a TWI module can be used with this component
 *
 * \section exampledescription Description of the example
 * This example will read data from the connected maXTouch explained board
 * over TWI. This data is then processed and sent over a USART data line
 * to the board controller. The board controller will create a USB CDC class
 * object on the host computer and repeat the incoming USART data from the
 * main controller to the host. On the host this object should appear as a
 * serial port object (COMx on windows, /dev/ttyxxx on your chosen Linux flavour).
 *
 * Connect a terminal application to the serial port object with the settings
 * Baud: 57600
 * Data bits: 8-bit
 * Stop bits: 1 bit
 * Parity: None
 *
 * \section compinfo Compilation Info
 * This software was written for the GNU GCC and IAR for AVR.
 * Other compilers may or may not work.
 *
 * \section contactinfo Contact Information
 * For further information, visit
 * <A href="http://www.atmel.com/">Atmel</A>.\n
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */

#include <asf.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "tfont.h"
#include "maquina1.h"
#include "calibri_36.h"
#include "arial_72.h"
#include "conf_board.h"
#include "conf_example.h"
#include "conf_uart_serial.h"

#define MAX_ENTRIES        3
#define STRING_LENGTH     70

#define USART_TX_MAX_LENGTH     0xff

#define LED_PIO       PIOC
#define LED_PIO_ID    ID_PIOC
#define LED_IDX       8u
#define LED_IDX_MASK  (1u << LED_IDX)

	/* IMAGENS DEFINE */
 typedef struct {
	 const uint8_t *data;
	 uint16_t width;
	 uint16_t height;
	 uint8_t dataSize;
 } tImage2;
 #include "icones/anterior.h"
 #include "icones/proximo.h"
 #include "icones/play.h"
 #include "icones/stop.h"
 #include "icones/enxague.h"
 #include "icones/rapido.h"
 #include "icones/centrifuga.h"
 #include "icones/diario.h"
 #include "icones/pesado.h"
 #include "icones/lock.h"


#define YEAR        2018
#define MOUNT       3
#define DAY         19
#define WEEK        12
#define HOUR        15
#define MINUTE      45
#define SECOND      0

int segundos = 55;
int	minutos = 0;
int horas = 0;

char string_segundos[32];
char string_minutos[32];
char string_horas[32];

int isRunning = 0;
int isPressingLock = 0;
int isPressingLockCount = 0;
int isLocked = 0;
char test[32];


volatile Bool f_rtt_alarme = false;
static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses);


struct ili9488_opt_t g_ili9488_display_opt;
//const uint32_t BUTTON_W = 120;
//const uint32_t BUTTON_H = 150;

t_ciclo *p_primeiro;

const uint32_t previous_BUTTON_W = 93;
const uint32_t previous_BUTTON_H = 93;

const uint32_t next_BUTTON_W = 93;
const uint32_t next_BUTTON_H = 93;

const uint32_t left_BUTTON_W = 93;
const uint32_t left_BUTTON_H = 93;

const uint32_t x_timer = 60;
const uint32_t y_timer = 160;

const uint32_t x_lock = 190;
const uint32_t y_lock = 30;

const uint32_t proximo_x = 200;
const uint32_t proximo_y = 370;

const uint32_t anterior_x = 30;
const uint32_t anterior_y = 370;

const uint32_t play_stop_x = 115;
const uint32_t play_stop_y = 370;

const uint32_t previous_BUTTON_X = 30+96/2;;
const uint32_t previous_BUTTON_Y = 370+96/2;

const uint32_t next_BUTTON_X = 200+96/2;
const uint32_t next_BUTTON_Y = 370+96/2;

const uint32_t left_BUTTON_X = 115+96/2;
const uint32_t left_BUTTON_Y = 370+96/2;


//const uint32_t BUTTON_X = ILI9488_LCD_WIDTH/2;
//const uint32_t BUTTON_Y = ILI9488_LCD_HEIGHT/2;

const uint32_t BUTTON_BORDER = 2;

void pin_toggle(Pio *pio, uint32_t mask);

int led_flag = 0;

void draw_left_button(uint32_t clicked) {
	static uint32_t last_state = 255; // undefined
	if(clicked == last_state) return;

	if(clicked) {
		ili9488_draw_pixmap(play_stop_x,play_stop_y, stop.width, stop.height, stop.data);
		} else {
		ili9488_draw_pixmap(play_stop_x,play_stop_y, play.width, play.height, play.data);
	}
	last_state = clicked;
}

void draw_mode_icon(){
	if (p_primeiro==0x20400088){
		ili9488_draw_pixmap(90,210, enxague.width, enxague.height, enxague.data);
	}
	if (p_primeiro==0x20400100){
		ili9488_draw_pixmap(90,210, rapido.width, rapido.height, rapido.data);
	}
	if (p_primeiro==0x20400010){
		ili9488_draw_pixmap(90,210, centrifuga.width, centrifuga.height, centrifuga.data);
	}
	if (p_primeiro==0x2040004c){
		ili9488_draw_pixmap(90,210, diario.width, diario.height, diario.data);
	}
	if (p_primeiro==0x204000c4){
		ili9488_draw_pixmap(90,210, pesado.width, pesado.height, pesado.data);
	}
}

void draw_lock(){
	if(isLocked == 0)	{
		//subsituir aqui para o cadeado aberto
		ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE));
		ili9488_draw_filled_rectangle(x_lock, y_lock, x_lock+lock.width, y_lock+lock.height+20);
		ili9488_draw_pixmap(x_lock, y_lock, lock.width, lock.height, lock.data);
					
					
		
	}else{
		//arrumar aqui quando tiver o icone do cadeado aberto
		ili9488_draw_filled_rectangle(x_lock, y_lock, x_lock+lock.width, y_lock+lock.height+20);
		ili9488_draw_pixmap(x_lock, y_lock+20, lock.width, lock.height, lock.data);	
	
	}
}
void font_draw_text(tFont *font, const char *text, int x, int y, int spacing) {
	char *p = text;
	while(*p != NULL) {
		char letter = *p;
		int letter_offset = letter - font->start_char;
		if(letter <= font->end_char) {
			tChar *current_char = font->chars + letter_offset;
			ili9488_draw_pixmap(x, y, current_char->image->width, current_char->image->height, current_char->image->data);
			x += current_char->image->width + spacing;
		}
		p++;
	}
}

void increment_time(){
	if(isRunning){
		segundos -=1;
	}
	if(isPressingLock){
		isPressingLockCount +=1;		
		if(isPressingLockCount == 5){
			isLocked = !isLocked;
			draw_lock();
	
			
			isPressingLock = 0;
			isPressingLockCount = 0;	
		}
	}else{
		isPressingLockCount = 0;
	}
	sprintf(test,"%d",isPressingLockCount);
	font_draw_text(&calibri_36,test ,x_lock	, y_lock+40,2);
					

	
	if(segundos < 0){
		segundos = 59;
		
		ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE));
		ili9488_draw_filled_rectangle(x_timer, y_timer, ILI9488_LCD_WIDTH-1, ILI9488_LCD_HEIGHT-1);
		
		minutos -=1;
		
		if(minutos < 0 ){
			minutos = 59;
			
			ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE));
			ili9488_draw_filled_rectangle(x_timer, y_timer, ILI9488_LCD_WIDTH-1, ILI9488_LCD_HEIGHT-1);
			
			horas-=1;
		}
		if(horas < 0){
			pin_toggle(LED_PIO,LED_IDX_MASK);
			segundos = 0;
			minutos = 0;
			horas = 0;
			isRunning = 0;
			draw_left_button(0);
			ili9488_draw_pixmap(proximo_x, proximo_y, proximo.width, proximo.height, proximo.data);
			ili9488_draw_pixmap(anterior_x, anterior_y, anterior.width, anterior.height, anterior.data);
			ili9488_draw_pixmap(play_stop_x, play_stop_y, play.width, play.height, play.data);
			draw_mode_icon();

			
		}
	}
}



void print_time(){

	
	if(isRunning == 0)
	{
		minutos = p_primeiro->enxagueTempo * p_primeiro->enxagueQnt + p_primeiro->centrifugacaoTempo;
		minutos = 0; //teste
		horas = 0;
		segundos = 0;
		segundos = 11; //teste
	}
	
	sprintf(string_segundos,"%d",segundos);
	sprintf(string_minutos,"%d",minutos);
	sprintf(string_horas,"%d",horas);
	
	char buf[256];
	//sprintf(buf, sizeof buf, "%s%s%s%s%s", string_horas, "a", string_minutos, "a",string_segundos);
	//sprintf(buf,"%sh%s:%s",string_horas,string_minutos,string_segundos);
				
	if(strlen(string_horas) == 1){
		sprintf(buf,"0%s",string_horas);
		}else{
		sprintf(buf,"%s",string_horas);
	}
	if(strlen(string_minutos) == 1){
		sprintf(buf,"%s:0%s",buf,string_minutos);
		}else{
		sprintf(buf,"%s:%s",buf,string_minutos);
	}
	if(strlen(string_segundos) == 1){
		sprintf(buf,"%s:0%s",buf,string_segundos);
		}else{
		sprintf(buf,"%s:%s",buf,string_segundos);
					
	}
				
				
				
	ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE));
	font_draw_text(&calibri_36, buf,x_timer, y_timer, 2);
	font_draw_text(&calibri_36,p_primeiro->nome ,50	, 340,2);
	/*Draw dos icones*/
	//ili9488_draw_pixmap(proximo_x, proximo_y, proximo.width, proximo.height, proximo.data);
	//ili9488_draw_pixmap(anterior_x, anterior_y, anterior.width, anterior.height, anterior.data);

	//font_draw_text(&arial_72, string_minutos,130, 380, 2);
	//font_draw_text(&arial_72, string_segundos,220, 380, 2);
}

void RTT_Handler(void)
{
	//if(pulsos == idle_lastPulse){
		//idle_count +=1;
		//if(idle_count == 20){
			//idle_mode();
		//}
		//}else{
		//idle_count = 0;
	//}
	uint32_t ul_status;
	
	/* Get RTT status */
	ul_status = rtt_get_status(RTT);
	
	/* IRQ due to Time has changed */
	if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC) {  }
	
	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		//pin_toggle(LED_PIO, LED_IDX_MASK);    // BLINK Led
		f_rtt_alarme = true;                  // flag RTT alarme
		
		if(isRunning==1 || isPressingLock)
			{
				increment_time();
				print_time();
			}
	}

}

static void configure_lcd(void){
	/* Initialize display parameter */
	g_ili9488_display_opt.ul_width = ILI9488_LCD_WIDTH;
	g_ili9488_display_opt.ul_height = ILI9488_LCD_HEIGHT;
	g_ili9488_display_opt.foreground_color = COLOR_CONVERT(COLOR_WHITE);
	g_ili9488_display_opt.background_color = COLOR_CONVERT(COLOR_WHITE);

	/* Initialize LCD */
	ili9488_init(&g_ili9488_display_opt);
}

static float get_time_rtt(){
	uint ul_previous_time = rtt_read_timer_value(RTT);
}

static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses)
{
	uint32_t ul_previous_time;

	/* Configure RTT for a 1 second tick interrupt */
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	
	ul_previous_time = rtt_read_timer_value(RTT);
	while (ul_previous_time == rtt_read_timer_value(RTT));
	
	rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);

	/* Enable RTT interrupt */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 5);
	NVIC_EnableIRQ(RTT_IRQn);
	rtt_enable_interrupt(RTT, RTT_MR_ALMIEN);
}



t_ciclo *initMenuOrder(){
	c_rapido.previous = &c_enxague;
	c_rapido.next = &c_diario;

	c_diario.previous = &c_rapido;
	c_diario.next = &c_pesado;

	c_pesado.previous = &c_diario;
	c_pesado.next = &c_enxague;

	c_enxague.previous = &c_pesado;
	c_enxague.next = &c_centrifuga;

	c_centrifuga.previous = &c_enxague;
	c_centrifuga.next = &c_rapido;

	return(&c_diario);
}



/**
 * \brief Set maXTouch configuration
 *
 * This function writes a set of predefined, optimal maXTouch configuration data
 * to the maXTouch Xplained Pro.
 *
 * \param device Pointer to mxt_device struct
 */
static void mxt_init(struct mxt_device *device)
{
	enum status_code status;

	/* T8 configuration object data */
	uint8_t t8_object[] = {
		0x0d, 0x00, 0x05, 0x0a, 0x4b, 0x00, 0x00,
		0x00, 0x32, 0x19
	};

	/* T9 configuration object data */
	uint8_t t9_object[] = {
		0x8B, 0x00, 0x00, 0x0E, 0x08, 0x00, 0x80,
		0x32, 0x05, 0x02, 0x0A, 0x03, 0x03, 0x20,
		0x02, 0x0F, 0x0F, 0x0A, 0x00, 0x00, 0x00,
		0x00, 0x18, 0x18, 0x20, 0x20, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x02,
		0x02
	};

	/* T46 configuration object data */
	uint8_t t46_object[] = {
		0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x03,
		0x00, 0x00
	};
	
	/* T56 configuration object data */
	uint8_t t56_object[] = {
		0x02, 0x00, 0x01, 0x18, 0x1E, 0x1E, 0x1E,
		0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E,
		0x1E, 0x1E, 0x1E, 0x1E, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00
	};

	/* TWI configuration */
	twihs_master_options_t twi_opt = {
		.speed = MXT_TWI_SPEED,
		.chip  = MAXTOUCH_TWI_ADDRESS,
	};

	status = (enum status_code)twihs_master_setup(MAXTOUCH_TWI_INTERFACE, &twi_opt);
	Assert(status == STATUS_OK);

	/* Initialize the maXTouch device */
	status = mxt_init_device(device, MAXTOUCH_TWI_INTERFACE,
			MAXTOUCH_TWI_ADDRESS, MAXTOUCH_XPRO_CHG_PIO);
	Assert(status == STATUS_OK);

	/* Issue soft reset of maXTouch device by writing a non-zero value to
	 * the reset register */
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_COMMANDPROCESSOR_T6, 0)
			+ MXT_GEN_COMMANDPROCESSOR_RESET, 0x01);

	/* Wait for the reset of the device to complete */
	delay_ms(MXT_RESET_TIME);

	/* Write data to configuration registers in T7 configuration object */
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_POWERCONFIG_T7, 0) + 0, 0x20);
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_POWERCONFIG_T7, 0) + 1, 0x10);
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_POWERCONFIG_T7, 0) + 2, 0x4b);
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_POWERCONFIG_T7, 0) + 3, 0x84);

	/* Write predefined configuration data to configuration objects */
	mxt_write_config_object(device, mxt_get_object_address(device,
			MXT_GEN_ACQUISITIONCONFIG_T8, 0), &t8_object);
	mxt_write_config_object(device, mxt_get_object_address(device,
			MXT_TOUCH_MULTITOUCHSCREEN_T9, 0), &t9_object);
	mxt_write_config_object(device, mxt_get_object_address(device,
			MXT_SPT_CTE_CONFIGURATION_T46, 0), &t46_object);
	mxt_write_config_object(device, mxt_get_object_address(device,
			MXT_PROCI_SHIELDLESS_T56, 0), &t56_object);

	/* Issue recalibration command to maXTouch device by writing a non-zero
	 * value to the calibrate register */
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_COMMANDPROCESSOR_T6, 0)
			+ MXT_GEN_COMMANDPROCESSOR_CALIBRATE, 0x01);
}

void draw_screen(void) {
	ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE));
	ili9488_draw_filled_rectangle(0, 0, ILI9488_LCD_WIDTH-1, ILI9488_LCD_HEIGHT-1);
}

//void draw_button(uint32_t clicked) {
	//static uint32_t last_state = 255; // undefined
	//if(clicked == last_state) return;
	//
	//ili9488_set_foreground_color(COLOR_CONVERT(COLOR_BLACK));
	//ili9488_draw_filled_rectangle(BUTTON_X-BUTTON_W/2, BUTTON_Y-BUTTON_H/2, BUTTON_X+BUTTON_W/2, BUTTON_Y+BUTTON_H/2);
	//if(clicked) {
		//ili9488_set_foreground_color(COLOR_CONVERT(COLOR_TOMATO));
		//ili9488_draw_filled_rectangle(BUTTON_X-BUTTON_W/2+BUTTON_BORDER, BUTTON_Y+BUTTON_BORDER, BUTTON_X+BUTTON_W/2-BUTTON_BORDER, BUTTON_Y+BUTTON_H/2-BUTTON_BORDER);
	//} else {
		//ili9488_set_foreground_color(COLOR_CONVERT(COLOR_GREEN));
		//ili9488_draw_filled_rectangle(BUTTON_X-BUTTON_W/2+BUTTON_BORDER, BUTTON_Y-BUTTON_H/2+BUTTON_BORDER, BUTTON_X+BUTTON_W/2-BUTTON_BORDER, BUTTON_Y-BUTTON_BORDER);
	//}
	//last_state = clicked;
//}

void draw_next_button(uint32_t clicked) {
	static uint32_t last_state = 255; // undefined
	if(clicked == last_state) return;
	
	ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE));
	ili9488_draw_filled_rectangle(next_BUTTON_X-next_BUTTON_W/2, next_BUTTON_Y-next_BUTTON_H/2, next_BUTTON_X+next_BUTTON_W/2, next_BUTTON_Y+next_BUTTON_H/2);
	
	
}

void draw_previous_button(uint32_t clicked) {
	static uint32_t last_state = 255; // undefined
	if(clicked == last_state) return;
	
	ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE));
	ili9488_draw_filled_rectangle(previous_BUTTON_X-previous_BUTTON_W/2, previous_BUTTON_Y-previous_BUTTON_H/2, previous_BUTTON_X+previous_BUTTON_W/2, previous_BUTTON_Y+previous_BUTTON_H/2);
	
	
}



void io_init(void){

		
	/* led */
	pmc_enable_periph_clk(LED_PIO_ID);
	pio_configure(LED_PIO, PIO_OUTPUT_0, LED_IDX_MASK, PIO_DEFAULT);
}

uint32_t convert_axis_system_x(uint32_t touch_y) {
	// entrada: 4096 - 0 (sistema de coordenadas atual)
	// saida: 0 - 320
	return ILI9488_LCD_WIDTH - ILI9488_LCD_WIDTH*touch_y/4096;
}

uint32_t convert_axis_system_y(uint32_t touch_x) {
	// entrada: 0 - 4096 (sistema de coordenadas atual)
	// saida: 0 - 320
	return ILI9488_LCD_HEIGHT*touch_x/4096;
}

void pin_toggle(Pio *pio, uint32_t mask){
	
	if(pio_get_output_data_status(pio, mask)){
		pio_clear(pio, mask);
		}else{
		pio_set(pio,mask);
	}
}



void update_screen(uint32_t tx, uint32_t ty,uint32_t status) {

	
	if(status == 0x20)
			{
				//update left button
				
					if(tx >= play_stop_x && tx <= play_stop_x + play.width && ty >= play_stop_y && ty <= play_stop_y + play.height && isLocked == 0) {
						if(isRunning == 0){
							pin_toggle(LED_PIO, LED_IDX_MASK);
							led_flag = 1;
							isRunning =1;
							draw_left_button(1);
						}else{
							if(isRunning == 1){
								pin_toggle(LED_PIO, LED_IDX_MASK);
								led_flag = 0;
								isRunning = 0;
								draw_left_button(0);
							}
						}
					}

					
	
				
				
				
				if(tx >= previous_BUTTON_X - previous_BUTTON_W/2 && tx <= previous_BUTTON_X + previous_BUTTON_W/2 && ty >= previous_BUTTON_Y - previous_BUTTON_H/2 && ty <= previous_BUTTON_Y + previous_BUTTON_H/2 && isRunning == 0 && isLocked == 0) {
					p_primeiro = p_primeiro->previous;
					
					ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE));
					ili9488_draw_filled_rectangle(50,340, ILI9488_LCD_WIDTH-1, 380);
					font_draw_text(&calibri_36,p_primeiro->nome ,50	, 340,2);
					print_time();
				}
				
				if(tx >= next_BUTTON_X - next_BUTTON_W/2 && tx <= next_BUTTON_X + next_BUTTON_W/2 && ty >= next_BUTTON_Y - next_BUTTON_H/2 && ty <= next_BUTTON_Y + next_BUTTON_H/2 && isRunning == 0 && isLocked == 0) {
					p_primeiro = p_primeiro->next;
					
					ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE));
					ili9488_draw_filled_rectangle(50,340, ILI9488_LCD_WIDTH-1, 380);
					font_draw_text(&calibri_36,p_primeiro->nome ,50	, 340,2);
					print_time();
					
				}
				draw_mode_icon();
				
				if(tx >= x_lock && tx <= x_lock + lock.width && ty >= y_lock && ty <= y_lock+ lock.height) {
					isPressingLock = 0;
					isPressingLockCount = 0;
					font_draw_text(&calibri_36,"0" ,x_lock	, y_lock,2);
					
				}
			}else{
				if(tx >= x_lock && tx <= x_lock + lock.width && ty >= y_lock && ty <= y_lock+ lock.height) {
					isPressingLock = 1;
					font_draw_text(&calibri_36,"1" ,x_lock	, y_lock,2);
					
				}
						
			}
	

}

void mxt_handler(struct mxt_device *device)
{
	/* USART tx buffer initialized to 0 */
	char tx_buf[STRING_LENGTH * MAX_ENTRIES] = {0};
	uint8_t i = 0; /* Iterator */

	/* Temporary touch event data struct */
	struct mxt_touch_event touch_event;

	/* Collect touch events and put the data in a string,
	 * maximum 2 events at the time */
	do {
		/* Temporary buffer for each new touch event line */
		char buf[STRING_LENGTH];
	
		/* Read next next touch event in the queue, discard if read fails */
		if (mxt_read_touch_event(device, &touch_event) != STATUS_OK) {
			continue;
		}
		
		 // eixos trocados (quando na vertical LCD)
		uint32_t conv_x = convert_axis_system_x(touch_event.y);
		uint32_t conv_y = convert_axis_system_y(touch_event.x);
		
		/* Format a new entry in the data string that will be sent over USART */
		sprintf(buf, "Nr: %1d, X:%4d, Y:%4d, Status:0x%2x conv X:%3d Y:%3d\n\r",
				touch_event.id, touch_event.x, touch_event.y,
				touch_event.status, conv_x, conv_y);
		update_screen(conv_x, conv_y,touch_event.status);

		/* Add the new string to the string buffer */
		strcat(tx_buf, buf);
		i++;

		/* Check if there is still messages in the queue and
		 * if we have reached the maximum numbers of events */
	} while ((mxt_is_message_pending(device)) & (i < MAX_ENTRIES));

	/* If there is any entries in the buffer, send them over USART */
	if (i > 0) {
		usart_serial_write_packet(USART_SERIAL_EXAMPLE, (uint8_t *)tx_buf, strlen(tx_buf));
	}
}



int main(void)
{
	struct mxt_device device; /* Device data container */

	/* Initialize the USART configuration struct */
	const usart_serial_options_t usart_serial_options = {
		.baudrate     = USART_SERIAL_EXAMPLE_BAUDRATE,
		.charlength   = USART_SERIAL_CHAR_LENGTH,
		.paritytype   = USART_SERIAL_PARITY,
		.stopbits     = USART_SERIAL_STOP_BIT
	};
	
	f_rtt_alarme = true;
	

	sysclk_init(); /* Initialize system clocks */
	board_init();  /* Initialize board */
	configure_lcd();
	draw_screen();
	io_init();
	draw_left_button(0);
	
	pio_set(LED_PIO,LED_IDX_MASK);
	draw_lock();
	draw_next_button(0);
	draw_previous_button(0);
	/* Initialize the mXT touch device */
	mxt_init(&device);
	
	p_primeiro = initMenuOrder();
	print_time();
	
		
	
	font_draw_text(&calibri_36,p_primeiro->nome ,50	, 340,2);
	/*Draw dos icones*/
	ili9488_draw_pixmap(proximo_x, proximo_y, proximo.width, proximo.height, proximo.data);
	ili9488_draw_pixmap(anterior_x, anterior_y, anterior.width, anterior.height, anterior.data);
	ili9488_draw_pixmap(play_stop_x, play_stop_y, play.width, play.height, play.data);
	ili9488_draw_pixmap(90,210, diario.width, diario.height, diario.data);
	
	
	/* Initialize stdio on USART */
	stdio_serial_init(USART_SERIAL_EXAMPLE, &usart_serial_options);

	printf("\n\rmaXTouch data USART transmitter\n\r");
		

	while (true) {
	
		
		if (f_rtt_alarme){
      
		  /*
		   * O clock base do RTT é 32678Hz
		   * Para gerar outra base de tempo é necessário
		   * usar o PLL pre scale, que divide o clock base.
		   *
		   * Nesse exemplo, estamos operando com um clock base
		   * de pllPreScale = 32768/32768/2 = 2Hz
		   *
		   * Quanto maior a frequência maior a resolução, porém
		   * menor o tempo máximo que conseguimos contar.
		   *
		   * Podemos configurar uma IRQ para acontecer quando 
		   * o contador do RTT atingir um determinado valor
		   * aqui usamos o irqRTTvalue para isso.
		   * 
		   * Nesse exemplo o irqRTTvalue = 8, causando uma
		   * interrupção a cada 2 segundos (lembre que usamos o 
		   * pllPreScale, cada incremento do RTT leva 500ms (2Hz).
		   */
		  uint16_t pllPreScale = (int) (((float) 32768) / 32.0);
		  uint32_t irqRTTvalue  = 32;
      
		  // reinicia RTT para gerar um novo IRQ
		  RTT_init(pllPreScale, irqRTTvalue);
		 
		  
		  
      
		 /*
		  * caso queira ler o valor atual do RTT, basta usar a funcao
		  *   rtt_read_timer_value()
		  */
      
		  /*
		   * CLEAR FLAG
		   */
		  f_rtt_alarme = false;
		}
		
		/* Check for any pending messages and run message handler if any
		 * message is found in the queue */
		if (mxt_is_message_pending(&device)) {
			mxt_handler(&device);
		}
		 //pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
		 
		
		
	}

	return 0;
}
