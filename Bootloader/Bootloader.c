/*
* Bootloader.c
*
* Created: 13/12/2012 5:28:26 PM
*  Author: Nikhil
*/

#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>

//#define SERIAL_DEBUG
//#define TESTING

// <avr/boot.h> uses sts instructions, but this version uses out instructions
// This saves cycles and program memory.
#include "boot.h"
#define RAMSTART (0x100)

/*******************************************************
EEROM Memory Organization
*******************************************************/

// Model No of Device
// Cant be 0x10, starts from 0x01 to 0xff
#define MODEL_NO 0x01
#define MODEL_NO_ADDRESS 1000

// Software Version of Device
//Will be of form VH:VL , either of them cant be 0x10
#define SOFTWARE_VER_HIGH		1001
#define SOFTWARE_VER_LOW		1002

// Flash Write Flag
#define FLASH_WRITE_FLAG 1003

// Flash write complete flag value
#define FLASH_WR_COMPLETE_VALUE 0xAB

// GPRS Settings for the device:  "APN","USER","PASS"
#define GPRS_SETTINGS	200
#define GPRS_SETTINGS_MAX_LENGTH		70

#define BAUD_RATE 115200

#define BOOT_SIZE 4096

#if BOOT_SIZE == 2048
#define MAX_ADDR	0x7800
#elif BOOT_SIZE == 4096
#define MAX_ADDR	0x7000
#endif

#define NRWWSTART		0x7000

#define MAX_ERROR_COUNT		10
#define MAX_ERROR_WRITE		4
#define PWRKEY _BV(1)
#define DTR _BV(0)

#define F_CPU 16000000L
#define BAUD_RATE 115200

#define UNIT_TIME (F_CPU/1024)

// Delay time counts
#define _100ms (65535 - (UNIT_TIME * .1))
#define _500ms (65535 - (UNIT_TIME * .5))
#define _1s (65535 - (UNIT_TIME * 1))
#define _2s (65535 - (UNIT_TIME * 2))
#define _4s (65535 - (UNIT_TIME * 4))

typedef enum{
	_START = 1,
	_SEND,
	_RECIEVE,
	_ERROR
}SEND;

typedef enum {
	_VERSION = 1,
	_LENGTH,
	_PAGE,
}CMD;

#define DEBUG_LED _BV(5) //PB5

/* Function Prototypes */
/* The main function is in init9, which removes the interrupt vector table */
/* we don't need. It is also 'naked', which means the compiler does not    */
/* generate any entry or exit code itself. */
//int main(void) __attribute__ ((naked)) __attribute__ ((section (".init9")));


#ifdef SERIAL_DEBUG
void println(char* msg);
void printhex(uint16_t num, uint8_t len);
void printdec(uint16_t num, uint8_t len);
#define printnum(num) printhex(num, 4)
#define printnet(num) printhex(num, 2)
#endif


static void print(char * str);
static void read(char *rec_buf,uint16_t wait_time,uint8_t restart);
static void delay(uint16_t wait_time);
static void write(uint8_t value);
static uint8_t eeprom_read(uint16_t address);
static void eeprom_write(uint16_t address, uint8_t value);
static uint8_t validImage(uint8_t* base);
void Blink_LED(uint16_t time);
uint16_t atoi_local(char * str, uint8_t len);
void itoa_local(char *str , uint16_t value);
void appStart() __attribute__ ((naked));

static char rec_buf[1024];
static uint16_t buf_length;

int main()
{
	// After the zero init loop, this is the first code to run.
	//
	// This code makes the following assumptions:
	//  No interrupts will execute
	//  SP points to RAMEND
	//  r1 contains zero
	//
	// If not, uncomment the following instructions:
	// cli();
	asm volatile ("clr __zero_reg__");
	
	//Configure PWRKEY and DTR as Output
	DDRB = (PWRKEY | DTR | DEBUG_LED);
	// DTR is LOW
	PORTB &= ~DTR;
	
	//Setup Timer 1 with prescale of 16 Mhz / 1024  = 64 us
	TCCR1B = _BV(CS12) | _BV(CS10);
	
	// Setup UART at 115200
	UCSR0A = _BV(U2X0); //Double speed mode USART0
	UCSR0B = _BV(RXEN0) | _BV(TXEN0);
	UCSR0C = _BV(UCSZ00) | _BV(UCSZ01);
	UBRR0L = (uint8_t)( (F_CPU + BAUD_RATE * 4L) / (BAUD_RATE * 8L) - 1 );
	///*
	// Delay for 16 sec to check whether the Module has Turned on
	// Each iteration is worth 4 sec
	uint8_t i;
	for(i = 4; i > 0;i--)
	{
		delay(_4s);
	}
	print("AT\r");
	read(rec_buf,_500ms,1);
	if(!strstr(rec_buf,"OK"))
	{
		// Turn the GSM Module H-L-H 2s
		PORTB |= PWRKEY;
		PORTB &= ~PWRKEY;
		delay(_2s);
		PORTB |= PWRKEY;
		for(i = 4; i > 0;i--)
		{
			delay(_4s);
		}
	}
	
	// Initialize the modem
	print("ATE0\r"); // Skip check for OK to save flash. Give delay of 100ms for return data
	delay(_100ms);
	
	print("AT+IPR=115200\r");
	delay(_100ms);
	
	// At high signal it seems to be around 10 - 15s,  so assuming that the device will in low signal area try to give high delay
	// So please do the registration check after atleast 40 - 50s after startup
	// Give some 500 ms delay and check for registration 5 times with 4 sec delay between attempts
	// Till this point the delay is 20 sec so add 20 sec more to this count
	for(i = 5; i > 0;i--)
	{
		delay(_4s);
	}
	
	for(i = 5; i > 0;i--)
	{
		print("AT+CREG?\r");
		read(rec_buf,_500ms,1);
		if((strstr(rec_buf,"0,5")) || (strstr(rec_buf,"0,1")))
		{
			break;
		}
		delay(_4s);
	}
	
	// Please skip all checks on the modem
	// All the errors will be checked when the device tries to connect to TCP
	// Initialize the TCP Connection by setting the APN, User and Pass
	print("AT+CGATT=1\r");
	read(rec_buf,_500ms,1);
	print("AT+CIPMUX=0\r");
	read(rec_buf,_500ms,1);
	// Check for APN  presence in eeprom by looking for "
	/*
	if(eeprom_read(GPRS_SETTINGS) == '"')
	{
		print("AT+CSTT=");
		for(i = GPRS_SETTINGS;i < (GPRS_SETTINGS + GPRS_SETTINGS_MAX_LENGTH);i++)
		{
			uint8_t temp;
			temp = eeprom_read(i);
			if(!temp)
			{
				break;
			}
			write(temp);
		}
		write('\r');
	}
	*/
	print("AT+CSTT=\"airtelgprs.com\",\"guest\",\"guest\"\r");
	read(rec_buf,_500ms,1);
	print("AT+CIICR\r");
	read(rec_buf,_4s,1);
	
	print("AT+CIFSR\r");
	read(rec_buf,_4s,1);

	// Initiate connection with the server and read the data
	//*/
	//print("ATE0\r"); // Skip check for OK to save flash. Give delay of 100ms for return data
	//delay(_100ms);
	
	SEND send_status = _START;
	CMD cmd_status = _VERSION;
	uint8_t ver_high , ver_low, no_pages,page_no;
	uint8_t Error_count = MAX_ERROR_COUNT;
	//uint8_t i;
	uint16_t write_addr;
	while(1)
	{
		if(!Error_count)
		{
			//Start Application based on Flash Write Flag
			uint8_t flag;
			flag = eeprom_read(FLASH_WRITE_FLAG);
			if(flag == FLASH_WR_COMPLETE_VALUE)
			{
				//Start Application
#ifndef  TESTING
				appStart();
#endif
#ifdef SERIAL_DEBUG
				println("Starting ");
#endif
				while(1)
				{
					Blink_LED(_2s);
				}
			}
			else
			{
				//LED Flash
#ifdef SERIAL_DEBUG
				println("Error App not started");
#endif
				while(1)
				{
					Blink_LED(_500ms);
				}
			}
		}
		if(send_status == _START)
		{
			for(i =4;i > 0; i--)
			{
				print("AT+CIPSTART=\"TCP\",\"lonewolf.freevar.com\",\"80\"\r\n"); // CONNECT
				read(rec_buf,_4s,1);
				if(strstr(rec_buf,"CONNECT"))
				{
					send_status = _SEND;
					break;
				}
				else
				{
					--Error_count;
				}
			}
			// What to do if i reaches 0
		}
		else if(send_status == _SEND)
		{
			send_status = _START;
			print("AT+CIPSEND\r\n"); // wait for >
			read(rec_buf,_500ms,1);
			if(strchr(rec_buf,'>'))
			{
				print("GET /update/");
				if(cmd_status == _VERSION)
				{
					print("vr");
				}
				else if(cmd_status == _LENGTH)
				{
					write(ver_high);
					write(ver_low);
					print("/ln");
				}
				else if(cmd_status == _PAGE)
				{
					uint8_t pageno_str[4];
					write(ver_high);
					write(ver_low);
					write('/');
					itoa(page_no,pageno_str,10);
					//itoa_local(page_no,pageno_str);
					print(pageno_str);
				}
				print(" HTTP/1.1\r\nHost: lonewolf.freevar.com\r\n\r\n");
				write(0x1A);
				read(rec_buf,_4s,1);
				read(rec_buf,_4s,0);
				send_status =_RECIEVE;
			}
			else
			{
				--Error_count;
			}
		}
		else if(send_status == _RECIEVE)
		{
			send_status = _START;
			if((strstr(rec_buf,"200 OK")))
			{
				unsigned char *ptr;
				// Data is of format ..    @l3l2l1l0@data
				ptr = (strchr(rec_buf,'@'));
				if(ptr)
				{
					uint16_t packet_length;
					// Terminate the recieved string at second @ to convert it to a integer
					//*(ptr + 5) = 0;
					//packet_length = atoi(ptr + 1);
					packet_length = atoi_local(ptr + 1,4);
					ptr += 6;
					if(cmd_status == _VERSION)
					{
						//get the version no's
						if(packet_length == 2)
						{
							ver_high = *(ptr);
							ver_low = *(ptr + 1);
							// Now Cross check the values with current version and then decide to update
							uint8_t curr_ver_high = eeprom_read(SOFTWARE_VER_HIGH);
							uint8_t curr_ver_low = eeprom_read(SOFTWARE_VER_LOW);
							if((curr_ver_high >= ver_high) && (curr_ver_low >= ver_low))
							{
#ifdef SERIAL_DEBUG
								println("No updates available");
#endif
								Error_count = 0;
							}
#ifdef SERIAL_DEBUG
							println("packet length = ");
							printnum(packet_length);
							println("curr version high = ");
							printnum(curr_ver_high);
							println("curr version low = ");
							printnum(curr_ver_low);
							println("version high = ");
							printnum(ver_high);
							println("version low = ");
							printnum(ver_low);
#endif
							cmd_status = _LENGTH;
						}
						
					}
					else if(cmd_status == _LENGTH)
					{
						//  get the no of pages
						//*(ptr + packet_length) = 0;
						//no_pages = atoi(ptr);
						no_pages = atoi_local(ptr,(uint8_t)packet_length);
						page_no = 1;
						cmd_status = _PAGE;
#ifdef SERIAL_DEBUG
						println("packet length = ");
						printnum(packet_length);
						println("no of pages = ");
						printnum(no_pages);
#endif
					}
					else if(cmd_status == _PAGE)
					{
						// get the binary data and start writing to memory
						if(no_pages--)
						{
#ifdef SERIAL_DEBUG
							println("current page = ");
							printnum(page_no);
#endif
							// Start writing data into Flash
							write_addr = (page_no - 1) << 9;
#ifdef SERIAL_DEBUG
							println("Writing Data for block ");
							printnum(page_no);
#endif
							// write_addr = 0 means we have not yet started writing to Flash
							if(write_addr == 0)
							{
								// Check for code validity
								if(!validImage(ptr))
								{
#ifdef SERIAL_DEBUG
									println("Invalid Image");
#endif
									// the code is corrupted so dont erase the flag also exit, below is a hack to exit
									write_addr = MAX_ADDR + 5;
								}
								else
								{
#ifdef SERIAL_DEBUG
									println("Valid Image");
#endif
									//Erase the flag in eeprom
									eeprom_write(FLASH_WRITE_FLAG,0xff);
								}
							}
							if(!((write_addr + packet_length) > MAX_ADDR))
							{
#ifdef SERIAL_DEBUG
								println("Writing data from address ");
								printnum(write_addr);
#endif
								uint16_t offset = 0; // Block offset
								offset = packet_length;
								while(packet_length % SPM_PAGESIZE)
								{
									packet_length++;
								}
#ifdef SERIAL_DEBUG
								println("Packet length adjusted to ");
								printnum(packet_length);
								//println("Offset= ");
								//printnum(offset);
								//println("Entering Loop");
#endif
								// As the packet_size is extended to block limit, extra increase in packet_length adds garbage at the end, so fill it with 0's
								while(offset < packet_length)
								{
									*(ptr + offset) = 0x00;
									offset++;
#ifdef SERIAL_DEBUG
									//println("Offset= ");
									//printnum(offset);
#endif
								}
								// Load data into buffer
								for(offset = 0 ; offset < packet_length;)
								{
									uint16_t write_value = (ptr[offset]) | ((ptr[offset+1]) << 8);
#ifndef TESTING
									//__boot_page_fill_short((write_addr + offset), write_value);
									boot_page_fill((write_addr + offset), write_value);
#endif
#ifdef SERIAL_DEBUG
									//if((offset == 0) || ((offset == (packet_length - 2))))
									//{
										println("Writing ");
										printnum(write_value);
										print(" at offset ");
										printnum(write_addr + offset);
									//}
#endif
									offset += 2;
									if(offset % SPM_PAGESIZE == 0)
									{
#ifdef SERIAL_DEBUG
										println("Flashing Data");
#endif
#ifndef TESTING
										uint8_t write_error = MAX_ERROR_WRITE;
										do
										{
											//__boot_page_erase_short(write_addr + offset - SPM_PAGESIZE);
											boot_page_erase(write_addr + offset - SPM_PAGESIZE);
											boot_spm_busy_wait();
											//__boot_page_write_short(write_addr + offset - SPM_PAGESIZE);
											boot_page_write(write_addr + offset - SPM_PAGESIZE);
											boot_spm_busy_wait();
											
#if defined(RWWSRE)
											// Reenable read access to flash
											boot_rww_enable();
#endif
#endif
											// Verifying whether the data has been written or not
											uint16_t temp_add = write_addr + offset - SPM_PAGESIZE;
											uint8_t temp_counter = SPM_PAGESIZE;
											uint16_t data_pointer = offset - SPM_PAGESIZE;
#ifdef SERIAL_DEBUG
											println("Verifying");
											println("");
#endif
											do
											{
												if(pgm_read_byte_near(temp_add) != *(ptr + data_pointer))
												{
#ifdef SERIAL_DEBUG
													println("Error while writing");
#endif
													offset-= SPM_PAGESIZE;
													write_error--;
													break;
												}
#ifdef SERIAL_DEBUG
												printnum(temp_add);
												print(" : ");
												printnum(pgm_read_byte_near(temp_add));
												println("");
#endif
												temp_add++;
												data_pointer++;
											}while (--temp_counter);
											if(!temp_counter)
											{
												// counter is zero, so no error
												write_error = 0;
											}
										}while(write_error);
									}
								}
							}
							else
							{
#ifdef SERIAL_DEBUG
								println("Flash Full");
#endif
								//Write addr is greater that Flash Size, Donot transfer control
								no_pages = 0xff; //Just some garbage to prevent transfer to next if statement
								Error_count = 0;
							}
							if(!no_pages)
							{
#ifdef SERIAL_DEBUG
								println("Flashing Complete");
#endif
								//Update Flash Write flag and Update the software version
								eeprom_write(FLASH_WRITE_FLAG,FLASH_WR_COMPLETE_VALUE);
								eeprom_write(SOFTWARE_VER_HIGH,ver_high);
								eeprom_write(SOFTWARE_VER_LOW,ver_low);
								// Start the Application
								// Instead of having multiple locations for app start,  put Error = 0 to start the app
								Error_count = 0;
							}
							page_no++;
						}
					}
				}
			}
			else
			{
				--Error_count;
#ifdef SERIAL_DEBUG
				println("Error has Occured :");
				printnum(Error_count);
#endif
			}
		}
	}
}

static void delay(uint16_t wait_time)
{
	TCNT1 = wait_time;
	TIFR1 |= _BV(TOV1);
	while(!(TIFR1 & _BV(TOV1)));
}

static void read(char *rec_buf,uint16_t wait_time,uint8_t restart)
{
	TCNT1 = wait_time;
	TIFR1 |= _BV(TOV1);
	if(restart)
	{
		buf_length = 0;
	}
	// if flag is set then timeout has exceeded return the buffer
	while(!(TIFR1 & _BV(TOV1)))
	{
		while(!(UCSR0A & _BV(RXC0)))
		{
			if(TIFR1 & _BV(TOV1))
			{
				break;
			}
		}
		rec_buf[buf_length++] = UDR0;
	}
	rec_buf[buf_length] = 0; // Make sure the string is terminated
}

static void print(char * str)
{
	uint8_t i;
	uint16_t length;
	length = strlen(str);
	i = 0;
	while(i<length)
	{
		while (!(UCSR0A & _BV(UDRE0)));
		UDR0 = str[i];
		i++;
	}
}

static void write(uint8_t value)
{
	while (!(UCSR0A & _BV(UDRE0)));
	UDR0 = value;
}

static void eeprom_write(uint16_t address, uint8_t value)
{
	while(EECR & (1 << EEPE));
	EEAR = address;
	EEDR = value;
	EECR |= (1<<EEMPE);
	EECR |= (1<<EEPE);
}

static uint8_t eeprom_read(uint16_t address)
{
	while(EECR & (1 << EEPE));
	EEAR = address;
	EECR |= (1 << EERE);
	return EEDR;
}


void appStart()
{
	__asm__ __volatile__(
	// Jump to RST vector
	"clr r30\n"
	"clr r31\n"
	"ijmp\n"
	);
}

void Blink_LED(uint16_t time)
{
	PORTB |= DEBUG_LED;
	delay(time);
	PORTB &= ~ DEBUG_LED;
	delay(time);
}


static uint8_t validImage(uint8_t* base)
{
	/* Check that a jump table is present in the first flash sector */
	uint8_t i;
	for(i = 0; i < 0x34; i += 4)
	{
		// For each vector, check it is of the form:
		// 0x0C 0x94 0xWX 0xYZ  ; JMP 0xWXYZ
		if(base[i] != 0x0c)
		{
			return(0);
		}
		if(base[i + 1] != 0x94)
		{
			return(0);
		}
	}
	return(1);
}

uint16_t atoi_local(char * str, uint8_t len)
{
	uint8_t i = 0;
	uint16_t value = 0;
	while(i < (len))
	{
		value = value * 10 + (*(str + i)-0x30);
		i++;
	}
	return value;
}

void itoa_local(char *str , uint16_t value)
{
	uint8_t i = 0,j,c;
	do
	{
		str[i++] = value%10 + 0x30;
		value = value / 10;
	}while(value);
	str[i]=0;
	j = i - 1;
	for (i = 0; i<j; i++, j--) 
	{
		c = str[i];
		str[i] = str[j];
		str[j] = c;
	}
}

#ifdef SERIAL_DEBUG
void println(char* msg)
{
	print("\r\n");
	print(msg);
}

void puthex(uint8_t c)
{
	c &= 0xf;
	if(c > 9) c += 7;
	while(!(UCSR0A & _BV(UDRE0)));
	UDR0 = c + '0';
}

void printhex(uint16_t num, uint8_t len)
{
	print("0x");
	while(len > 0) {
		puthex(num >> (4 * (len - 1)));
		len--;
	}
}
#endif

