#include "TMCStepper.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include <esp_log.h>

// Protected
// addr needed for TMC2209
TMC2208Stepper::TMC2208Stepper(uint32_t SerialPort, float RS, uint8_t addr) :
	TMCStepper(RS),
	slave_address(addr)
	{
		HWSerial = SerialPort;
	}

void TMC2208Stepper::push() {
	GCONF(GCONF_register.sr);
	IHOLD_IRUN(IHOLD_IRUN_register.sr);
	SLAVECONF(SLAVECONF_register.sr);
	TPOWERDOWN(TPOWERDOWN_register.sr);
	TPWMTHRS(TPWMTHRS_register.sr);
	VACTUAL(VACTUAL_register.sr);
	CHOPCONF(CHOPCONF_register.sr);
	PWMCONF(PWMCONF_register.sr);
}

bool TMC2208Stepper::isEnabled() { return !enn() && toff(); }

uint8_t TMC2208Stepper::calcCRC(uint8_t datagram[], uint8_t len) {
	uint8_t crc = 0;
	for (uint8_t i = 0; i < len; i++) {
		uint8_t currentByte = datagram[i];
		for (uint8_t j = 0; j < 8; j++) {
			if ((crc >> 7) ^ (currentByte & 0x01)) {
				crc = (crc << 1) ^ 0x07;
			} else {
				crc = (crc << 1);
			}
			crc &= 0xff;
			currentByte = currentByte >> 1;
		}
	}
	return crc;
}

void TMC2208Stepper::write(uint8_t addr, uint32_t regVal) {

	uint8_t datagram[8] = {0};

	datagram[0] = 0x05;
	datagram[1] = 0x00;
	datagram[2] = addr | TMC_WRITE;
	datagram[3] = (regVal >> 24) & 0xFF;
	datagram[4] = (regVal >> 16) & 0xFF;
	datagram[5] = (regVal >> 8 ) & 0xFF;
	datagram[6] = (regVal      ) & 0xFF;
	datagram[7] = calcCRC(datagram, 7);

	uart_write_bytes(UART_NUM_1, &datagram, sizeof(datagram));
	uart_wait_tx_done(UART_NUM_1, 100);
	uart_flush(UART_NUM_1);

}

uint32_t TMC2208Stepper::read(uint8_t addr) {
	
	uint8_t request_datagram[4]= {0};
	uint8_t reply_datagram[8] = {0};

	request_datagram[0] = 0x05;
	request_datagram[1] = 0x00;
	request_datagram[2] = addr;
	request_datagram[3] = calcCRC(request_datagram, 3);

	//The 32-bit data words are transferred with the highest byte first. 
	//Therefore the 4 bytes must be swapped with the byteswap function.
	//Since the serial interface uses only one line, all sent data automatically ends up in the RX buffer. Therefore the RX Buffer must be cleared after transmission

	TickType_t UART_MAX_DELAY = 2; // 5 worked ok
	uart_write_bytes(UART_NUM_1, &request_datagram, sizeof(request_datagram));
    uart_wait_tx_done(UART_NUM_1, UART_MAX_DELAY); // was 100 = one second
    uart_flush(UART_NUM_1);
    uart_read_bytes(UART_NUM_1, &reply_datagram, sizeof(reply_datagram), UART_MAX_DELAY);

	uint32_t out = ((uint32_t)reply_datagram[3] << 24) | ((uint32_t)reply_datagram[4] << 16) | (reply_datagram[5] << 8) | reply_datagram[6];
	return out;
}

uint8_t TMC2208Stepper::IFCNT() {
	return read(IFCNT_t::address);
}

void TMC2208Stepper::SLAVECONF(uint16_t input) {
	SLAVECONF_register.sr = input&0xF00;
	write(SLAVECONF_register.address, SLAVECONF_register.sr);
}
uint16_t TMC2208Stepper::SLAVECONF() {
	return SLAVECONF_register.sr;
}
void TMC2208Stepper::senddelay(uint8_t B) 	{ SLAVECONF_register.senddelay = B; write(SLAVECONF_register.address, SLAVECONF_register.sr); }
uint8_t TMC2208Stepper::senddelay() 		{ return SLAVECONF_register.senddelay; }

void TMC2208Stepper::OTP_PROG(uint16_t input) {
	write(OTP_PROG_t::address, input);
}

uint32_t TMC2208Stepper::OTP_READ() {
	return read(OTP_READ_t::address);
}

uint32_t TMC2208Stepper::IOIN() {
	return read(TMC2208_n::IOIN_t::address);
}
bool TMC2208Stepper::enn()			{ TMC2208_n::IOIN_t r{0}; r.sr = IOIN(); return r.enn;		}
bool TMC2208Stepper::ms1()			{ TMC2208_n::IOIN_t r{0}; r.sr = IOIN(); return r.ms1;		}
bool TMC2208Stepper::ms2()			{ TMC2208_n::IOIN_t r{0}; r.sr = IOIN(); return r.ms2;		}
bool TMC2208Stepper::diag()			{ TMC2208_n::IOIN_t r{0}; r.sr = IOIN(); return r.diag;		}
bool TMC2208Stepper::pdn_uart()		{ TMC2208_n::IOIN_t r{0}; r.sr = IOIN(); return r.pdn_uart;	}
bool TMC2208Stepper::step()			{ TMC2208_n::IOIN_t r{0}; r.sr = IOIN(); return r.step;		}
bool TMC2208Stepper::sel_a()		{ TMC2208_n::IOIN_t r{0}; r.sr = IOIN(); return r.sel_a;	}
bool TMC2208Stepper::dir()			{ TMC2208_n::IOIN_t r{0}; r.sr = IOIN(); return r.dir;		}
uint8_t TMC2208Stepper::version() 	{ TMC2208_n::IOIN_t r{0}; r.sr = IOIN(); return r.version;	}

uint16_t TMC2208Stepper::FACTORY_CONF() {
	return read(FACTORY_CONF_register.address);
}
void TMC2208Stepper::FACTORY_CONF(uint16_t input) {
	FACTORY_CONF_register.sr = input;
	write(FACTORY_CONF_register.address, FACTORY_CONF_register.sr);
}
void TMC2208Stepper::fclktrim(uint8_t B){ FACTORY_CONF_register.fclktrim = B; write(FACTORY_CONF_register.address, FACTORY_CONF_register.sr); }
void TMC2208Stepper::ottrim(uint8_t B)	{ FACTORY_CONF_register.ottrim = B; write(FACTORY_CONF_register.address, FACTORY_CONF_register.sr); }
uint8_t TMC2208Stepper::fclktrim()		{ FACTORY_CONF_t r{0}; r.sr = FACTORY_CONF(); return r.fclktrim; }
uint8_t TMC2208Stepper::ottrim()		{ FACTORY_CONF_t r{0}; r.sr = FACTORY_CONF(); return r.ottrim; }

void TMC2208Stepper::VACTUAL(uint32_t input) {
	VACTUAL_register.sr = input;
	write(VACTUAL_register.address, VACTUAL_register.sr);
}
uint32_t TMC2208Stepper::VACTUAL() {
	return VACTUAL_register.sr;
}

uint32_t TMC2208Stepper::PWM_SCALE() {
	return read(TMC2208_n::PWM_SCALE_t::address);
}
uint8_t TMC2208Stepper::pwm_scale_sum() {
	TMC2208_n::PWM_SCALE_t r{0};
	r.sr = PWM_SCALE();
	return r.pwm_scale_sum;
}

int16_t TMC2208Stepper::pwm_scale_auto() {
	TMC2208_n::PWM_SCALE_t r{0};
	r.sr = PWM_SCALE();
	return r.pwm_scale_auto;
	// Not two's complement? 9nth bit determines sign
	/*
	uint32_t d = PWM_SCALE();
	int16_t response = (d>>PWM_SCALE_AUTO_bp)&0xFF;
	if (((d&PWM_SCALE_AUTO_bm) >> 24) & 0x1) return -response;
	else return response;
	*/
}

// R: PWM_AUTO
uint32_t TMC2208Stepper::PWM_AUTO() {
	return read(PWM_AUTO_t::address);
}
uint8_t TMC2208Stepper::pwm_ofs_auto()  { PWM_AUTO_t r{0}; r.sr = PWM_AUTO(); return r.pwm_ofs_auto; }
uint8_t TMC2208Stepper::pwm_grad_auto() { PWM_AUTO_t r{0}; r.sr = PWM_AUTO(); return r.pwm_grad_auto; }
