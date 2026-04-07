#include "TMCStepper.h"

void TMC2209Stepper::begin() {
	defaults();
	push();
}

// Go through and set every setting
void TMC2209Stepper::defaults() {

	// General Registers
	GCONF_register.i_scale_analog = 0;
	GCONF_register.internal_rsense = 0; 
	GCONF_register.en_spreadcycle = 0;
	GCONF_register.shaft = 0;
	GCONF_register.index_otpw = 0;
	GCONF_register.index_step = 1;
	GCONF_register.pdn_disable = 1;
	GCONF_register.mstep_reg_select = 1;
	GCONF_register.multistep_filt = 1;
	
	// NODECONF Registers
	SLAVECONF_register.slaveaddr = 0;
	SLAVECONF_register.senddelay = 0;

	// Factory Registers
	FACTORY_CONF_register.ottrim = 0;

	// Velocity Dependent Control 
	IHOLD_IRUN_register.ihold = 0;
	IHOLD_IRUN_register.irun = 31;
	IHOLD_IRUN_register.iholddelay = 0;
	TPOWERDOWN_register.sr = 20;
	TPWMTHRS_register.sr = 0;
	VACTUAL_register.sr = 0;

	// StallGuard Control 
	TCOOLTHRS_register.sr = 0;
	SGTHRS_register.sr = 0;

	// Smart Energy Control CoolStep 
	COOLCONF_register.semin = 3; // 1-15 / If the StallGuard4 result falls below SEMIN*32, the motor CURRENT becomes increased to reduce motor load angle.
	COOLCONF_register.seup = 1; // 0-3 / For fast response to increasing motor load, use a high CURRENT increment step SEUP
	COOLCONF_register.semax = 2; // 0-15 / If the StallGuard4 result is equal to or above (SEMIN+SEMAX+1)*32, the motor CURRENT becomes decreased to save energy/ 0 to 2 recommended
	COOLCONF_register.sedn = 3; // For each StallGuard4 value decrease by one
	COOLCONF_register.seimin = 1; // Attention: use with IRUN≥20

	// CHOPCONF – Chopper Configuration 
	CHOPCONF_register.diss2vs = 0;
	CHOPCONF_register.diss2g = 0;
	CHOPCONF_register.dedge = 0;
	CHOPCONF_register.intpol = 1;
	CHOPCONF_register.mres = 0;
	CHOPCONF_register.vsense = 0;
	CHOPCONF_register.tbl = 2;
	CHOPCONF_register.hend = 3;
	CHOPCONF_register.hstrt = 5;
	CHOPCONF_register.toff = 3;
	
	//PWMCONF – Voltage PWM Mode StealthChop 
	PWMCONF_register.pwm_lim = 12;
	PWMCONF_register.pwm_reg = 1;	// Try 2
	PWMCONF_register.freewheel = 0; // Was 0
	PWMCONF_register.pwm_autograd= 1; 
	PWMCONF_register.pwm_autoscale = 1;
	PWMCONF_register.pwm_freq = 1;
	PWMCONF_register.pwm_grad = 10; // Test different initial values. Use scope. 
	PWMCONF_register.pwm_ofs = 36;
	
}

uint32_t TMC2209Stepper::IOIN() {
	return read(TMC2209_n::IOIN_t::address);
}
bool TMC2209Stepper::enn()			{ TMC2209_n::IOIN_t r{0}; r.sr = IOIN(); return r.enn;		}
bool TMC2209Stepper::ms1()			{ TMC2209_n::IOIN_t r{0}; r.sr = IOIN(); return r.ms1;		}
bool TMC2209Stepper::ms2()			{ TMC2209_n::IOIN_t r{0}; r.sr = IOIN(); return r.ms2;		}
bool TMC2209Stepper::diag()			{ TMC2209_n::IOIN_t r{0}; r.sr = IOIN(); return r.diag;		}
bool TMC2209Stepper::pdn_uart()		{ TMC2209_n::IOIN_t r{0}; r.sr = IOIN(); return r.pdn_uart;	}
bool TMC2209Stepper::step()			{ TMC2209_n::IOIN_t r{0}; r.sr = IOIN(); return r.step;		}
bool TMC2209Stepper::spread_en()	{ TMC2209_n::IOIN_t r{0}; r.sr = IOIN(); return r.spread_en;}
bool TMC2209Stepper::dir()			{ TMC2209_n::IOIN_t r{0}; r.sr = IOIN(); return r.dir;		}
uint8_t TMC2209Stepper::version() 	{ TMC2209_n::IOIN_t r{0}; r.sr = IOIN(); return r.version;	}

void TMC2209Stepper::push() {
	IHOLD_IRUN(IHOLD_IRUN_register.sr);
	TPOWERDOWN(TPOWERDOWN_register.sr);
	TPWMTHRS(TPWMTHRS_register.sr);
	GCONF(GCONF_register.sr);
	SLAVECONF(SLAVECONF_register.sr);
	VACTUAL(VACTUAL_register.sr);
	COOLCONF(COOLCONF_register.sr);
	CHOPCONF(CHOPCONF_register.sr);
	PWMCONF(PWMCONF_register.sr);
}

void TMC2209Stepper::SGTHRS(uint8_t input) {
	SGTHRS_register.sr = input;
	write(SGTHRS_register.address, SGTHRS_register.sr);
}
uint8_t TMC2209Stepper::SGTHRS() {
	return SGTHRS_register.sr;
}

// W: TCOOLTHRS
uint32_t TMC2209Stepper::TCOOLTHRS() { return TCOOLTHRS_register.sr; }
void TMC2209Stepper::TCOOLTHRS(uint32_t input) {
  TCOOLTHRS_register.sr = input;
  write(TCOOLTHRS_register.address, TCOOLTHRS_register.sr);
}

uint16_t TMC2209Stepper::SG_RESULT() {
	return read(TMC2209_n::SG_RESULT_t::address);
}
