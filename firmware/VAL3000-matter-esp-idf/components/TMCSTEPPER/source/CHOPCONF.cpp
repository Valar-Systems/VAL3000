#include "TMCStepper.h"

#define SET_REG(SETTING) CHOPCONF_register.SETTING = B; write(CHOPCONF_register.address, CHOPCONF_register.sr)

// CHOPCONF
void TMC2208Stepper::CHOPCONF(uint32_t input) {
	CHOPCONF_register.sr = input;
	write(CHOPCONF_register.address, CHOPCONF_register.sr);
}
uint32_t TMC2208Stepper::CHOPCONF() {
	return read(CHOPCONF_register.address);
}
void TMC2208Stepper::toff	( uint8_t  B )	{ SET_REG(toff); 	}
void TMC2208Stepper::hstrt	( uint8_t  B )	{ SET_REG(hstrt); 	}
void TMC2208Stepper::hend	( uint8_t  B )	{ SET_REG(hend); 	}
void TMC2208Stepper::tbl	( uint8_t  B )	{ SET_REG(tbl); 	}
void TMC2208Stepper::vsense	( bool     B )	{ SET_REG(vsense); 	}
void TMC2208Stepper::mres	( uint8_t  B )	{ SET_REG(mres); 	}
void TMC2208Stepper::intpol	( bool     B )	{ SET_REG(intpol); 	}
void TMC2208Stepper::dedge	( bool     B )	{ SET_REG(dedge); 	}
void TMC2208Stepper::diss2g	( bool     B )	{ SET_REG(diss2g); 	}
void TMC2208Stepper::diss2vs( bool     B )	{ SET_REG(diss2vs); }

uint8_t TMC2208Stepper::toff()		{ TMC2208_n::CHOPCONF_t r{0}; r.sr = CHOPCONF(); return r.toff; 	}
uint8_t TMC2208Stepper::hstrt()		{ TMC2208_n::CHOPCONF_t r{0}; r.sr = CHOPCONF(); return r.hstrt; 	}
uint8_t TMC2208Stepper::hend()		{ TMC2208_n::CHOPCONF_t r{0}; r.sr = CHOPCONF(); return r.hend; 	}
uint8_t TMC2208Stepper::tbl()		{ TMC2208_n::CHOPCONF_t r{0}; r.sr = CHOPCONF(); return r.tbl;	 	}
bool 	TMC2208Stepper::vsense()	{ TMC2208_n::CHOPCONF_t r{0}; r.sr = CHOPCONF(); return r.vsense; 	}
uint8_t TMC2208Stepper::mres()		{ TMC2208_n::CHOPCONF_t r{0}; r.sr = CHOPCONF(); return r.mres; 	}
bool 	TMC2208Stepper::intpol()	{ TMC2208_n::CHOPCONF_t r{0}; r.sr = CHOPCONF(); return r.intpol; 	}
bool 	TMC2208Stepper::dedge()		{ TMC2208_n::CHOPCONF_t r{0}; r.sr = CHOPCONF(); return r.dedge; 	}
bool 	TMC2208Stepper::diss2g()	{ TMC2208_n::CHOPCONF_t r{0}; r.sr = CHOPCONF(); return r.diss2g; 	}
bool 	TMC2208Stepper::diss2vs()	{ TMC2208_n::CHOPCONF_t r{0}; r.sr = CHOPCONF(); return r.diss2vs; 	}
