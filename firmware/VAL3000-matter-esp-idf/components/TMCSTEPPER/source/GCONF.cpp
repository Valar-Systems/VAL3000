#include "TMCStepper.h"

#define SET_REG(SETTING) GCONF_register.SETTING = B; write(GCONF_register.address, GCONF_register.sr)

uint32_t TMC2208Stepper::GCONF() {
	return read(GCONF_register.address);
}
void TMC2208Stepper::GCONF(uint32_t input) {
	GCONF_register.sr = input;
	write(GCONF_register.address, GCONF_register.sr);
}

void TMC2208Stepper::I_scale_analog(bool B)		{ SET_REG(i_scale_analog);	}
void TMC2208Stepper::internal_Rsense(bool B)	{ SET_REG(internal_rsense);	}
void TMC2208Stepper::en_spreadCycle(bool B)		{ SET_REG(en_spreadcycle);	}
void TMC2208Stepper::shaft(bool B) 				{ SET_REG(shaft);			}
void TMC2208Stepper::index_otpw(bool B)			{ SET_REG(index_otpw);		}
void TMC2208Stepper::index_step(bool B)			{ SET_REG(index_step);		}
void TMC2208Stepper::pdn_disable(bool B)		{ SET_REG(pdn_disable);		}
void TMC2208Stepper::mstep_reg_select(bool B)	{ SET_REG(mstep_reg_select);}
void TMC2208Stepper::multistep_filt(bool B)		{ SET_REG(multistep_filt);	}

bool TMC2208Stepper::I_scale_analog()	{ TMC2208_n::GCONF_t r{0}; r.sr = GCONF(); return r.i_scale_analog;		}
bool TMC2208Stepper::internal_Rsense()	{ TMC2208_n::GCONF_t r{0}; r.sr = GCONF(); return r.internal_rsense;	}
bool TMC2208Stepper::en_spreadCycle()	{ TMC2208_n::GCONF_t r{0}; r.sr = GCONF(); return r.en_spreadcycle;		}
bool TMC2208Stepper::shaft()			{ TMC2208_n::GCONF_t r{0}; r.sr = GCONF(); return r.shaft;				}
bool TMC2208Stepper::index_otpw()		{ TMC2208_n::GCONF_t r{0}; r.sr = GCONF(); return r.index_otpw;			}
bool TMC2208Stepper::index_step()		{ TMC2208_n::GCONF_t r{0}; r.sr = GCONF(); return r.index_step;			}
bool TMC2208Stepper::pdn_disable()		{ TMC2208_n::GCONF_t r{0}; r.sr = GCONF(); return r.pdn_disable;		}
bool TMC2208Stepper::mstep_reg_select()	{ TMC2208_n::GCONF_t r{0}; r.sr = GCONF(); return r.mstep_reg_select;	}
bool TMC2208Stepper::multistep_filt()	{ TMC2208_n::GCONF_t r{0}; r.sr = GCONF(); return r.multistep_filt;		}
