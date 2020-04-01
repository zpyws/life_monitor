/* 
 * SGM4151x battery charging driver 
 * 
 * Copyright (C) 2019 SG MICRO 
 * 
 * This package is free software; you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License version 2 as 
 * published by the Free Software Foundation. 

 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR 
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED 
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE. 
 */ 


#ifndef _LINUX_SGM4151x_I2C_H 
#define _LINUX_SGM4151x_I2C_H 


//#include <linux/power_supply.h> 




struct SGM4151x_charge_param { 
	int vlim; 
	int ilim; 
	int ichg; 
	int vreg; 
}; 


enum stat_ctrl { 
	STAT_CTRL_STAT, 
	STAT_CTRL_ICHG, 
	STAT_CTRL_INDPM, 
	STAT_CTRL_DISABLE, 
}; 


enum vboost { 
	BOOSTV_4850 = 4850, 
	BOOSTV_5000 = 5000, 
	BOOSTV_5150 = 5150, 
	BOOSTV_5300	= 5300, 
}; 


enum iboost { 
	BOOSTI_500 = 500, 
	BOOSTI_1200 = 1200, 
}; 


enum vac_ovp { 
	VAC_OVP_5500 = 5500, 
	VAC_OVP_6200 = 6200, 
	VAC_OVP_10500 = 10500, 
	VAC_OVP_14300 = 14300, 
}; 




struct SGM4151x_platform_data { 
	struct SGM4151x_charge_param usb; 
	struct SGM4151x_charge_param ta; 
	int iprechg; 
	int iterm; 
	 
	enum stat_ctrl statctrl; 
	enum vboost boostv;	// options are 4850, 
	enum iboost boosti; // options are 500mA, 1200mA 
	enum vac_ovp vac_ovp; 
	 
}; 


#endif 
