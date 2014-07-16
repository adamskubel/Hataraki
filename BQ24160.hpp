
#ifndef HATARAKI_DEVICES_BQ24160_HPP_
#define HATARAKI_DEVICES_BQ24160_HPP_

#include <iostream>
#include <vector>

#include "I2CDevice.hpp"

class BQ24160 : public I2CDevice {
	
	/*
	 b7
	 timer reset
	 
	 b4:6
	 status
	 
	 000- No Valid Source Detected
	 001- IN Ready (shows preferred source when both connected) 
	 010- USB Ready (shows preferred source when both connected) 
	 011- Charging from IN
	 100- Charging from USB
	 101- Charge Done
	 110- NA
	 111- Fault
	 
	 b3
	 0 = in
	 1 = usb
	 
	 b2:0
	 
	 000- Normal
	 001- Thermal Shutdown
	 010- Battery Temperature Fault
	 011- Watchdog Timer Expired (bq24160/1/1B/3 only) 
	 100- Safety Timer Expired (bq24160/1/1B/3 only) 
	 101- IN Supply Fault
	 110- USB Supply Fault
	 111- Battery Fault
	 
	 
	 */
	struct StatusRegister {
		
		int Status;
		int Fault;
		int Source;
		
		StatusRegister(unsigned char value)
		{
			Fault = 0x07 & value;
			Status = 0x07 & (value >> 4);
			Source = 0x01 & (valye >> 3);
			TimerReset = 0;
		}
	};
	
	/*
	 B6:7 - Input State
	 
	 00-Normal
	 01-Supply OVP
	 10-Weak Source Connected (No Charging) 
	 11- VIN<VUVLO
	 
	 B5:4 - USB Status
	 00-Normal
	 01-Supply OVP
	 01-Weak Source Connected (No Charging) 
	 11- VUSB<VUVLO
	 
	 B3 - OTG_Lock
	 0 – No OTG supply present. Use USB input as normal.
	 1 – OTG supply present. Lockout USB input for charging. (default 0)
	 
	 B2:1 - Battery Status
	 00-Battery Present and Normal 
	 01-Battery OVP
	 10-Battery Not Present
	 11- NA
	 
	 B0 - NoBattery Mode
	 0-Normal Operation
	 1-Enables No Battery Operation when termination is disabled (default 0)
	 
	 */
	struct BatterySupplyStatusRegister {
		
		int InStatus;
		int USBStatus;
		
				 
		BatterySupplyStatusRegister(unsigned char value)
		{
			
		}
	};
	
	/*
	 
	 B7 - Write Only
	 1 = Reset all registers
	 
	 B6:4 - Usb Current Limit
	 
	 000 – USB2.0 host with 100mA current limit
	 001 – USB3.0 host with 150mA current limit
	 010 – USB2.0 host with 500mA current limit
	 011 – USB host/charger with 800mA current limit 100 – USB3.0 host with 900mA current limit
	 101 – USB host/charger with 1500mA current limit 110–111 – NA (default 000(1))
	 
	 B3 - Enable/Disable STAT output
	 0 = Disable
	 1 = Enable
	 
	 B2 - Enable/Disable Charge Current Termination
	 
	 B1 - Enable/Disable not-charging
	 0 = Enable
	 1 = Enable
	 
	 B0 - High impedance mode
	 1 = Yes
	 0 = No
	 
	 */
	struct ControlRegister {
		
		int UsbCurrentLimit;
		
		bool StatOutputEnabled;
		bool ChargeCurrentEnabled;
		bool ChargingDisabled;
		bool HighImpedanceMode;
		
		bool ResetAllRegisters;
		
		ControlRegister(unsigned char value)
		{
			ResetAllRegisters = 0;
			
			UsbCurrentLimit = 0x07 & (value >> 4);
			
			StatOutputEnabled = (0x08 & value) != 0;
			ChargeCurrentEnabled = (0x04 & value) != 0;
			ChargingDisabled = (0x02 & value) != 0;
			HighImpedanceMode = (0x01 & value) != 0;
		}
	};
	
	
	/*
	 B7:2 - Battery Regulation Voltage
	 V_BATT = 3500 + (B[7:2] * 20) mv
	 
	 IINLIMIT
	 Read/Write
	 Input Limit for IN input- 0 – 1.5A
	 1 – 2.5A (default 0)
	 B0 (LSB)
	 D+/D–_EN
	 Read/Write
	 0 – Normal state, D+/D- Detection done
	 1 – Force D+/D– Detection. Returns to “0” after detection is done. (default 0)
	 */
	struct BatteryVoltageRegister {
		
	};
	
	/*
	 B7:3 - Charge current
	 I_CHARGE = 550 + (B[7:3] * 75) mA
	 Default value is 550 + 450 = 1A
	 
	 B2:0 - Termination current sense
	 I_TERM = 50 + (B[2:0]*50) mA
	 Default value is 50 + 100 = 150mA
	 
	 */
	struct ChargeCurrentRegister {
	};
	
	
	/*
	 B7 - Enable/Disable minimum system voltage mode
	 B6 - Enable/Disable Vin DPM mode (wtf is that?)
	 
	 B5:3 - USB Input DPM adjust, offset is 4.2V, scale is 80mV
	 B2:0 - 5V Input DPM adjust, offset is 4.2V, scale is 80mV
	 
	 */
	struct VoltageDPPMStatusRegister {
	};
	
	/*
	 
	 */
	struct SafetyTimerNTCMonitorRegister {
	};

};

#endif





















