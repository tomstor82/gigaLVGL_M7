Arduino script for managing 2 heater relays, 1 water boiler relay and 1 inverter start relay. Utilises 4 DHT22 sensors for thermostatic control and CANBUS communication with BMS.

There are comprehensive coding to manage a daxtromn inverter through relay and solar input relay control as the inverter lacks many sought features found on higher end brands.

CANBUS is used for communication to and from the Orion JR 2 BMS which protects a Li battery bank, as well as clearing fault messages that appears.

The lvgl script includes 3 external fonts libraries for required fonts only, to keep size down, utilises touch buttons and data graphics to show required information at a glance.
More information is found in message boxes that pop up on touching specific items.

Features include

Solar sensing disabling MPPT if insufficient solar and MPPT is draining batteries or MPPT relay flapping. Disables for 10 minutes currently.
Power surge protection when MPPT has insufficient power to charge and inverter start, disables MPPT for 20s enabling inverter to start without issues.
Inverter Sleep mode turning it off for 3 minutes to conserve battery.
Charge current limit protection disabling solar if high voltage or 0 CCL from BMS
Discharge current limit protection disabling buttons for low voltage, DCL from BMS or DCH relay status.
Individual DCL limits for buttons.
Thermostatically controlled heaters controlled by 4 dht22 sensors and DCL protected and disabled if sensors fail.
Timer controlled hot water which also remains on if charging above a SOC threshold.
Inverter sleep timer and also conditions like solar will keep it on.

Message boxes for more data on dht22 sensors and BMS data with 10s refresh data interval set

Data display including arcs and easy reading of important information, including dynamic charge and usage arcs. Colours changing on battery icon depending on SOC and AMPS with a timer indicating charge or discharge time.
