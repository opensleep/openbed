# openbed
WIP project to build an open source cooling mattress. The goal of openbed is to provide a resource that allows you to affordably get a perfectly optimized sleep.

### Prototype 1 TODO:
- [x] Test feasibility (concept works, simple to implement)
- [x] Create a product design mockup
- [ ] 88/100 Source components
- [ ] 78/100 Build the thermal engine assembly for the TEC watercooling block
- [ ] 35/100 Design and build the case for all components that is also protected against water spillage
- [ ] 26/100 Design and implement an easily refillable 3D printed(?) water reservoir
- [ ] 15/100 Build mounting box for TEC controllers 
- [ ] 10/100 Build mount for Arduino controller module
- [ ] 5/100 Optimize quick connect system for modular accessories
- [ ] 5/100 Create a prototype cooling pillow

## First prototype
Proof of concept and testing components. 
Initial BOM was created after a teardown published on reddit from my reddit post: 
[https://www.reddit.com/r/EightSleep/comments/18xwllx/need_pictures_of_8sleep_pod3_internals_for_diy/](reddit link)

#### 8Sleep Pod 2 internals:
 - 4 peltier TEC 127106FX (water sealed version of the TEC12706 with slightly higher efficiency) $20
   - There are 2 peltier coolers for each side of the bed. So TEC12706 = 12v 6 amp peak draw * 2 = 12v 12amp necessary per person.
   - Two TEC12703's were acquired for testing to see if a lower power (3 amps) would result in significantly worse performance. 
 - A generic 12v water pump running at low power for reduced noise (one for each side). $16
 - A no-contact water level meter placed outside the water reservoir. $5
 - A thermometer placed on the heat sink to detect overheating and shut the unit down if it malfunctions. $3
 - A Meanwell (300W?) power supply $50
 - (assumed) A Thermometer within the water reservoir to confirm the correct temperature and adjust for drift. $3
 - A solenoid valve (not entirely sure of the purpose, probably for drainage or to help with temp control. $5
   - Reviews mention pod 2 has an annoying clicking noise, would be the solenoid valve, so this should be replaced anyways. 
 - Custom mounting brackets for the TECs, the cold side is only exposed to the waterblock, hot side is always facing the radiator unless polarity is reversed. $15
 - They use thin 0.14mm no mess thermal pads for the TEC mounts instead of thermal paste $3
 - Black TPV tubing secured with zip ties $15
 - Reservoir float valve (separate from the magnetic water level sensor? Max fill vs min fill?) $5
 - The waterblock is housed in styrofoam thermal insulation to isolate the cold plates from the hot radiator. $3
 - A custom ARM board running a cheap chipset. Has controls for polarity and current for TECS, and inputs for the other modules. (pi/arduino should work as well) $80
 - Case: injection molded plastic: $20 (after one-off mold cost)
 - Custom quick connect connectors: maybe $10
Note: Their radiator is not custom, all parts except for the arm board are available on Alibaba and Aliexpress. Radiator cost (bulk) $15 single sample ($80).
Total estimated pod cost (+ shipping, taxes, extra bom cost): ~$300

### Assembling the proof of concept
The first cooler test consisted of two TEC12703 peltier coolers. They were mounted to 40x80mm watercooling block, which was then mounted to generic CPU air cooler.
CPU thermal cooling paste was applied between the surface of the watercooling block and the CPU cooler copper contact plate.
