# openbed
WIP project to build an open source cooling mattress. The goal of openbed is to provide a resource that allows you to affordably get a perfectly optimized sleep. To achieve this goal, all components should be sourced from common items without overuse of speciality vendors and custom parts. If a custom part is used, it should be simple to install and order a replacement from the design files. Soldering should be minimal and all parts should be simple to swap. Included software should be entirely open source and work without a network connection.

**OpenBed/OpenCooler/Pod (haven't decided on a final name) will be hot swappable with several potential accessories:**
1. A pet cooler pad in large (dog) and small (cat/other) sizes to give your animal the best sleep possible
2. A pillow cover for a standard sized pillow
3. A single mattress cover, then mattress covers of other sizes.
4. Chiller connect for bioreactors
5. Medical hospital mat version for post surgery and long term care
6. Wearable option for sports/vr training and recovery 

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
Proof of concept and testing components. The cooler prototype is meant to heat and cool a single mattress/pillow/blanket. It is not a split design for two people. Future iterations could do both, but for now I am focusing on raw cooling power and simplicity of design. No valves to exchange water flow, no separate reservoirs for hot and cool water. 

Initial BOM was created after a teardown published on reddit from my reddit post: 

[reddit link](https://www.reddit.com/r/EightSleep/comments/18xwllx/need_pictures_of_8sleep_pod3_internals_for_diy/)

#### 8Sleep Pod 2 internals:
| Component & key notes | Est. cost (USD) |
|---|---:|
| 4× Peltier TEC 127106FX (water-sealed TEC12706 variant; slightly higher efficiency)<br>• 2 Peltier coolers per bed side → TEC12706 draws 12 V × 6 A peak → per person: 12 V, ~12 A required<br>• Two TEC12703 units acquired for testing (lower-power 3 A option) | $20 |
| Generic 12 V water pump (run at low power for reduced noise; one per side) | $16 |
| No-contact water level meter (mounted outside reservoir) | $5 |
| Thermometer on heat sink (detect overheating; shut down on fault) | $3 |
| Meanwell power supply (≈300 W?) | $50 |
| Thermometer in water reservoir (confirm temp; adjust for drift) | $3 |
| Solenoid valve (likely for drainage or temp control)<br>• Reviews mention Pod 2 clicking noise—probably this valve; consider replacing | $5 |
| Custom mounting brackets for TECs (cold side to waterblock; hot side to radiator unless polarity is reversed) | $15 |
| 0.14 mm “no-mess” thermal pads for TEC mounts (instead of paste) | $3 |
| Black TPV tubing, secured with zip ties | $15 |
| Reservoir float valve (separate from magnetic water-level sensor? max-fill vs min-fill) | $5 |
| Waterblock housed in styrofoam insulation (isolate cold plates from hot radiator) | $3 |
| Custom ARM control board (cheap chipset) with polarity/current control for TECs; inputs for other modules (Pi/Arduino would also work) | $80 |
| Case: injection-molded plastic (unit cost after one-off mold) | $20 |
| Custom quick-connect fittings | ~$10 |
| Radiator (not custom; all parts except the ARM board available on Alibaba/AliExpress)<br>• Bulk: $15<br>• Single sample: $80 | $15–$80 |
| **Total estimated pod cost (incl. shipping, taxes, extra BOM)** | **~$300** |


### Assembling the proof of concept
The first cooler test consisted of two TEC12703 peltier coolers. They were mounted to 40x80mm watercooling block, which was then mounted to generic CPU air cooler.
CPU thermal cooling paste was applied between the surface of the watercooling block and the CPU cooler copper contact plate.

- The thermometer needs to be inserted into the water tank. Idea presented: ip67 wire grommet for simple insertion and removal. 
- Caps are difficult to fill, and it can be tough to see how much water is needed before it overflows. Solution: A jerry can fill/cap system. Use clear acrylic with acrylic cement for the water reservoir. 
