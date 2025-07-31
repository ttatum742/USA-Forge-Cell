Furnace steps



PREHEAT REQD ON FIRST PART

1. Pick from input -- place in preheat
2. Wait for preheat
3. **Pick from preheat -- place in final\_furnace**
4. Pick from input -- Place in preheat
5. Wait for final\_furnace
6. Pick from final\_furnace -- place in stamp
7. Pick from stamp -- place in output
8. Jump to step 3



NO PREHEAT REQD

1. Pick from input -- place in final\_furnace
2. **Pick from input -- place in preheat**
3. Wait for final\_furnace
4. Pick from final\_furnace -- place in stamp
5. Pick from stamp -- place in output
6. Pick from preheat -- place in final\_furnace
7. Jump to step 2







Questions:



* Is preheating necessary, or is it purely for cycle time increase?
* ------ depends on alloy, for the most part preheating not reqd
* ------ both coils can get to final temps for lower forge temp alloys
* ------ get to 1850degf in 30sec
* What's the min/max time the part can be in the preheater?
* ------ no time limit here
* ------ longer time = better for exotics
* Does furnace auto shut-off at a set temperature?
* ------ auto shutoff is configurable
* How long will it take to get from preheated temp to final temp (roughly)? -- would like to go get another part to preheat while part gets to final temp
* ------ unsure, but temperature increase is linear for standard alloys
* **Are there >2 set points on whatever is monitoring the part temp? -- would like to have the robot start motion when the part is almost ready, not when the part is ready**
* How long do we have to stamp part? 
* ------ no longer than 10 seconds
* How long do we have to wait to eject part?
* ------ 10 seconds for small parts - 30 seconds for larger diameter





Tech onsite on Monday, Don onsite on Tuesday

