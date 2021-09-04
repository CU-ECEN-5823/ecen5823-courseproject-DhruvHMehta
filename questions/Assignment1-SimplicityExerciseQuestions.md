Please include your answers to the questions below with your submission, entering into the space below each question
See [Mastering Markdown](https://guides.github.com/features/mastering-markdown/) for github markdown formatting if desired.

**1. How much current does the system draw (instantaneous measurement) when a single LED is on with the GPIO pin set to StrongAlternateStrong?**
   Answer: 5.16mA


**2. How much current does the system draw (instantaneous measurement) when a single LED is on with the GPIO pin set to WeakAlternateWeak?**
   Answer: 5.23mA


**3. Is there a meaningful difference in current between the answers for question 1 and 2? Please explain your answer, 
referencing the [Mainboard Schematic](https://www.silabs.com/documents/public/schematic-files/WSTK-Main-BRD4001A-A01-schematic.pdf) and [AEM Accuracy](https://www.silabs.com/documents/login/user-guides/ug279-brd4104a-user-guide.pdf) section of the user's guide where appropriate. Extra credit is avilable for this question and depends on your answer.**
   Answer: 
   
   There is no meaningful difference between the current measurements for LED0 with StrongAlternateStrong and WeakAlternateWeak setting. Based on the values obtained from the Energy Profile, the system consumes similar amounts of current at about 3.3V.
   The mainboard schematic shows a resistor of 3K ohm in series with the LED. The voltage drop across the LED was measured to be 1.8V. Hence, the current can be calculated as:  
   I = (Vsource - VLED)/R = (3.3 - 1.8)/3000 = 0.5mA.  This was also practically measured by measuring the voltage drop across the resistor (1.44V). This means the current is 1.44/3000 = 0.48mA.
   As per the datasheet, StrongAlternateStrong can drive upto 10mA and WeakAlternateWeak can drive upto 1mA. Since the current required by this circuit is less than both these values, there is no difference observed.  


**4. With the WeakAlternateWeak drive strength setting, what is the average current for 1 complete on-off cycle for 1 LED with an on-off duty cycle of 50% (approximately 1 sec on, 1 sec off)?**
   Answer: 4.85mA


**5. With the WeakAlternateWeak drive strength setting, what is the average current for 1 complete on-off cycle for 2 LEDs (both on at the time same and both off at the same time) with an on-off duty cycle of 50% (approximately 1 sec on, 1 sec off)?**
   Answer: 5.03mA


