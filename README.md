This is an external button triggered PWM driven DC Motor project. This is how it works:
1) The GPIO connected to the external button is pulled high with an external pull-up resistor in idle condition - when the button is not pressed.
2) There are variables to check the state of the button, the state of the motor(is the motor running).
3) The DC motor spins with first press of the button and stops with a second press. The variables get reset after the second press of the button.
