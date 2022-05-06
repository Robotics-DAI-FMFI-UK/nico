Simple demonstration of controlling Nico hand using a glove with bend sensors
and three gyros.

Requires three Arduino Nano each with its MPU6050, 
communicating with custom 4wcomm protocol (modified
for two separate channels).

The first arduino in the chain contains also BT module (HC-05 compatible),
and passive buzzer and 4 bend sensors on analogue pins.

On the PC side, Nico is controlled directly using Dynamixel SDK from C.

<iframe width="1280" height="720" src="https://www.youtube.com/embed/7e5xwhA107s" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

Done in 2021/2022 as part of a student project of Elisabet Delgado Mas to control Nico hand by home-made VR glove.

contact: Pavel, pavel.petrovic at uniba.sk.
