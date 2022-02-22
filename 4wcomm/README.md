# 4wirecomm

4-wire bi-directional timer-less fast background communication for two Arduinos with buffer based on pin-change interrupt only

Motivation and explanation:

HW communication capabilities of Arduino (UART, I2C, SPI...) are limited and often the pins are used for other purpose.
SW simulation libraries such as SoftwareSerial are very fragile when using with other hardware - such as Gyro, servomotors, etc.
This program implements a bi-directional very fast communication using 4 cables between two Arduinos only using pin change interrupt.
That means, the timing is not an issue - if the system is busy serving gyro, or motor, or anything else, this communication 
can wait as much as needed for the pin change interrupt to finally occur, and continue from that point. For that end it requires
8 stages to transmit a single bit. This however allows fully asynchronous timer-less communication where all transmissions are
acknowledges by the communicating peer. This software is a public domain, use it for whatever you find useful.
As of Feb 2022, it's just been finished and not tested heavily. More thinking needs to be done into resolving conflicts
when both sides start communicating. I think it is already taken care of sufficiently, but I am not 100% sure. And also
the initialization might need some improvements, currently, the init needs to be called twice as we are getting some spurious
pin change interrupts when the bootloader starts the program or something of that sort. But double-init solves the issue fine.


signals: 

    A1 -> A2: A, C
    A2 -> A1: B, D


meaning:

    A: control A -> B
    B: response control A -> B, data B -> A
    C: data A -> B, response control B -> A
    D: control B -> A


connections:

Arduino1:

    A: D6 (output)
    B: D5 (input)
    C: D7 (output)
    D: D8 (input)

Arduino2:

    A: D3 (input)
    B: D4 (output)
    C: D7 (input)
    D: D8 (output)


Communication states (all this is just 1 bit transfer)

    1. IDLE
    2. SEND_REQUEST_SENT
    3. SEND_REQUEST_CONFIRMED
    4. SEND_REQUEST_CONF_RECEIVED
    5. GETTING_READY_FOR_DATA
    6. DATA_TRANSMISSION
    7. DATA_TRANS_CONFIRMED
    8. DATA_TRANS_CONF_RECEIVED


Communication protocol (time diagrams):

Arduino1 sending data to Arduino2:

    (1=--, 0=__, D=data)
    
    A --|__|__|--|--|__|__|--|--
        |  |  |  |  |  |  |  |  
    B --|--|__|__|--|--|__|__|--
        |  |  |  |  |  |  |  |
    C --|--|--|--|-D|DD|D-|--|--
        |  |  |  |  |  |  |  |
    D --|--|--|--|--|--|--|--|--
    
      1  2  3  4  5  6  7  8  1  (state)
    
Arduino2 sending data to Arduino1:

    A --|--|--|--|--|--|--|--|--
        |  |  |  |  |  |  |  |  
    B --|--|--|--|-D|DD|D-|--|--
        |  |  |  |  |  |  |  |
    C --|--|__|__|--|--|__|__|--
        |  |  |  |  |  |  |  |
    D --|__|__|--|--|__|__|--|--
    
      1  2  3  4  5  6  7  8  1  (state)

Both Arduinos decide to send data at the same time:

    A --|__|__|__|
        |  |  |  |
    B --|--|--|__|
        |  |  |  | continue from state 4 A1 to A2
    C --|--|--|--|  (A1 has priority, A1 ignores A2, A2 gives up when conflict detected)
        |  |  |  |
    D --|-_|__|--|
    
      1  2  3  4  

Pavel, pavel.petrovic at uniba.sk, February 2022
