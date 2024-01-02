#include "am2302.h"

//#define DEBUG_PRINT

#define PINB_INPUT_MASK 0b00001000
#define PIND_INPUT_MASK 0b00001000
#define PAYLOAD_BIT_LENGTH 40

volatile uint8_t pinb_reg_state = 0, pind_reg_state = 0;
volatile bool calculate_data = false;
volatile bool get_data = false;

#define STEP_IDLE             0
#define STEP_START_SIGNAL     bit(0)
#define STEP_WAIT_FOR_SENSOR  bit(1)
#define STEP_SENSOR_READY     bit(2)
#define STEP_DATA_TRANSFER    bit(3)
#define STEP_COMPLETE         bit(4)

#define MS_TO_TICKS(MS) = (MS / 1000) / 0.0000016

volatile struct {
  uint8_t transfer_step = 0;
  uint8_t data_bit_count = 0;
  uint8_t error = 0;
  uint8_t data[5];
  uint16_t raw_data[40];
} protocol;

void setup()
{
  /* Configure I/O */
  /* OUTPUTS */
  DDRB |= (1 << PB3);
  DDRD |= (1 << PD3);

  /* Initial State(s) */
  PORTB |= (1 << PB3);
  PORTD |= (1 << PD3);

  /* Pin Change Interrupt(s) */
  PCICR |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT19);

  /* Data Timer
    - All outputs disconnected
    - CTC mode with OCR1A as top
    - Sys Clock / 256 = ~16uS per tick. But is initially disabled
    - Output Compare A match interrupt enabled
  */
  TCCR1A &= ~((1 << COM1A1) | (1 << COM1A0) | (1 << COM1B1) | (1 << COM1B0) | (1 << WGM11) | (1 << WGM10));
  TCCR1B &= ~((1 << ICNC1) | (1 << ICES1) | (1 << WGM13) | (1 << CS12) | (1 << CS11) | (1 << CS10));
  TCCR1B |= (1 << WGM12);
  TCNT1 = 0;
  OCR1A = 500;
  TIMSK1 |= (1 << OCIE1A);

  Serial.begin(115200);
  Serial.println(F("am2302 Start!"));

  sei();
}

void loop()
{
  if (protocol.error != 0)
  {
    switch (protocol.error)
    {
      case STEP_WAIT_FOR_SENSOR:
        Serial.print(F("Error: "));
        print_step(protocol.error, true);
        break;
      case STEP_DATA_TRANSFER:
        Serial.print(F("Error: "));
        print_step(protocol.error, true);
        Serial.print(F("Bit #: "));
        Serial.println(protocol.data_bit_count);
        break;
    }
    protocol.error = 0;
  } else
  {
    if (get_data)
    {
      protocol.transfer_step |= STEP_START_SIGNAL;

      /* Generate start signal */
      DDRD |= (1 << PD3);
      PORTD &= ~(1 << PD3); // Drive SDA LOW to initiate transfer

      /* Hold SDA LOW for ~2mS */
      OCR1A = 500;

      /* Reset and start timer */
      TCNT1 = 0;
      TCCR1B |= (1 << CS11) | (1 << CS10);

      get_data = false;
    }
  }

  if (calculate_data)
  {
    /* Determine bit value */
    uint8_t byte_index = 0;
    uint8_t shift_count = 0;

    for (uint8_t i = 0; i < PAYLOAD_BIT_LENGTH; i++)
    {
      byte_index = i / 8;
      shift_count = 7 - (i % 8);

      if (protocol.raw_data[i] <= 22)// Time less than 100 uS is a 0
      {
        protocol.data[byte_index] &= ~(1 << shift_count);
      } else // Time greater than 100 uS is a 1
      {
        protocol.data[byte_index] |= (1 << shift_count);
      }
    }

#ifdef DEBUG_PRINT
    Serial.print(F("Raw Data: "));
    for (uint8_t i = 0; i <= 4; i++)
    {
      Serial.print(protocol.data[i], BIN);
      Serial.print(F(" "));
    }
    Serial.println(F(""));
    Serial.print(F("Checksum (data/calc): "));
    Serial.print(String(protocol.data[4]) + "/" + uint8_t(protocol.data[0] + protocol.data[1] + protocol.data[2] + protocol.data[3]) + "\n");
#endif

    Serial.print(F("Relative Humidity: "));
    float rh = (uint16_t(protocol.data[0] << 8) | protocol.data[1]) / 10.0;
    Serial.print(String(rh) + " %  ");

    Serial.print(F("Temperature: "));
    float t = (uint16_t(protocol.data[2] << 8) | protocol.data[3]) / 10.0;
    Serial.print(String(t) + "°C  ");
    Serial.println(String((t * 1.8) + 32) + "°F  ");

    /* Reset Data */
    protocol.data[0] = protocol.data[1] = protocol.data[2] = protocol.data[3] = protocol.data[4] = 0;

    calculate_data = false;
  }

  delay(2000);
  get_data = true;
}

void serialEvent(void)
{
  String new_command = Serial.readStringUntil('\n');  // Read until newline char. The newline char is truncated.

  switch (new_command.charAt(0))
  {
    case 'A':
      switch (new_command.charAt(1))
      {
        case '0':
          get_data = true;
          break;
      }
      break;
  }
}

ISR(PCINT2_vect)
{
  uint8_t reg = PIND; // Capture I/O register state
  uint8_t pins = reg & PIND_INPUT_MASK; // Mask input pins
  pins ^= pind_reg_state; // Determine which input pin(s) changed. ie. triggered the Pin Change Interrupt
  pind_reg_state = reg & PIND_INPUT_MASK; // Store new state of I/O register for next time

  /* Determine if pin toggled */
  if (pins & (1 << PD3))
  {
    /* Determine polarity of pin */
    if (reg & (1 << PD3)) // Pin pulled HIGH
    {
      // Do nothing
    }
    else // Pin pulled LOW
    {
      switch (protocol.transfer_step)
      {
        case STEP_WAIT_FOR_SENSOR:
          /* Stop timer */
          TCCR1B &= ~((1 << ICNC1) | (1 << ICES1) | (1 << WGM13) | (1 << CS12) | (1 << CS11) | (1 << CS10));

          /* Keep SDA pin as input and enable pull-up so AM2302 can have full control of SDA line */
          PORTD &= (1 << PD3);

          /* Update protocol step */
          protocol.transfer_step &= ~STEP_WAIT_FOR_SENSOR;
          protocol.transfer_step |= STEP_SENSOR_READY;
          break;

        case STEP_SENSOR_READY:
          /* Update protocol step */
          protocol.transfer_step &= ~STEP_SENSOR_READY;
          protocol.transfer_step |= STEP_DATA_TRANSFER;

          /* For the first data bit falling edge, only start timer */
          TCNT1 = 0;
          TCCR1B |= (1 << CS11) | (1 << CS10); // Start timer
          break;

        case STEP_DATA_TRANSFER:
          if (protocol.data_bit_count >= (PAYLOAD_BIT_LENGTH - 1))
          {
            /* Transfer complete. Grab last data bit, clean up timer and control signals, then exit. */

            /* Stop timer */
            TCCR1B &= ~((1 << ICNC1) | (1 << ICES1) | (1 << WGM13) | (1 << CS12) | (1 << CS11) | (1 << CS10));

            /* Capture time stamp */
            protocol.raw_data[protocol.data_bit_count] = TCNT1;

            protocol.transfer_step = STEP_IDLE;
            protocol.data_bit_count = 0;

            calculate_data = true;
          }

          else
          {
            /* Stop timer */
            TCCR1B &= ~((1 << ICNC1) | (1 << ICES1) | (1 << WGM13) | (1 << CS12) | (1 << CS11) | (1 << CS10));

            /* Capture time stamp */
            protocol.raw_data[protocol.data_bit_count] = TCNT1;

            /* Restart Timer */
            TCNT1 = 0;
            TCCR1B |= (1 << CS11) | (1 << CS10); // Start timer

            protocol.data_bit_count++;
          }

          break;
      }
    }
  }
}

ISR(TIMER1_COMPA_vect)
{
  switch (protocol.transfer_step)
  {
    case STEP_START_SIGNAL:
      /* Stop timer */
      TCCR1B &= ~((1 << ICNC1) | (1 << ICES1) | (1 << WGM13) | (1 << CS12) | (1 << CS11) | (1 << CS10));

      /* Pull up SDA */
      PORTD |= (1 << PD3);
      DDRD &= ~(1 << PD3);

      /* Update protocol step. */
      protocol.transfer_step &= ~STEP_START_SIGNAL;
      protocol.transfer_step |= STEP_WAIT_FOR_SENSOR;

      /* Wait for sensor response (~20-40uS, Time-out after 1 millisecond)
        If sensor responds, it will trigger the PCINT ISR associated with the SDA pin.
        That ISR will stop the timer and move program to the next step of the protocol.
      */
      OCR1A = 125;
      /* Start Timer */
      TCNT1 = 0;
      TCCR1B |= (1 << CS11) | (1 << CS10); // Start timer

      break;
    case STEP_WAIT_FOR_SENSOR:
      /* If this case is executed, that means the AM2302 did not respond to the MCU's start signal.
        Send error to host.
      */
      protocol.error = protocol.transfer_step; // Capture step value when error occurred.
      protocol.transfer_step = STEP_IDLE;
      break;

    case STEP_DATA_TRANSFER:
      /* If this case is ever executed, data transfer failed because a bit was not delivered within
          the specified time. Capture step and send error to host.
      */
      protocol.error = protocol.transfer_step;
      protocol.transfer_step = STEP_IDLE;
      break;
  }
}

void print_step(uint8_t step_, bool error)
{
  if (!error)
    Serial.print(F("Step: "));

  switch (step_)
  {
    case STEP_IDLE:
      Serial.println(F("IDLE"));
      break;
    case STEP_START_SIGNAL:
      Serial.println(F("START"));
      break;
    case STEP_WAIT_FOR_SENSOR:
      Serial.println(F("WAIT"));
      break;
    case STEP_SENSOR_READY:
      Serial.println(F("READY"));
      break;
    case STEP_DATA_TRANSFER:
      Serial.println(F("TRANSFER"));
      break;
    case STEP_COMPLETE:
      Serial.println(F("COMPLETE"));
      break;
  }
}
