/* ###################################################################
 * K64F USB CDC Console + Stepper Motor Control
 * - Receives text over USB CDC (CsIO1 / virtual COM)
 * - "dog"  -> spin stepper CW, print "dog confirmed"
 * - "cat"  -> spin stepper CCW, print "cat confirmed"
 * ################################################################### */

#include "Cpu.h"
#include "Events.h"
#include "Pins1.h"
#include "FX1.h"
#include "GI2C1.h"
#include "WAIT1.h"
#include "MCUC1.h"
#include "CI2C1.h"
#include "CsIO1.h"
#include "IO1.h"
#include "SM1.h"
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "PDD_Includes.h"
#include "Init_Config.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>    /* for toupper */

/* ================================================================
 *  Stepper Motor Direct GPIO Control (PTD0–PTD5)
 * ================================================================ */

/* Step patterns for IN1–IN4 (PTD0–PTD3) */
static const uint8_t coilPattern[4][4] = {
    {0,1,1,0},  // 0110
    {1,0,1,0},  // 1010
    {1,0,0,1},  // 1001
    {0,1,0,1}   // 0101
};

/* crude busy-wait delay */
static void delayLoops(uint32_t loops) {
  volatile uint32_t i;
  for (i = 0; i < loops; i++) {
    __asm volatile("nop");
  }
}

/* Configure PTD0–PTD5 as outputs, PTA1/PTA2 as inputs if you still want them */
static void initMotorGPIO(void) {
  /* Enable clocks for PORTA and PORTD (already defined in PE headers) */
  SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTD_MASK;

  /* PTD0–PTD5 = GPIO outputs for motor */
  for (int pin = 0; pin <= 5; pin++) {
    PORTD_PCR(pin) = PORT_PCR_MUX(1);   /* ALT1 = GPIO */
    GPIOD_PDDR |= (1u << pin);          /* output */
  }

  /* If you still want DIP switches on PTA1/2: inputs with pull-ups */
  PORTA_PCR1 = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
  PORTA_PCR2 = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
  GPIOA_PDDR &= ~((1u << 1) | (1u << 2));

  /* Enable motor driver (PTD4, PTD5 high) */
  GPIOD_PSOR = (1u << 4) | (1u << 5);
}

/* Apply one step pattern to PTD0–PTD3 */
static void applyStepPattern(uint8_t idx) {
  /* clear PTD0–PTD3 */
  GPIOD_PCOR = 0x0Fu;

  /* set pattern bits */
  GPIOD_PSOR =
      (coilPattern[idx][0] << 0) |
      (coilPattern[idx][1] << 1) |
      (coilPattern[idx][2] << 2) |
      (coilPattern[idx][3] << 3);
}

/* Spin the motor a fixed number of steps
 * cw = true  -> CW
 * cw = false -> CCW
 */
static void spinMotorSteps(uint32_t steps, bool cw) {
  /* tweak these for speed */
  const uint32_t STEP_DELAY_LOOPS = 80000u;  /* smaller = faster */

  static uint8_t stepIndex = 0;  /* remember last position */

  for (uint32_t i = 0; i < steps; i++) {
    if (cw) {
      stepIndex = (uint8_t)((stepIndex + 1) & 0x03);  /* 0→1→2→3→0 */
    } else {
      stepIndex = (stepIndex == 0) ? 3 : (uint8_t)(stepIndex - 1);
    }
    applyStepPattern(stepIndex);
    delayLoops(STEP_DELAY_LOOPS);
  }
}

/* ================================================================
 *  Console helpers
 * ================================================================ */

/* trim CR/LF from end of line */
static void trim_eol(char *s) {
  size_t len = strlen(s);
  while (len > 0 && (s[len - 1] == '\r' || s[len - 1] == '\n')) {
    s[len - 1] = '\0';
    len--;
  }
}

/* uppercase string in-place */
static void str_to_upper(char *s) {
  while (*s) {
    *s = (char)toupper((unsigned char)*s);
    s++;
  }
}

/* ================================================================
 *  MAIN
 * ================================================================ */

int main(void) {
	PE_low_level_init();       /* Processor Expert init (clock, USB, etc) */
  initMotorGPIO();           /* our motor pin setup */

  printf("K64F console + stepper ready\r\n");
  printf("Send 'dog' or 'cat' to move 30 steps and return to origin\r\n");

  char line[64];

  /* 45-degree move = 30 steps with your setup */
  const uint32_t STEPS_45_DEG = 30;

  for (;;) {
    /* Blocking read of one line from USB CDC / console */
    if (fgets(line, sizeof(line), stdin) != NULL) {
      printf("RX raw line: \"%s\"\r\n", line);

      trim_eol(line);
      str_to_upper(line);

      printf("RX processed line: \"%s\"\r\n", line);

      /* Simple behavior:
       * CAT: 30 steps CCW, then 30 steps CW (back to origin)
       * DOG: 30 steps CW,  then 30 steps CCW (back to origin)
       */

      if (strstr(line, "CAT") != NULL) {
        printf("cat → 30 steps CCW, then back to origin\r\n");

        /* Move CCW 30 steps */
        spinMotorSteps(STEPS_45_DEG, false);   /* CCW */

        /* Return to origin: CW 30 steps */
        spinMotorSteps(STEPS_45_DEG, true);    /* CW */
      }
      else if (strstr(line, "DOG") != NULL) {
        printf("dog → 30 steps CW, then back to origin\r\n");

        /* Move CW 30 steps */
        spinMotorSteps(STEPS_45_DEG, true);    /* CW */

        /* Return to origin: CCW 30 steps */
        spinMotorSteps(STEPS_45_DEG, false);   /* CCW */
      }
      else if (strstr(line, "HELLO") != NULL) {
        printf("Hiibjubiiii\r\n");
      }
      else {
        printf("Unknown command: \"%s\"\r\n", line);
      }
    }
    /* If fgets returns NULL, just loop again */
  }

#ifdef PEX_RTOS_START
  PEX_RTOS_START();  /* (won't actually reach here with the for(;;) above) */
#endif

  for (;;){}

  /* just to satisfy the compiler */
  return 0;
}
