/*
 * @brief High speed ADC example
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2013
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#include "board.h"
#include <stdio.h>

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/* The default example uses DMA. To use the example in an interrutp based
   configuration, enable the following definition. */
#define USE_INTERRUPT_MODE

/* HSADC clock rate used for sampling */
#define HSADC_CLOCK_RATE (80 * 1000000)

/* Base clock lookup table used for finding best HSADC clock rate. Only the
   base clock sources in this table are used as a possible source for the
   HSASC peripheral clock before being divided. Add or remove sources as
   needed, but an exact frequency may not be available unless it's set up
   on an unused PLL or clock input. */
static const CHIP_CGU_CLKIN_T adcBaseClkSources[] = {
	CLKIN_IRC,			/* Usually 12MHz */
	CLKIN_CLKIN,		/* External clock in rate */
	CLKIN_CRYSTAL,	/* Usually 12MHz */
	CLKIN_AUDIOPLL,	/* Unknown, will be 0 if not configured */
	CLKIN_MAINPLL		/* Usually 204MHz, may be too fast to use with a divider */
};

/* Periodic sample rate in Hz */
#define SAMPLERATE (5)

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/* Last saved ADC sample for each input */
volatile uint32_t lastSample[6];

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Returns a computed clock rate based on input clock and divider
   selection */
static uint32_t getClockRate(int clkIndex, uint32_t maxCGU)
{
	uint32_t clkRate;

	clkRate = Chip_Clock_GetClockInputHz(adcBaseClkSources[clkIndex]);
	clkRate = clkRate / maxCGU;

	return clkRate;
}

/* Clock setup function for generating approximate HSADC clock. Trim this
   example function as needed to get the size down in a production system. */
static void setupClock(uint32_t rate)
{
	CHIP_CGU_IDIV_T freeDivider, bestDivider;
	CHIP_CGU_CLKIN_T divider, mappedCGUDuv, savedClkIn;
	uint32_t bestRate, testRate, maxCGU, savedMaxCGU;
	uint32_t maxCGUDiv;
	int clkIndex;

	/* The HSADC clock (sample) rate is derived from the HSADC base clock
	   divided by the HSADC clock divider (only 1 or 2). The HSADC base
	   clock can be selected from a number of internal clock sources such
	   as the main PLL1, audio PLL, external crystal rate, IRC, RTC, or a
	   CGU divider. Unless a PLL is setup for the exact rate desired, a
	   rate close to the target rate may be the closest approximation. */

	/* Determine if there are any free dividers in the CGU. Assumes an
	   unused divider is attached to CLOCKINPUT_PD. Divider A can only
	   divide 1-4, B/C/D can divide 1-16, E can divider 1-256. */
	freeDivider = CLK_IDIV_A;	/* Dividers only */
	divider = CLKIN_IDIVA;	/* CGU clock input sources */
	bestDivider = CLK_IDIV_LAST;
	while (freeDivider < CLK_IDIV_LAST) {
		CHIP_CGU_CLKIN_T clkIn;

		/* A CGUI divider that is pulled on input down is free */
		clkIn = Chip_Clock_GetDividerSource(freeDivider);
		if (clkIn == CLKINPUT_PD) {
			/* Save available divider and mapped CGU clock divider source */
			bestDivider = freeDivider;
			mappedCGUDuv = divider;
		}

		/* Try next divider */
		freeDivider++;
		divider++;
	}

	/* Determine maximum divider value per CGU divider */
	if (bestDivider != CLK_IDIV_LAST) {
		if (bestDivider == CLK_IDIV_A) {
			maxCGUDiv = 4;
		}
		else if ((bestDivider >= CLK_IDIV_B) && (bestDivider <= CLK_IDIV_D)) {
			maxCGUDiv = 16;
		}
		else {
			maxCGUDiv = 256;
		}
	}
	else {
		/* No CGU divider available */
		maxCGUDiv = 1;
	}

	/* Using the best available maximum CGU and CCU dividers, attempt to
	   find a base clock rate that will get as close as possible to the
	   target rate without going over the rate. */
	savedMaxCGU = 1;
	bestRate = 0xFFFFFFFF;
	for (clkIndex = 0; clkIndex < (sizeof(adcBaseClkSources) / sizeof(CHIP_CGU_CLKIN_T)); clkIndex++) {
		for (maxCGU = 1; maxCGU <= maxCGUDiv; maxCGU++) {
			testRate = getClockRate(clkIndex, maxCGU);
			if (rate >= testRate) {
				if ((rate - testRate) < (rate - bestRate)) {
					bestRate = testRate;
					savedClkIn = adcBaseClkSources[clkIndex];
					savedMaxCGU = maxCGU;
				}
			}
		}
	}

	/* Now to setup clocks */
	if (maxCGUDiv == 1) {
		/* CCU divider and base clock only */
		/* Select best clock as HSADC base clock */
		Chip_Clock_SetBaseClock(CLK_BASE_ADCHS, savedClkIn, true, false);
	}
	else {
		/* CCU divider with base clock routed via a CGU divider */
		Chip_Clock_SetDivider(bestDivider, savedClkIn, savedMaxCGU);
		Chip_Clock_SetBaseClock(CLK_BASE_ADCHS, mappedCGUDuv, true, false);
	}

	/* Enable ADC clock */
	Chip_Clock_EnableOpts(CLK_ADCHS, true, true, 1);
}

static void timer_setup(void)
{
	/* Enable timer 1 clock and reset it */
	Chip_TIMER_Init(LPC_TIMER1);
	Chip_RGU_TriggerReset(RGU_TIMER1_RST);
	while (Chip_RGU_InReset(RGU_TIMER1_RST)) {}

	/* Timer setup for match and interrupt at SAMPLERATE */
	Chip_TIMER_Reset(LPC_TIMER1);
	Chip_TIMER_MatchEnableInt(LPC_TIMER1, 1);
	Chip_TIMER_SetMatch(LPC_TIMER1, 1, (Chip_Clock_GetRate(CLK_MX_TIMER1) / SAMPLERATE));
	Chip_TIMER_ResetOnMatchEnable(LPC_TIMER1, 1);
	Chip_TIMER_Enable(LPC_TIMER1);

	/* Enable timer interrupt */
	NVIC_EnableIRQ(TIMER1_IRQn);
	NVIC_ClearPendingIRQ(TIMER1_IRQn);
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	Handle interrupt from 32-bit timer
 * @return	Nothing
 */
void TIMER1_IRQHandler(void)
{
	/* Clear timer interrupt */
	Chip_TIMER_ClearMatch(LPC_TIMER1, 1);

	/* Reset descriptor pointer to table 0, descriptor 0 and force SW trigger */
	Chip_HSADC_SetActiveDescriptor(LPC_ADCHS, 0, 0);
	Chip_HSADC_SWTrigger(LPC_ADCHS);
}

/**
 * @brief	Handle interrupt for HSADC peripheral
 * @return	Nothing
 * @note	The HSADC IRQ handler is called SPIFI_ADCHS_IRQHandler() on the
 * M0 cores. For the M4 core, it's called ADCHS_IRQHandler().
 */
void ADCHS_IRQHandler(void)
{
	uint32_t sts, data, sample, ch;
	static bool on;
	int prn_delay = 0;

	/* Toggle LED on each sample interrupt */
	on = !on;
	Board_LED_Set(0, on);

	/* Get threshold interrupt status on group 1 and toggle on any crossing */
	sts = Chip_HSADC_GetIntStatus(LPC_ADCHS, 1) & Chip_HSADC_GetEnabledInts(LPC_ADCHS, 1);
	if (sts & (HSADC_INT1_THCMP_DCROSS(0) | HSADC_INT1_THCMP_DCROSS(1) |
			   HSADC_INT1_THCMP_UCROSS(2) | HSADC_INT1_THCMP_UCROSS(3) |
			   HSADC_INT1_THCMP_DCROSS(4) | HSADC_INT1_THCMP_UCROSS(5))) {
		Board_LED_Set(1, true);
	}
	else {
		Board_LED_Set(1, false);
	}

	/* Clear threshold interrupts statuses */
	Chip_HSADC_ClearIntStatus(LPC_ADCHS, 1, sts);

	/* Pull data from FIFO */
	data = Chip_HSADC_GetFIFO(LPC_ADCHS);
	while (!(data & HSADC_FIFO_EMPTY)) {
		/* Pull sample data and channel from FIFO data */
		sample = HSADC_FIFO_SAMPLE(data);
		ch = HSADC_FIFO_CHAN_ID(data);

		/* We don't really have anythng to do with the data,
		   so just save it */
		lastSample[ch] = sample;

		/* Next sample */
		data = Chip_HSADC_GetFIFO(LPC_ADCHS);
	}

	/* Get ADC interrupt status on group 0 */
	sts = Chip_HSADC_GetIntStatus(LPC_ADCHS, 0) & Chip_HSADC_GetEnabledInts(LPC_ADCHS, 0);

	/* Set LED 2 (if it exists) on an error */
	if (sts & (HSADC_INT0_FIFO_OVERFLOW | HSADC_INT0_DSCR_ERROR)) {
		Board_LED_Set(2, true);
	}
	else {
		Board_LED_Set(2, false);
	}

	/* Clear group 0 interrupt statuses */
	Chip_HSADC_ClearIntStatus(LPC_ADCHS, 0, sts);
}

/**
 * @brief	main routine for HSADC example
 * @return	Function should not exit
 */
int main(void)
{
	uint32_t freqHSADC = 0;
	uint32_t stored_last_0 = 0;

	SystemCoreClockUpdate();
	Board_Init();

	/* Setting up the HSADC clock is more complex than other peripherals.
	   The HSADC clock is driven directly from the CGU/CCU and has limited
	   source and divider options. Because the HSADC clocking is entirely
	   handled outside the HSADC peripheral, example code for setting up
	   the CGU/CCU to get a rough HSADC clock rate is included in this
	   example. */
	setupClock(HSADC_CLOCK_RATE);

	/* Initialize HSADC */
	Chip_HSADC_Init(LPC_ADCHS);

	/* Show the actual HSADC clock rate */
	freqHSADC = Chip_HSADC_GetBaseClockRate(LPC_ADCHS);
	DEBUGOUT("HSADC sampling rate = %dKHz\r\n", freqHSADC / 1000);

	/* Setup FIFO trip points for interrupt/DMA to 8 samples, no packing */
	Chip_HSADC_SetupFIFO(LPC_ADCHS, 8, false);

	/* Software trigger only, 0x90 recovery clocks, add channel IF to FIFO entry */
	Chip_HSADC_ConfigureTrigger(LPC_ADCHS, HSADC_CONFIG_TRIGGER_SW,
								HSADC_CONFIG_TRIGGER_RISEEXT, HSADC_CONFIG_TRIGGER_NOEXTSYNC,
								HSADC_CHANNEL_ID_EN_ADD, 0x90);

	/* Select both positive and negative DC biasing for input 3 */
	//Chip_HSADC_SetACDCBias(LPC_ADCHS, 3, HSADC_CHANNEL_DCBIAS, HSADC_CHANNEL_DCBIAS);
	Chip_HSADC_SetACDCBias(LPC_ADCHS, 0, HSADC_CHANNEL_DCBIAS, HSADC_CHANNEL_NODCBIAS);

	/* Set low A threshold to 10% and high A threshold to 90% */
	Chip_HSADC_SetThrLowValue(LPC_ADCHS, 0, ((HSADC_MAX_SAMPLEVAL * 1) / 10));
	Chip_HSADC_SetThrHighValue(LPC_ADCHS, 0, ((HSADC_MAX_SAMPLEVAL * 9) / 10));

	/* Set low B threshold to 40% and high B threshold to 60% */
	Chip_HSADC_SetThrLowValue(LPC_ADCHS, 1, ((HSADC_MAX_SAMPLEVAL * 4) / 10));
	Chip_HSADC_SetThrHighValue(LPC_ADCHS, 1, ((HSADC_MAX_SAMPLEVAL * 6) / 10));

	/* Setup data format for 2's complement and update clock settings. This function
	   should be called whenever a clock change is made to the HSADC */
	Chip_HSADC_SetPowerSpeed(LPC_ADCHS, true);

	/* Enable HSADC power */
	Chip_HSADC_EnablePower(LPC_ADCHS);

	/* Setup HSADC table 0 descriptors */
	/* Descriptor entries are mapped as follows */
	/* 0 : mapped to input 0, branch to next descriptor after sample, match time
	   is 0x90 clocks for the initial sample (must be greater than or equal to
	     recovery clocks for auto power-up), test against threshold A */
	Chip_HSADC_SetupDescEntry(LPC_ADCHS, 0, 0, (HSADC_DESC_CH(0) |
												HSADC_DESC_BRANCH_NEXT | HSADC_DESC_MATCH(0x95) | HSADC_DESC_THRESH_A |
												HSADC_DESC_RESET_TIMER));
	/* 1 : mapped to input 0, branch to next descriptor after sample, match time
	   is 1, test against threshold A */
	Chip_HSADC_SetupDescEntry(LPC_ADCHS, 0, 1, (HSADC_DESC_CH(0) |
												HSADC_DESC_BRANCH_NEXT | HSADC_DESC_MATCH(1) |
												HSADC_DESC_THRESH_A | HSADC_DESC_RESET_TIMER));
	/* 2-3 : mapped to input 1, branch to next descriptor after sample, match time
	   is 1 test against threshold A */
	Chip_HSADC_SetupDescEntry(LPC_ADCHS, 0, 2, (HSADC_DESC_CH(1) |
												HSADC_DESC_BRANCH_NEXT | HSADC_DESC_MATCH(1) |
												HSADC_DESC_THRESH_A | HSADC_DESC_RESET_TIMER));
	Chip_HSADC_SetupDescEntry(LPC_ADCHS, 0, 3, (HSADC_DESC_CH(1) |
												HSADC_DESC_BRANCH_NEXT | HSADC_DESC_MATCH(1) |
												HSADC_DESC_THRESH_A | HSADC_DESC_RESET_TIMER));
	/* 4-5 : mapped to input 2, branch to next descriptor after sample, match time
	   is 1 test against threshold A */
	Chip_HSADC_SetupDescEntry(LPC_ADCHS, 0, 4, (HSADC_DESC_CH(2) |
												HSADC_DESC_BRANCH_NEXT | HSADC_DESC_MATCH(1) |
												HSADC_DESC_THRESH_A | HSADC_DESC_RESET_TIMER));
	Chip_HSADC_SetupDescEntry(LPC_ADCHS, 0, 5, (HSADC_DESC_CH(2) |
												HSADC_DESC_BRANCH_NEXT | HSADC_DESC_MATCH(1) |
												HSADC_DESC_THRESH_A | HSADC_DESC_RESET_TIMER));
	/* 6 : mapped to input 3, branch to next descriptor after sample, match time
	   is 1 test against threshold A */
	Chip_HSADC_SetupDescEntry(LPC_ADCHS, 0, 6, (HSADC_DESC_CH(3) |
												HSADC_DESC_BRANCH_NEXT | HSADC_DESC_MATCH(1) |
												HSADC_DESC_THRESH_A | HSADC_DESC_RESET_TIMER));
	/* 7 : mapped to input 4, branch to next descriptor after sample, match time
	   is 1, test against threshold B, halt after conversion with interrupt, power
	     down after conversion */
	Chip_HSADC_SetupDescEntry(LPC_ADCHS, 0, 7, (HSADC_DESC_CH(4) |
												HSADC_DESC_HALT | HSADC_DESC_INT | HSADC_DESC_POWERDOWN |
												HSADC_DESC_MATCH(1) | HSADC_DESC_THRESH_B | HSADC_DESC_RESET_TIMER));

	/* Setup HSADC interrupts on group 0 - FIFO trip (full), FIFO overrun
	   error, and descriptor statuses */
	Chip_HSADC_EnableInts(LPC_ADCHS, 0, (HSADC_INT0_FIFO_FULL | HSADC_INT0_DSCR_DONE |
										 HSADC_INT0_FIFO_OVERFLOW | HSADC_INT0_DSCR_ERROR));

	/* Setup HSADC threshold interrupts on group 1 */
	Chip_HSADC_EnableInts(LPC_ADCHS, 1,
						  (HSADC_INT1_THCMP_DCROSS(0) | HSADC_INT1_THCMP_DCROSS(1) |/* Inputs 0 and 1 below threshold below range */
						   HSADC_INT1_THCMP_UCROSS(2) | HSADC_INT1_THCMP_UCROSS(3) |/* Inputs 2 and 3 above threshold range */
						   HSADC_INT1_THCMP_DCROSS(4) |	/* Inputs 4 downward threshold crossing detect */
						   HSADC_INT1_THCMP_UCROSS(5)));/* Inputs 5 upward threshold crossing detect */

	/* Enable HSADC interrupts in NVIC */
	NVIC_EnableIRQ(ADCHS_IRQn);

	/* Update descriptor tables - needed after updating any descriptors */
	Chip_HSADC_UpdateDescTable(LPC_ADCHS, 0);

	/* Setup periodic timer to perform software triggering */
	timer_setup();

	/* Sleep while waiting for conversions */
	while (1) {
		__WFI();
		if (lastSample[0] >> 6 != stored_last_0 >> 6) {
			DEBUGOUT("ADC VALUE[0]: %08x\r\n", lastSample[0]);
			stored_last_0 = lastSample[0];
		}
	}

	/* Shutdown HSADC when done */
	Chip_HSADC_DeInit(LPC_ADCHS);
}






