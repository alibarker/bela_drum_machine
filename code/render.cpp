/*
 * assignment2_drums
 *
 * Second assignment for ECS732 RTDSP, to create a sequencer-based
 * drum machine which plays sampled drum sounds in loops.
 *
 * This code runs on BeagleBone Black with the Bela/BeagleRT environment.
 *
 * 2016 
 * Becky Stewart
 *
 * 2015
 * Andrew McPherson and Victor Zappi
 * Queen Mary University of London
 */


#include <Utilities.h>
#include <BeagleRT.h>
#include <rtdk.h>
#include <cmath>
#include "drums.h"

/* Pin numbers */

int buttonPin1 =  P8_07;

/* Global State Variables */

int previousButton1State = 0;

/* Variables which are given to you: */

/* Drum samples are pre-loaded in these buffers. Length of each
 * buffer is given in gDrumSampleBufferLengths.
 */
extern float *gDrumSampleBuffers[NUMBER_OF_DRUMS];
extern int gDrumSampleBufferLengths[NUMBER_OF_DRUMS];


int gIsPlaying = 0;			/* Whether we should play or not. Implement this in Step 4b. */

/* Read pointer into the current drum sample buffer.
 *
 * TODO (step 3): you will replace this with two arrays, one
 * holding each read pointer, the other saying which buffer
 * each read pointer corresponds to.
 */
int gReadPointer = 0;

/* Patterns indicate which drum(s) should play on which beat.
 * Each element of gPatterns is an array, whose length is given
 * by gPatternLengths.
 */
extern int *gPatterns[NUMBER_OF_PATTERNS];
extern int gPatternLengths[NUMBER_OF_PATTERNS];

/* These variables indicate which pattern we're playing, and
 * where within the pattern we currently are. Used in Step 4c.
 */
int gCurrentPattern = 0;
int gCurrentIndexInPattern = 0;

/* Triggers from buttons (step 2 etc.). Read these here and
 * do something if they are nonzero (resetting them when done). */
int gTriggerButton1;
int gTriggerButton2;

/* This variable holds the interval between events in **milliseconds**
 * To use it (Step 4a), you will need to work out how many samples
 * it corresponds to.
 */
int gEventIntervalMilliseconds = 250;

/* This variable indicates whether samples should be triggered or
 * not. It is used in Step 4b, and should be set in gpio.cpp.
 */
extern int gIsPlaying;

/* This indicates whether we should play the samples backwards.
 */
int gPlaysBackwards = 0;

/* For bonus step only: these variables help implement a fill
 * (temporary pattern) which is triggered by tapping the board.
 */
int gShouldPlayFill = 0;
int gPreviousPattern = 0;

/* TODO: Declare any further global variables you need here */

// setup() is called once before the audio rendering starts.
// Use it to perform any initialisation and allocation which is dependent
// on the period size or sample rate.
//
// userData holds an opaque pointer to a data structure that was passed
// in from the call to initAudio().
//
// Return true on success; returning false halts the program.

bool setup(BeagleRTContext *context, void *userData)
{
	pinModeFrame(context, 0, buttonPin1, INPUT);
	return true;
}

// render() is called regularly at the highest priority by the audio engine.
// Input and output are given from the audio hardware and the other
// ADCs and DACs (if available). If only audio is available, numMatrixFrames
// will be 0.

void render(BeagleRTContext *context, void *userData)
{
	/* Step 2: use gReadPointer to play a drum sound */

	for (int n = 0; n < context->digitalFrames; n++)
	{
		float output = 0.0;
		// Check button 1 state and trigger if just pressed.
		int button1State = digitalReadFrame(context, n, buttonPin1);
		if (button1State == 1 && previousButton1State == 0)
		{
			gTriggerButton1 = 1;
			gReadPointer = 0;
		}
		previousButton1State = button1State;


		// If drum triggered read through samples and add to output buffer
		if (gTriggerButton1)
		{
			output += gDrumSampleBuffers[0][gReadPointer];
			gReadPointer++;

			if (gReadPointer >= gDrumSampleBufferLengths[0])
			{
				gReadPointer = 0;
				gTriggerButton1 = 0;

			}
		}

		// Write to output buffers
		audioWriteFrame(context, n, 0, output);
		audioWriteFrame(context, n, 1, output);

	}

	/* Step 3: use multiple read pointers to play multiple drums */

	/* Step 4: count samples and decide when to trigger the next event */
}

/* Start playing a particular drum sound given by drumIndex.
 */
void startPlayingDrum(int drumIndex) {
	/* TODO in Steps 3a and 3b */
}

/* Start playing the next event in the pattern */
void startNextEvent() {
	/* TODO in Step 4 */
}

/* Returns whether the given event contains the given drum sound */
int eventContainsDrum(int event, int drum) {
	if(event & (1 << drum))
		return 1;
	return 0;
}

// cleanup_render() is called once at the end, after the audio has stopped.
// Release any resources that were allocated in initialise_render().

void cleanup(BeagleRTContext *context, void *userData)
{

}
