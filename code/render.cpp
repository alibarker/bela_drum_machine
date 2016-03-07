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
 int potPin = 0;
 int ledPin = P8_08;
 int xPin = 1;
 int yPin = 2;
 int zPin = 3;

/* Global State Variables */

 int previousButton1State = 0;
 int previousButton2State = 0;
 int gSampleCount = 0;
 int gCountForSensor = 0;
 int gLedState = 0;

 float gXCalibrate;
 float gYCalibrate;
 float gZCalibrate;
 int gStartup = 10;
int gXState = 0; // +1 = Positive
int gYState = 0; // 0 = Zero
int gZState = 0; // -1 = Negative
int gOrientation;

enum {
	flat = 0,
	left,
	right,
	front,
	back
};


float gHighThreshold = 0.07;
float gLowThreshold = 0.04;

int gNumAudioFramesPerAnalog;


// Filter variables
float gA1, gA2, gB0, gB1, gB2;
float gX1, gX2, gY1, gY2;

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

#define NUMBER_OF_READPOINTERS 16

 int gReadPointers[NUMBER_OF_READPOINTERS];
 int gDrumBufferForReadPointer[NUMBER_OF_READPOINTERS];

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
 int gFillPattern = 5;

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


 void setFilterCoefficients(float sampleRate, float cutoff)
 {

	float wc = 2 * M_PI * cutoff; // cutoff frequency in radians
	float Q = sqrt(2) / 2; // Q of filter

	float wca = tanf(wc * 0.5 / sampleRate); // warped analogue frequency

	// set coeffiecients for butterworth filters high pass):

		float a0 = 1 + wca/Q + pow(wca,2); 

		gB0 = 1 / a0;
		gB1 = -2 / a0;
		gB2 = 1 / a0;

		gA1 = (-2 + 2 * pow(wca,2) ) / a0;
		gA2 = (1 - wca/Q + pow(wca,2) ) / a0;

		rt_printf("B0: %f\tB1:%f\tB2: %f\tA1:%f\tA2: %f", gB0, gB1, gB2, gA1, gA2);
		
		gX1 = gX2 = gY1 = gY2 = 0;
	
}


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
 	pinModeFrame(context, 0, ledPin, OUTPUT);

 	gNumAudioFramesPerAnalog = context->audioFrames / context->analogFrames;
 	for (int i = 0; i < NUMBER_OF_READPOINTERS; i++)
 	{
 		gDrumBufferForReadPointer[i] = -1;
 	}

 	setFilterCoefficients(context->audioSampleRate/2.0, 150.0);

 	gShouldPlayFill = 0;

 	return true;
 }

// render() is called regularly at the highest priority by the audio engine.
// Input and output are given from the audio hardware and the other
// ADCs and DACs (if available). If only audio is available, numMatrixFrames
// will be 0.

void updateOrientation(float x, float y, float z)
{

	if (gXState == 0 && fabs(x) > gHighThreshold){
		gXState = x/fabs(x);
	} else if (gXState != 0 && fabs(x) < gLowThreshold) {
		gXState = 0;
	}


	if (gYState == 0 && fabs(y) > gHighThreshold){
		gYState = y/fabs(y);
	} else if (gYState != 0 && fabs(y) < gLowThreshold) {
		gYState = 0;
	}

	if (gZState == 0 && fabs(z) > gHighThreshold){
		gZState = z/fabs(z);
	} else if (gZState != 0 && fabs(z) < gLowThreshold) {
		gZState = 0;
	}

	if (gXState == 0 && gYState == 0 && gZState == 1){
		gOrientation = flat;
		gPlaysBackwards = 0;

	}
	if (gXState == -1 && gYState == 0 && gZState == 0){
		gOrientation = left;
		gPlaysBackwards = 0;
	}
	if (gXState == 1 && gYState == 0 && gZState == 0){
		gOrientation = right;
		gPlaysBackwards = 0;
	}
	if (gXState == 0 && gYState == -1 && gZState == 0){
		gOrientation = front;
		gPlaysBackwards = 0;
	}
	if (gXState == 0 && gYState == 1 && gZState == 0){
		gOrientation = back;
		gPlaysBackwards = 0;
	}
	if (gXState == 0 && gYState == 0 && gZState == -1){
		gPlaysBackwards = 1;
	}

}

void render(BeagleRTContext *context, void *userData)
{
	/* Step 2: use gReadPointer to play a drum sound */


	if (gStartup > 0)
	{

		gXCalibrate = 0.42;
		gYCalibrate = 0.42;
		gZCalibrate = 0.42;

		gStartup--;

		gShouldPlayFill = 0;

		rt_printf("Callibration: X: %f\tY: %f\tZ: %f\n", gXCalibrate, gYCalibrate, gZCalibrate);
	} 


	for (int n = 0; n < context->digitalFrames; n++)
	{
		// initialize output
		float output = 0.0;

		// check button state and change gIsPlaying if just pressed
		int button1State = digitalReadFrame(context, n, buttonPin1);
		if (button1State == 0 && previousButton1State == 1)
		{
			if (gIsPlaying == 0) {
				gIsPlaying = 1;
				gSampleCount = 0;
			} else {
				gIsPlaying = 0;
			}
		}
		previousButton1State = button1State;

		float gEventIntervalMilliseconds = map(analogReadFrame(context, n/gNumAudioFramesPerAnalog, potPin), 0, 0.829, 50, 1000);

		gCountForSensor++;


		float x = analogReadFrame(context, n/gNumAudioFramesPerAnalog, xPin) - gXCalibrate;
		float y = analogReadFrame(context, n/gNumAudioFramesPerAnalog, yPin) - gYCalibrate;
		float z = analogReadFrame(context, n/gNumAudioFramesPerAnalog, zPin) - gZCalibrate;

		float accelerationMagnitude = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));

 			// Filter out gravity

		float filteredAcceleration = gB0 * accelerationMagnitude + gB1 * gX1 + gB2 * gX2 
		- gA1 * gY1 - gA2 * gY2;

		gX2 = gX1;
		gX1 = accelerationMagnitude;
		gY2 = gY1;
		gY1 = filteredAcceleration;


		if (filteredAcceleration > 0.2 && gShouldPlayFill == 0) {
			rt_printf("Filtered acceleration: %f\n", filteredAcceleration);
			gShouldPlayFill = 1;
			if (gCurrentPattern != gFillPattern)
				gPreviousPattern = gCurrentPattern;
			gCurrentIndexInPattern = 0;
		}

		if (gCountForSensor == 128)
		{
			gCountForSensor = 0;

			updateOrientation(x, y, z);

			if(gShouldPlayFill == 0) 
				gCurrentPattern = gOrientation;
			else if (gShouldPlayFill == 1)
				gCurrentPattern = gFillPattern;
			gCurrentIndexInPattern = gCurrentIndexInPattern % gPatternLengths[gCurrentPattern];
			// rt_printf("X: %d\tY: %d\tZ: %d Pattern: %d\n", gXState, gYState, gZState, gCurrentPattern);

		}
		



		// count samples and trigger event if running

		if (gIsPlaying)
		{
			if (gSampleCount >= gEventIntervalMilliseconds * 0.001 * context->audioSampleRate)
			{
				startNextEvent();
				gSampleCount = 0;

				if(gLedState == 0) {
					gLedState = 1;
				}
				else {
					gLedState = 0;
				}

			}		
			gSampleCount++;
		}

		digitalWriteFrame(context, n, ledPin, gLedState);

		// If drum triggered read through samples and add to output buffer

		for (int i = 0; i < NUMBER_OF_READPOINTERS; i++)
		{
			int buffer = gDrumBufferForReadPointer[i];
			// rt_printf("Buffer %d: %d\n", i, buffer);
			if (buffer != -1)
			{
				output += gDrumSampleBuffers[buffer][gReadPointers[i]];

				if (gPlaysBackwards == 0) {
					gReadPointers[i]++;

					if (gReadPointers[i] >= gDrumSampleBufferLengths[buffer])
					{
						gDrumBufferForReadPointer[i] = -1;
					}
				} else if (gPlaysBackwards == 1)
				{
					gReadPointers[i]--;

					if (gReadPointers[i] == 0 )
					{
						gDrumBufferForReadPointer[i] = -1;
					}
				}
			}
		}
		// Write to output buffers

		output *= 0.25;

		audioWriteFrame(context, n, 0, output);
		audioWriteFrame(context, n, 1, output);

	}


}

/* Start playing a particular drum sound given by drumIndex.
 */
void startPlayingDrum(int drumIndex) {
	/* TODO in Steps 3a and 3b */
	// Find available readPointer
	// set gDrumBufferForReadPointer
	// Can return without changing if all are busy

	for (int i = 0; i < 16; i++)
	{
		if (gDrumBufferForReadPointer[i] == -1)
		{
			gDrumBufferForReadPointer[i] = drumIndex;
			if (gPlaysBackwards == 0)
				gReadPointers[i] = 0;
			if (gPlaysBackwards == 1)
				gReadPointers[i] = gDrumSampleBufferLengths[i] - 1;
			break;
		}
	}
}

/* Start playing the next event in the pattern */
void startNextEvent() {
	// Trigger kick sound

	int event = gPatterns[gCurrentPattern][gCurrentIndexInPattern];

 	rt_printf("Pattern: %d\t Index: %d\n", gCurrentPattern, gCurrentIndexInPattern);

	for (int i = 0; i < NUMBER_OF_DRUMS; i++)
	{
		if (eventContainsDrum(event, i))
		{
			startPlayingDrum(i);
		}
	}

	gCurrentIndexInPattern++;
	if (gCurrentIndexInPattern >= gPatternLengths[gCurrentPattern])
	{
		gCurrentIndexInPattern = 0;
		if (gCurrentPattern == gFillPattern){
			rt_printf("Reset\n");
			gCurrentPattern = gPreviousPattern;
			gShouldPlayFill = 0;
		}
	}
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
