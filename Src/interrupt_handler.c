#include "interrupt_handler.h"
//#include "interrupt_config.h"
//#include "bsp_adc.h"
#include "led.h"

__IO int rate[10]={0,};                    // array to hold last ten IBI values
__IO unsigned long sampleCounter;   // used to determine pulse timing
__IO unsigned long lastBeatTime;    // used to find IBI
__IO int P;                         // used to find peak in pulse wave, seeded
__IO int T;                         // used to find trough in pulse wave, seeded
__IO int thresh;                    // used to find instant moment of heart beat, seeded
__IO int amp;                       // used to hold amplitude of pulse waveform, seeded
__IO bool firstBeat;                 // used to seed rate array so we startup with reasonable BPM
__IO bool secondBeat;                // used to seed rate array so we startup with reasonable BPM
__IO int BPM;                       // used to hold the pulse rate
__IO int IBI;                       // holds the time between beats, must be seeded!
__IO bool Pulse;                     // true when pulse wave is high, False when it's low
__IO bool QS;                        // becomes True when Arduoino finds a beat.
__IO int fadeRate = 4000;             // breath light fadeRate

__IO int      counter;              // count timer interrupt to ten to send data
__IO uint32_t preSignal;            // store the last signal value
__IO int      Status_Flag;          // status flag stands for the state of current heart wave

extern __IO uint16_t adc[2];

/* used to detect trigger TRUE stands for needing to change state, FLASE not */
bool detectTrigger(uint32_t tempSignal) {

	bool changeFlag = false;                 // the first detect flag
	bool abnormalFlag = false;               // the return value of detect_state() method
	bool flag = false;                       // return value
	
	if (Status_Flag == NORMAL && BPM > 180) return true; // BPM>180 stands for abnormal so change current state
	
	// detect the signal value, delay a little bit time detect again to confirm if it's a trigger signal
	if (tempSignal <= LOWER_TRIGGER && (tempSignal - preSignal) < -TRIGGER_DELTA ) {
		Delay(0xff); //Delay(0xff);
		if ( (adc[0]>>2) <= LOWER_TRIGGER ) {
			changeFlag = true;
			abnormalFlag = detectState();      // detect if the following heart wave is normal or not
		} else {
			sendMessage("detect state, tempSignal <= LOWER_TRIGGER, !((adc[0]>>2) <= LOWER_TRIGGER )) \n\r");
		}
	} else {
		if (tempSignal > UPPER_TRIGGER && (tempSignal - preSignal) > TRIGGER_DELTA ) {
			Delay(0xff);
			if((adc[0]>>2) > UPPER_TRIGGER) {
				changeFlag = true;
				abnormalFlag = detectState();
			}
		}
	}
	
	if (changeFlag == false)
		flag = false;
	else if (Status_Flag == NORMAL && abnormalFlag == true)
		flag = true;
	else if (Status_Flag == ABNORMAL && abnormalFlag == false)
		flag = true;

	return flag;
}

/* detect 1500 signal values to judge if the following heart wave is normal or not*/
bool detectState() {

	bool abnormalFlag = false;                   // return value
	uint32_t detectedSignal = adc[0] >> 2;      // current signal value
	int abnormalCounter = 0;                    // the number of signal value between 513 and 520
	int sumCounter = 0;                         // control the num of detection, which equals to 1500
	uint64_t sumAll = 0;                        // sum of signal value
	uint64_t sumSquare = 0;                     // sum of the square of signal
	uint64_t variance = 0;                      // the variance of signal value
	uint64_t addNumber = 0;                     // the number of appropriate(400 to 700) signal value
	
	Delay(0xffff);
	// begin detection
	while (sumCounter != 1500) {
		if (detectedSignal < 700 && detectedSignal > 400) {
			sumAll += detectedSignal; // add the detected signal value
			sumSquare += detectedSignal * detectedSignal; // add the square of detected signal value
			++ addNumber; // add number

//			char temp_str[50];
//			snprintf(temp_str,sizeof(temp_str),"detectedSignal=%lu\n\r", detectedSignal );
//			sendMessage(temp_str);
		}
		++ sumCounter;

		// during the detection maintain previous state
		if (Status_Flag == ABNORMAL) {
			abnormalProcess(detectedSignal);
//			if (detectedSignal > 513 && detectedSignal < 520) ++ abnormalCounter; // if the signal value is in (513, 520), add number
			if (detectedSignal > DETECT_SIGNAL_LOW_LEVEL && detectedSignal < DETECT_SIGNAL_HIGH_LEVEL) ++ abnormalCounter; // if the signal value is in (513, 520), add number
		}

		if (Status_Flag == NORMAL) {
			normalProcess(detectedSignal);
			if (detectedSignal > DETECT_SIGNAL_LOW_LEVEL && detectedSignal < DETECT_SIGNAL_HIGH_LEVEL) ++ abnormalCounter;
		}

		Delay(0xfff);                             // delay to get new signal value
		detectedSignal = adc[0] >> 2;
	}

	// calculate variance
	sumAll /= addNumber; 
	sumSquare /= addNumber;
	variance = sumSquare - sumAll * sumAll;
	
	// if there are a lot of signal value between 513 and 520, or the variance is small enough
	// consider it as a abnormal state
	if (abnormalCounter > 250 || variance < 1000) abnormalFlag = true;

	if (!abnormalFlag) {
		char temp_str[50];
		snprintf(temp_str,sizeof(temp_str),"sumAll=%lu,variance=%lu,abnormalCounter=%u,", (uint32_t)sumAll,(uint32_t)variance,abnormalCounter );
		sendMessage(temp_str);
		sendMessage("abnormalFlag = false\n\r");
	}

	return abnormalFlag;
}

/* main method used to process signal */
void calculateProcess(uint32_t tempSignal) {

	int interval = 0;
	int i = 0;
	int runningTotal = 0;

//	sendMessage("Calculate process\n\r");
	sampleCounter += 2; // keep track of the time in mS with this variable
	interval = sampleCounter - lastBeatTime; // monitor the time since the last beat to avoid noise

// find the peak and trough of the pulse wave
	if (tempSignal < thresh && interval > (IBI/5)*3) { // avoid dichrotic noise by waiting 3/5 of last IBI
		if (tempSignal < T) { // T is the trough
			T = tempSignal; // keep track of lowest point in pulse wave
		}
	}

	if(tempSignal > thresh && tempSignal > P) { // thresh condition helps avoid noise
		P = tempSignal; // P is the peak
	} // keep track of highest point in pulse wave

// NOW IT'S TIME TO LOOK FOR THE HEART BEAT
// tempsignal surges up in value every time there is a pulse
	if (interval > 250) { // avoid high frequency noise
		if ((tempSignal > thresh) && (Pulse == false) && (interval > (IBI/5)*3) ) {
			Pulse = true; // set the Pulse flag when we think there is a pulse
			LED_On(); // turn on LED
			sendMessage("Pulse = true\n\r");
			IBI = sampleCounter - lastBeatTime; // measure time between beats in mS
			lastBeatTime = sampleCounter; // keep track of time for next pulse

			if (secondBeat) { // if this is the second beat, if secondBeat == True
				secondBeat = false; // clear secondBeat flag
				for (i=0; i<=9; i++) rate[i] = IBI; // seed the running total to get a realisitic BPM at startup
			}

			if (firstBeat) { // if it's the first time we found a beat, if firstBeat == True
				firstBeat = false; // clear firstBeat flag
				secondBeat = true; // set the second beat flag
				return; // IBI value is unreliable so discard it
			}
			
// keep a running total of the last 10 IBI values
			for (i=0; i<=8; i++) { // shift data in the rate array
				rate[i] = rate[i+1]; // and drop the oldest IBI value
				runningTotal += rate[i]; // add up the 9 oldest IBI values
			}

			rate[9] = IBI; // add the latest IBI to the rate array
			runningTotal += rate[9]; // add the latest IBI to runningTotal
			runningTotal /= 10; // average the last 10 IBI values
			BPM = 60000/runningTotal; // how many beats can fit into a minute? that's BPM!
			QS = true; // set Quantified Self flag
      // QS FLAG IS NOT CLEARED INSIDE THIS ISR
		}
	}

	if (tempSignal < thresh && Pulse == true) { // when the values are going down, the beat is over
		LED_Off();								// turn off LED
		Pulse = false;							// reset the Pulse flag so we can do it again
		amp = P - T;							// get amplitude of the pulse wave
		thresh = amp/2 + T;						// set thresh at 50% of the amplitude
		P = thresh;								// reset these for next time
		T = thresh;
	}

	if (interval > 2500) {				// if 2.5 seconds go by without a beat
		thresh = 512;					// set thresh default
		P = 512;						// set P default
		T = 512;						// set T default
		lastBeatTime = sampleCounter;	// bring the lastBeatTime up to date
		firstBeat = true;				// set these to avoid noise
		secondBeat = false;				// when we get the heartbeat back
	}
	ledFadeToBeat();
}

/* used to process the heart wave in normal state */
void normalProcess(uint32_t tempSignal) {

	calculateProcess(tempSignal);
	--counter;
	// count form 9 to 0 every 20ms to send
	if (counter == 0) {
		if (tempSignal > DETECT_SIGNAL_HIGH_LEVEL || tempSignal < DETECT_SIGNAL_LOW_LEVEL) // used to control wave
			sendDataToProcessing('S',512);
		else sendDataToProcessing('S', tempSignal);

		if (QS == true) { // Quantified Self flag is True when finds a heartbeat
			fadeRate = 4000;	
			sendDataToProcessing('B',BPM);               // send heart rate with a 'B' prefix
			sendDataToProcessing('Q',IBI);               // send time between beats with a 'Q' prefix
			QS = false;                                  // reset the Quantified Self flag for next time
		}
		counter = 9;
	}
	preSignal = tempSignal;                          // store current signal value
}

/* used to show straight line in abnormal state */
void abnormalProcess(uint32_t tempSignal) {

	--counter;
	if (counter == 0) {
		BPM = 0;
		IBI = 600;
		sendDataToProcessing('S', 512);
		sendDataToProcessing('B', 0);                  // send heart rate with a 'B' prefix
		sendDataToProcessing('Q', IBI);                // send time between beats with a 'Q' prefix
		counter = 9;
	}
	preSignal = tempSignal;
}
