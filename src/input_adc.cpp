#include <Arduino.h>
#include <ADC.h>
#include <Arduino.h>
#include <stdint.h>

#include "global.hpp"

static ADC* adc = new ADC();
static IntervalTimer timer;

// A14 (red) and A15 (yellow) are read by ADC0
// A16 (green) and A17 (broken blue) are read by ADC1
const int base_pins[] = {A14, A15, A16, A17};

void timer_callback();

void init_input() {
	for (size_t i = 0; i < num_channels; ++i)
		pinMode(base_pins[i], INPUT);

	if (num_channels >= 1) {
		adc->setAveraging(1);
		adc->setResolution(12);
		adc->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED);
		adc->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);
		adc->enableInterrupts(ADC_0);
	}

	if (num_channels >= 2) {
	    	adc->setAveraging(1, ADC_1);
		adc->setResolution(12, ADC_1);
		adc->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED, ADC_1);
		adc->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED, ADC_1);
	}
	timer.begin(timer_callback, int32_t(1000000 / sampling_rate));
}

static int16_t buffer[2][num_channels][block_size] __attribute__ ((aligned (128)));

static volatile size_t which_buffer = 0;
static volatile size_t buffer_idx = 0;

static volatile bool has_data = false;

int16_t (*input())[block_size] {
	while (!has_data);
	has_data = false;
	return buffer[1 - which_buffer];
}
bool onsecondread = false;
void timer_callback() {
	adc->startSynchronizedSingleRead(base_pins[0], base_pins[2]);

	onsecondread = false;
}
void adc0_isr() {
	if(!onsecondread){
		ADC::Sync_result result = adc->readSynchronizedSingle();
		buffer[which_buffer][0][buffer_idx] = result.result_adc0;
		buffer[which_buffer][2][buffer_idx] = result.result_adc1;

		adc->adc0->startReadFast(base_pins[1]);
		onsecondread = true;
	} else {
		buffer[which_buffer][1][buffer_idx] = adc->adc0->readSingle();

		++buffer_idx;
		if (buffer_idx >= block_size) {
			has_data = true;
			buffer_idx = 0;
			which_buffer = 1 - which_buffer;
		}
		onsecondread = false;
	}
}
