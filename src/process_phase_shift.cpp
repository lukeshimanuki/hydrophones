#include <float.h>
#include <inttypes.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

//#include <Arduino.h>

#include "global.hpp"
#include "fxpt_atan2.hpp"
#include "dspinst.hpp"

// times are in uint64_t (micros)
// time diff is in int64_t (nanos)

static constexpr const int16_t B8 = 1 << 8;
static constexpr const int32_t B16 = 1 << 16;

static constexpr const uint64_t window = 1900000; // in us
static constexpr const uint32_t threshold = 0;

static const uint64_t frequencies[] = {
	25000
	//30000,
	//35000,
	//40000,
};
static const int8_t  num_frequencies = sizeof(frequencies) / sizeof(uint64_t);

static constexpr const uint64_t sine_wave_size = 64;

static int16_t real_kern[num_frequencies][sine_wave_size][block_size] __attribute__ ((aligned (64)));
static int16_t imag_kern[num_frequencies][sine_wave_size][block_size] __attribute__ ((aligned (64)));

static int32_t max_volt[num_channels] = {0};

void init_process() {
	int16_t sin_buf[sine_wave_size];
	int16_t ncos_buf[sine_wave_size];
	for (uint64_t i = 0; i < sine_wave_size; ++i) {
		const float angle = 2 * M_PI * i / sine_wave_size;
		sin_buf[i] = sin(angle) * B16 / 4;
		ncos_buf[i] = -cos(angle) * B16 / 4;
	}

	for (uint8_t f = 0; f < num_frequencies; ++f) {
		for (size_t j = 0; j < sine_wave_size; ++j) {
			for (uint64_t i = 0; i < block_size; ++i) {
				const uint64_t time = i * 1000000 / sampling_rate; // in uc
				const uint64_t microrevs = frequencies[f] * time % 1000000; // angle = 2pi * microrevs / 1000000
				const uint64_t wave_idx = microrevs * sine_wave_size / 1000000;
				const uint64_t idx = (wave_idx + j) % sine_wave_size;
				real_kern[f][j][i] = sin_buf[idx];
				imag_kern[f][j][i] = ncos_buf[idx];
			}
		}
	}
}

static uint64_t max_amplitude[num_frequencies] = {}; // the highest recently observed amplitude
static uint64_t max_time[num_frequencies] = {}; // the time of the highest observed amplitude
static int32_t max_real[num_frequencies][num_channels] = {};
static int32_t max_imag[num_frequencies][num_channels] = {};

static int32_t real[num_frequencies][num_channels] = {};
static int32_t imag[num_frequencies][num_channels] = {};

static char out[1024];
static const size_t len_out = sizeof(out) / sizeof(char);
static uint64_t iter = 0;

static inline int32_t dot(int16_t* pSrcA, int16_t* pSrcB) {
	int64_t result = 0;

	static_assert(block_size % 2 == 0, "need even block size");

	uint32_t offset = block_size * 2;

	while (offset > 0) {
		offset -= 4;
		union {
			uint32_t uint32;
			int16_t int16[2];
		} a, b;

#ifdef TEENSY_ASM
		__asm volatile ("ldr %0, [%1, %2]" : "=r" (a.uint32) : "r" (pSrcA), "r" (offset));
		__asm volatile ("ldr %0, [%1, %2]" : "=r" (b.uint32) : "r" (pSrcB), "r" (offset));
		__asm volatile ("smlald %Q0, %R0, %1, %2" : "+r" (result) : "r" (a), "r" (b));
#else
		a.uint32 = ((uint32_t*)pSrcA)[offset >> 2];
		b.uint32 = ((uint32_t*)pSrcB)[offset >> 2];
		result += int64_t(int64_t(a.int16[0]) * b.int16[0]) + int64_t(a.int16[1]) * b.int16[1];
#endif

	}

	return result / int32_t(block_size) / 16 / B8;
}

const char* process(int16_t (* const in)[block_size]) {
	size_t out_idx = 0;
	out[0] = '\0';

	const uint64_t time = iter * block_size * 1000000 / sampling_rate; // in uc
/*	for(int y = 0; y < block_size; y++){
		for(int x = 0; x < 3; x++){
			if(in[x][y] > max_volt[x]){
				max_volt[x] = in[x][y];
			}
		}
	}
*//*	
	if ( time % 10000 < 10){
		for(int x = 0; x < 3; x++){
			Serial.print(in[x][0]);
			Serial.print(" ");
		}
		Serial.println();
	}
*/
/*	if ( time % 1000000 < 100){
		Serial.println(int32_t(time - micros()));
	}
*/	// dft for only the relevant frequencies
	for (uint8_t f = 0; f < num_frequencies; ++f) {
		const uint64_t microrevs = frequencies[f] * time % 1000000; // angle = 2pi * microrevs / 1000000
		const uint64_t wave_idx = microrevs * sine_wave_size / 1000000;
		for (uint8_t c = 0; c < num_channels; ++c) {
			const int32_t real_part = dot(in[c], real_kern[f][wave_idx]);
			const int32_t imag_part = dot(in[c], imag_kern[f][wave_idx]);

			//real[f][c] = (real[f][c] + real_part)/2;//signed_halving_add_16_and_16(real[f][c], real_part);
			//imag[f][c] = (imag[f][c] + imag_part)/2;//signed_halving_add_16_and_16(imag[f][c], imag_part);
			real[f][c] = real_part;//signed_halving_add_16_and_16(real[f][c], real_part);
			imag[f][c] = imag_part;//signed_halving_add_16_and_16(imag[f][c], imag_part);
		}
	}

	// find time for which the minimum amplitude is highest
	if (time > window) {
		/*
		for(int x = 0; x < 3; x++){
			Serial.print(max_volt[x]);
			Serial.print('\t');
		}
		Serial.println();
		*/
		for (uint8_t f = 0; f < num_frequencies; ++f) {
			// handle overflow
			// should happen rarely
			if (time < max_time[f]) max_time[f] = 0;

			uint64_t min_amplitude = -1;
			for (uint8_t c = 0; c < num_channels; ++c) {
				const uint64_t amplitude = uint64_t(real[f][c] * real[f][c]) + uint64_t(imag[f][c] * imag[f][c]);
				if (amplitude < min_amplitude) min_amplitude = amplitude;
			}

			if (min_amplitude > max_amplitude[f]) {
				max_amplitude[f] = min_amplitude;
				max_time[f] = time;
				memcpy(max_real[f], real[f], sizeof(real[f]));
				memcpy(max_imag[f], imag[f], sizeof(imag[f]));
			}

			// if highest in time window, report
			if (time - max_time[f] >= window && max_amplitude[f] >= threshold) {
				int64_t time_diffs[num_channels] = {0};
				const uint32_t base_phase = fxpt_atan2(max_imag[f][0], max_real[f][0]);
				out_idx += snprintf(out + out_idx, len_out - out_idx, "%" PRIu64 "\t", frequencies[f]);
				for (uint8_t c = 1; c < num_channels; ++c) {
					uint32_t phase = fxpt_atan2(max_imag[f][c], max_real[f][c]);
					int32_t phase_shift = int32_t(phase) - base_phase;

					if (phase_shift < -B16 / 2) phase_shift += B16;
					if (phase_shift >= B16 / 2) phase_shift -= B16;

					const int64_t time_diff = int64_t(phase_shift) * 1000000000 / B16 / int64_t(frequencies[f]); // in nanos
					time_diffs[c] = time_diff;

					out_idx += snprintf(out + out_idx, len_out - out_idx, "%" PRId64 "\t", time_diff);
				}
				out_idx += snprintf(out + out_idx, len_out - out_idx, "%" PRIu64 "\t", max_amplitude[f]);

				out_idx += snprintf(out + out_idx, len_out - out_idx, "%" PRId64 "\n",
					int64_t(atan2(
						float(time_diffs[2]-time_diffs[1]),
						float(2*time_diffs[2] -(time_diffs[2]-time_diffs[1]))
					) / (2 * M_PI) * 360)
				);

				max_amplitude[f] = 0;
				max_time[f] = time;

				for (uint8_t c = 0; c < num_channels; ++c) {
					real[f][c] = 0;
					imag[f][c] = 0;
				}
			}
		}
	}

	++iter;

	return out;
}

