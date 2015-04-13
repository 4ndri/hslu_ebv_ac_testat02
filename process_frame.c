/* Copying and distribution of this file, with or without modification,
 * are permitted in any medium without royalty. This file is offered as-is,
 * without any warranty.
 */

/*! @file process_frame.c
 * @brief Contains the actual algorithm and calculations.
 */

/* Definitions specific to this application. Also includes the Oscar main header file. */
#include "template.h"
#include <string.h>
#include <stdlib.h>

#define IMG_SIZE NUM_COLORS*(OSC_CAM_MAX_IMAGE_WIDTH/2)*(OSC_CAM_MAX_IMAGE_HEIGHT/2)
#define Border 6
#define scale 3
#define SizeBox 4

const int nc = OSC_CAM_MAX_IMAGE_WIDTH / 2;
const int nr = OSC_CAM_MAX_IMAGE_HEIGHT / 2;

int TextColor;
int avgDxy[3][IMG_SIZE];
int helpBuf[IMG_SIZE];
int mc[IMG_SIZE];
int locMaxBuf[IMG_SIZE];
int maxBin[IMG_SIZE];

void ResetProcess() {
	//called when "reset" button is pressed
	if (TextColor == CYAN)
		TextColor = MAGENTA;
	else
		TextColor = CYAN;
}

void ProcessFrame() {
	uint32 t1, t2;
	//initialize counters
	if (data.ipc.state.nStepCounter == 1) {
		//use for initialization; only done in first step
		memset(data.u8TempImage[THRESHOLD], 0, IMG_SIZE);
		TextColor = CYAN;
	} else {
		//example for time measurement
		t1 = OscSupCycGet();
		CalcDeriv();

		for (int i = 0; i < 3; i++) {
			AvgDeriv(i);
		}
		CalcMc();
		CopyIntBuffer(data.u8TempImage[BACKGROUND], mc, 10);
		CalcLocMax();
		CopyIntBuffer(data.u8TempImage[THRESHOLD], maxBin, 0);
		//example for time measurement
		t2 = OscSupCycGet();

		//example for log output to console
		OscLog(INFO, "required = %d us\n", OscSupCycToMicroSecs(t2 - t1));

		//example for drawing output
		//draw line
		//DrawLine(10, 100, 200, 20, RED);
		//draw open rectangle
		//DrawBoundingBox(20, 10, 50, 40, false, GREEN);
		//draw filled rectangle
		//DrawBoundingBox(80, 100, 110, 120, true, BLUE);
		//DrawString(200, 200, strlen(Text), TINY, TextColor, Text);
	}
}

void CopyIntBuffer(unsigned char* dst, int* src, int scaleExp2) {
	for (int i = 0; i < IMG_SIZE; i++)
		*(dst + i) = (uint8) MIN(255, MAX(0, (*(src + i)) >> scaleExp2));
}

void CalcDeriv() {
	int c, r;
	for (r = nc; r < nr * nc - nc; r += nc) {/* we skip the first and last line */
		for (c = 1; c < nc - 1; c++) {
			/* do pointer arithmetics with respect to center pixel location */
			unsigned char* p = &data.u8TempImage[SENSORIMG][r + c];
			/* implement Sobel filter */
			int dx = -(int) *(p - nc - 1) + (int) *(p - nc + 1)
					- 2 * (int) *(p - 1) + 2 * (int) *(p + 1)
					- (int) *(p + nc - 1) + (int) *(p + nc + 1);
			int dy = -(int) *(p - nc - 1) - 2 * (int) *(p - nc)
					- (int) *(p - nc + 1) + (int) *(p + nc - 1)
					+ 2 * (int) *(p + nc) + (int) *(p + nc + 1);
			avgDxy[0][r + c] = dx * dx;
			avgDxy[1][r + c] = dy * dy;
			avgDxy[2][r + c] = dx * dy;

		}
	}
}

void AvgDeriv(int Index) {
//do average in x-direction
	int c, r;
	for (r = nc; r < nr * nc - nc; r += nc) {
		/* we skip first and last lines (empty) */
		for (c = Border + 1; c < nc - (Border + 1); c++) {
			/* +1 because we have one empty border column */
			/* do pointer arithmetics with respect to center pixel location */
			int* p = &avgDxy[Index][r + c];

			int sx = (*(p - 6) + *(p + 6)) * 1 + (*(p - 5) + *(p + 5)) * 4
					+ (*(p - 4) + *(p + 4)) * 11 + (*(p - 3) + *(p + 3)) * 27
					+ (*(p - 2) + *(p + 2)) * 50 + (*(p - 1) + *(p + 1)) * 72
					+ (*p) * 82;
//now averaged
			helpBuf[r + c] = (sx >> 8);

		}
	}
//do average in y-direction
	for (r = (Border + 1) * nc; r < nr * nc - ((Border + 1) * nc); r += nc) {
		/* we skip first and last lines (empty) */
		for (c = 1; c < nc - 1; c++) {
			/* +1 because we have one empty border column */
			/* do pointer arithmetics with respect to center pixel location */
			int* p = &helpBuf[r + c];
			int sy = (*(p - (6 * nc)) + *(p + (6 * nc))) * 1
					+ (*(p - (5 * nc)) + *(p + (5 * nc))) * 4
					+ (*(p - (4 * nc)) + *(p + (4 * nc))) * 11
					+ (*(p - (3 * nc)) + *(p + (3 * nc))) * 27
					+ (*(p - (2 * nc)) + *(p + (2 * nc))) * 50
					+ (*(p - nc) + *(p + nc)) * 72 + (*p) * 82;
			avgDxy[Index][r + c] = (sy >> 8);
			avgDxy[Index][r + c] = avgDxy[Index][r + c] >> scale;
			data.u8TempImage[THRESHOLD][r + c] = (uint8) MIN(255,
					MAX(0, avgDxy[Index][r + c]));
		}
	}
}

void CalcMc() {
	for (int i = 0; i < IMG_SIZE; i++) {
		mc[i] = (avgDxy[0][i] * avgDxy[1][i] - avgDxy[2][i] * avgDxy[2][i])
				- 5
						* (((avgDxy[0][i] + avgDxy[1][i])
								* (avgDxy[0][i] + avgDxy[1][i])) >> 7);
	}
}

void CalcLocMax() {
	memset(helpBuf, 0, IMG_SIZE * sizeof(int));
	memset(locMaxBuf, 0, IMG_SIZE * sizeof(int));
	memset(maxBin, 0, IMG_SIZE * sizeof(int));
	int c, r;
	int globalMaxMC = 0;
	int absThreshold;
// find global max mc
	for (int i = 0; i < IMG_SIZE; i++)
		globalMaxMC = MAX(globalMaxMC, mc[i]);
	absThreshold = (globalMaxMC * data.ipc.state.nThreshold) / 100;
// x-direction
	for (r = nc; r < nr * nc - nc; r += nc) {
		for (c = Border + 1; c < nc - (Border + 1); c++) {
			int* p = &mc[r + c];
			for (int i = -6; i <= 6; i++)
				helpBuf[r + c] = MAX(*(p + i), helpBuf[r + c]);
		}
	}
//y-direction
	for (r = (Border + 1) * nc; r < nr * nc - ((Border + 1) * nc); r += nc) {
		/* we skip first and last lines (empty) */
		for (c = 1; c < nc - 1; c++) {
			int* p = &helpBuf[r + c];
			for (int i = -6 * nc; i <= 6 * nc; i += nc)
				locMaxBuf[r + c] = MAX(*(p + i), locMaxBuf[r + c]);
			if (locMaxBuf[r + c] == mc[r + c]) {
// is candidate, now check threshold
				if (mc[r + c] > absThreshold) {
					maxBin[r + c] = 255;
					DrawBoundingBox(c - SizeBox, r / nc + SizeBox, c + SizeBox,
							r / nc - SizeBox, 0, GREEN);
				}
			}
		}
	}
}
