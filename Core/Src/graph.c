/*
 * graph.c
 *
 *  Created on: Jul 29, 2025
 *      Author: chandisecherine
 */


#include "graph.h"
#include <stdlib.h>




uint16_t convert_position(float val){

	return ((3.3 - val)*(DATA_L - DATA_H))/3.3 + DATA_H;
}

void shift_buffer(uint16_t *buffer){

	for(int i = 0; i < CAPACITY-1; i++){
		buffer[i] = buffer[i+1];
	}
}


//ST7735_DrawPixel(uint16_t x, uint16_t y, uint16_t color)

void draw_graph(uint16_t *buffer){

	for(int i = 0; i < buffer_size; i++){
		ST7735_FillRectangleFast(i*THICKNESS, GRAPH_EDGE_T , THICKNESS, GRAPH_EDGE_B - GRAPH_EDGE_T, BACKGROUND);
		ST7735_FillRectangleFast(i*THICKNESS, buffer[i] , THICKNESS, THICKNESS, ST7735_BLUE);

	}

//	for(int i = 0; i < CAPACITY; i++){
//			ST7735_FillRectangleFast(i*THICKNESS, buffer[i], THICKNESS, THICKNESS, ST7735_BLUE);
//		}
}

