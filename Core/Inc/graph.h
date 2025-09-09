/*
 * graph.h
 *
 *  Created on: Jul 29, 2025
 *      Author: chandisecherine
 */

#ifndef INC_GRAPH_H_
#define INC_GRAPH_H_


 #include "st7735.h"

#define GRAPH_EDGE_L 20
#define GRAPH_EDGE_R 150
#define GRAPH_EDGE_T 30
#define GRAPH_EDGE_B 130
#define DATA_H 40
#define DATA_L 123
#define THICKNESS 3
#define CAPACITY (GRAPH_EDGE_R-GRAPH_EDGE_L)/THICKNESS
#define BACKGROUND ST7735_COLOR565(11100,110111, 11111)


extern volatile uint32_t buffer_size;

uint16_t convert_position(float val);

extern uint16_t buffer[CAPACITY];

void shift_buffer(uint16_t *buffer);

void draw_graph(uint16_t *buffer);




#endif /* INC_GRAPH_H_ */
