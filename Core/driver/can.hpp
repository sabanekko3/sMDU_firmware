/*
 * can.hpp
 *
 *  Created on: 2023/07/15
 *      Author: yaa3k
 */

#ifndef DRIVER_CAN_HPP_
#define DRIVER_CAN_HPP_

#include "board_data.hpp"

#define CAN_RX_BUFF_N 16
#define CAN_RX_BUFF_AND 0xF

typedef struct{
	uint8_t data[8];
	size_t size;
	uint32_t id;
	bool is_ext_id;
	bool is_remote;

	bool is_free;
}can_frame_t;

enum class filter_mode{
	only_std,
	only_ext,
	std_and_ext,
};

class CAN_COM{
private:
	CAN_HandleTypeDef *can;
	const uint32_t rx_fifo;

	can_frame_t rx_buff[CAN_RX_BUFF_N];
	uint32_t head = 0;
	uint32_t tail = 0;

public:
	CAN_COM(CAN_HandleTypeDef *_can,uint32_t _rx_fifo)
	:can(_can),rx_fifo(_rx_fifo){
		for(uint32_t i = 0; i < CAN_RX_BUFF_N; i++){
			memset(rx_buff[i].data,0,sizeof(rx_buff));
			rx_buff[i].size = 0;
			rx_buff[i].id = 0;
			rx_buff[i].is_ext_id = false;
			rx_buff[i].is_remote = false;
			rx_buff[i].is_free = true;
		}
	}

	//can tx functions
	uint32_t tx_available(void){
		return HAL_CAN_GetTxMailboxesFreeLevel(can);
	}
	bool tx(can_frame_t &tx_data);

	//can rx fuctions
	uint32_t rx_available(void);
	void rx_interrupt_task(void);
	bool rx(can_frame_t &rx_frame);

	//can filter setting
	void set_filter_mask(uint32_t id,uint32_t mask,filter_mode mode,bool as_std);
	void set_filter_free(void);
};


#endif /* DRIVER_CAN_HPP_ */
