#include "node_flash.h"

void parseUnsignedCharArrayToInt(unsigned char * char_array,unsigned int * dataReturn){
//parse unsigned char array to int
//private function
	dataReturn[0] = (uint64_t)char_array[0] << 24 |
									(uint64_t)char_array[1] << 16 |
									(uint64_t)char_array[2] << 8 |
									(uint64_t)char_array[3];

}

void EraseWriteFlash:: UnlockFlash(void){

	//unlock flash for writing
	HAL_FLASH_Unlock();

}
void EraseWriteFlash::LockFlash(void){

	//lock flash to aviod rewriting after this command flash only can be read 
	HAL_FLASH_Lock();
}

FlashStatusTypeDef EraseWriteFlash::AdjustSystemParams(EraseWriteFlash *flash){

	//////////////////////adjust class variables for flash read write
	flash->TypeErase=FLASH_TYPEERASE_PAGES;
	flash->PageAddress=FLASH_USER_START_ADDR;
	flash->NbPages=(FLASH_USER_END_ADDR - FLASH_USER_START_ADDR) / FLASH_PAGE_SIZE;
	flash->Address=FLASH_USER_START_ADDR;
	//////////////////////////////////////////////////////////////////////////////////
	//////////////////////adjust erase struct parameters//////////////////////////////
	flash->EraseInitStruct.TypeErase=flash->TypeErase;
	flash->EraseInitStruct.PageAddress= PageAddress;
	flash->EraseInitStruct.NbPages=flash->NbPages;

	return FLASH_OK;
}
FlashStatusTypeDef EraseWriteFlash::EraseFlash(EraseWriteFlash *flash){
	

	return (FlashStatusTypeDef)	HAL_FLASHEx_Erase(&flash->EraseInitStruct, &flash->PAGEError);
	
}


FlashStatusTypeDef EraseWriteFlash::WriteFlash(EraseWriteFlash *flash,unsigned char *data, unsigned int lenght){

		int dataLenght=lenght/4; // unsinged char lenght 1 byte 4 byte is an integer adress writing 
		int dataLenghtCounter;
		unsigned int dataForFlash;
		unsigned short errorCounter=0;
		
		while(flash->Address<FLASH_USER_END_ADDR){
				
				parseUnsignedCharArrayToInt(data,&dataForFlash);	//parse unsigned data array to long 
				if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flash->Address, dataForFlash) == HAL_OK){
						
						flash->Address=flash->Address+4;//4 is for address width
						dataLenghtCounter++;
						if(dataLenghtCounter>dataLenght)
								return FLASH_OVER_WRITE;// if length is shorter then page no overwirte ocurs
				}else{
					
					errorCounter++;
				
				}
				if(errorCounter>=0x7FFF)
					return FLASH_ERROR;
		}
	
		errorCounter=0;//reset error counter
		return FLASH_OK;
}
FlashStatusTypeDef EraseWriteFlash::ReadFlash(EraseWriteFlash *flash, unsigned int *data_array,unsigned int lenght){


	int data_lenght=lenght/4; // unsinged char lenght 1 byte 4 byte is an integer adress writing 
	int tmpC=0;
	while(flash->Address<FLASH_USER_END_ADDR){
		    
			data_array[tmpC] = *(__IO uint32_t *)flash->Address;
			flash->Address = flash->Address + 4;
			tmpC++;
			if(tmpC>data_lenght)
				 return FLASH_OVER_WRITE;// if length is shorter then page no overwirte ocurs
			}

		return FLASH_OK;	
}
		

