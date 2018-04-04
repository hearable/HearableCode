//Funktion aufrufen um die Werte des ADC in den Algorithmus zu laden
//Matrix Daten als Speicherschnittstelle auf die der ADC und der Algorithmus zugreifen können

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

#define windowSize 64

//Definition von ADC-main-Funktion
void ADC(uint32_t ADC_SAMPLE_BUF_SIZE);


void ADCout(int datenpushnr) //datenpushnr, an welcher Stelle Daten gelesen werden sollen
{
	//Matrix definiert, 3 Arrays aus denen Werte genommen werden können
	uint32_t* daten = (uint32_t*) calloc(3*windowSize, sizeof(uint32_t)); //Array auf Heap
	//Definition Outputvariable
	uint32_t datenpush;
	//Daten vom ADC nehmen
	//Daten abspeichern
	datenpush = daten[datenpushnr];
	
	return datenpush;
}

//mit i++ und modulo 3*64 damit automatisch an richtiger stelle