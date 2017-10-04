#include <stdint.h>

typedef struct FontTable {
    uint16_t    width;           
    uint16_t    start;     
		uint16_t    offset;	
} FONT_CHAR_INFO;



typedef struct
{
    uint8_t Height;                                   
    uint8_t FirstChar;                               
    uint8_t LastChar;                                
    uint8_t FontSpace;                              
    const FONT_CHAR_INFO *FontTable;    
    const uint16_t *FontBitmaps;         
} FONT_INFO;


/* Font data for Arial 10pt */
extern const uint16_t arial_10ptBitmaps[];
extern const FONT_INFO arial_10ptFontInfo;
extern const FONT_CHAR_INFO arial_10ptDescriptors[];


