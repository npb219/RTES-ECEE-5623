#include "ecclib.h"
#include <stdio.h>
#include <stdarg.h>  // For va_list, va_start, va_end

static int printTrace=0;
static FILE *tracefile;

void traceOn(void)
{
    printTrace=1;
    tracefile = fopen("tracefile.txt", "w");
}

void traceOff(void)
{
    printTrace=0;
}

// write a trace to the file
void write_trace(const char *format, ...) {
    if (tracefile != NULL) {
        va_list args;
        va_start(args, format);  // Initialize the va_list with the last known fixed argument

        vfprintf(tracefile, format, args);  // Write the formatted string to the file

        va_end(args);  // Clean up the va_list
    } else {
        printf("File not opened!\n");
    }
}

// close the file when done
void close_file() {
    if (tracefile != NULL) {
        fclose(tracefile);  // Close the file
    }
}

int write_byte(ecc_t *ecc, unsigned char *address, unsigned char byteToWrite) {
    unsigned int offset = address - ecc->data_memory;
    unsigned char codeword=0;

    ecc->data_memory[offset]= byteToWrite;
    codeword = get_codeword(ecc, offset);
    ecc->code_memory[offset] = codeword;

    if(printTrace) { write_trace("WRITE : COMPUTED PARITY = ");print_code(codeword); }
    if(printTrace) { write_trace("WRITE : PARITY          = ");print_code_word(ecc, address); }
    if(printTrace) { write_trace("WRITE : DATA            = ");print_data_word(ecc, address); }
    if(printTrace) { write_trace("WRITE : ECODED          = ");print_encoded(ecc, address); }
    if(printTrace) { write_trace("\n"); }
    

    return NO_ERROR;
}


int read_byte(ecc_t *ecc, unsigned char *address, unsigned char *byteRead) {
    unsigned int offset = address - ecc->data_memory;
    unsigned char SYNDROME=0, pW2=0, pW=0, codeword=0;

    codeword = get_codeword(ecc, offset);

    SYNDROME = (codeword & SYNBITS) ^ (ecc->code_memory[offset] & SYNBITS);

    pW = (ecc->code_memory[offset]) & PW_BIT;
    // BUG -- pW2 = (codeword) & PW_BIT; 
    pW2 |= (PW_BIT & (
                      (((
                          (ecc->data_memory[offset] & DATA_BIT_1) ^ 
                         ((ecc->data_memory[offset] & DATA_BIT_2)>>1) ^ 
                         ((ecc->data_memory[offset] & DATA_BIT_3)>>2) ^ 
                         ((ecc->data_memory[offset] & DATA_BIT_4)>>3) ^ 
                         ((ecc->data_memory[offset] & DATA_BIT_5)>>4) ^ 
                         ((ecc->data_memory[offset] & DATA_BIT_6)>>5) ^ 
                         ((ecc->data_memory[offset] & DATA_BIT_7)>>6) ^ 
                         ((ecc->data_memory[offset] & DATA_BIT_8)>>7)) ^ 
                         ((ecc->code_memory[offset] & P01_BIT) ^ 
                         ((ecc->code_memory[offset] & P02_BIT)>>1) ^ 
                         ((ecc->code_memory[offset] & P03_BIT)>>2) ^ 
                         ((ecc->code_memory[offset] & P04_BIT)>>3)))<<4) 
                        ) );


    if(printTrace) { write_trace("READ  : COMPUTED PARITY = ");print_code(codeword); }
    if(printTrace) { write_trace("READ  : PARITY          = ");print_code_word(ecc, address); }
    if(printTrace) { write_trace("READ  : DATA            = ");print_data_word(ecc, address); }
    if(printTrace) { write_trace("READ  : ECODED          = ");print_encoded(ecc, address); }
    if(printTrace) { write_trace("READ  : SYNDROME        = ");print_code(SYNDROME); }
    if(printTrace) { write_trace("READ  : PW              = ");print_code(pW); }
    if(printTrace) { write_trace("READ  : PW2             = ");print_code(pW2); }
    if(printTrace) { write_trace("\n"); }
    
    // 1) if SYNDROME ==0 and pW == pW2, return NO_ERROR
    if((SYNDROME == 0) && (pW == pW2))
    {
        return NO_ERROR;
    }

    // 2) if SYNDROME ==0 and pW != pW2, return PW_ERROR
        if((SYNDROME == 0) && (pW != pW2))
    {
        // restore pW to PW2
        write_trace("PW ERROR\n\n");
        ecc->code_memory[offset] |= pW2 & PW_BIT;
        return PW_ERROR;
    }

    // 3) if SYNDROME !=0 and pW == pW2, return DOUBLE_BIT_ERROR
    if((SYNDROME != 0) && (pW == pW2))
    {
        write_trace("DOUBLE BIT ERROR\n\n");
        return DOUBLE_BIT_ERROR;
    }

    // 4) if SYNDROME !=0 and pW != pW2, SBE, return SYNDROME
    if((SYNDROME != 0) && (pW != pW2))
    {
        write_trace("SBE @ %d\n\n", SYNDROME);
        return SYNDROME;
    }

    // if we get here, something is seriously wrong like triple bit
    // or worse error, so return UNKNOWN_ERROR    
    return UNKNOWN_ERROR;
}


unsigned char *enable_ecc_memory(ecc_t *ecc){
    int idx;
    for(idx=0; idx < MEM_SIZE; idx++) ecc->code_memory[idx]=0;
    return ecc->data_memory;
}

unsigned char get_codeword(ecc_t *ecc, unsigned int offset)
{

    unsigned char codeword=0;

    //write_trace("CODEWORD offset=%d\n", offset);

    // p01 - per spreadsheet model, compute even parity=0 over 7,5,4,3,2,1 bits
    codeword |= (P01_BIT & (
                        ((ecc->data_memory[offset] & DATA_BIT_1) ^ 
                        ((ecc->data_memory[offset] & DATA_BIT_2)>>1) ^ 
                        ((ecc->data_memory[offset] & DATA_BIT_4)>>3) ^ 
                        ((ecc->data_memory[offset] & DATA_BIT_5)>>4) ^ 
                        ((ecc->data_memory[offset] & DATA_BIT_7)>>6)) 
                       ) );

    // p02 - per spreadsheet modell, compute even parity=0 over 7,6,4,3,1 bits
    codeword |= (P02_BIT & (
                        (
                         ((ecc->data_memory[offset] & DATA_BIT_1) ^ 
                         ((ecc->data_memory[offset] & DATA_BIT_3)>>2) ^ 
                         ((ecc->data_memory[offset] & DATA_BIT_4)>>3) ^ 
                         ((ecc->data_memory[offset] & DATA_BIT_6)>>5) ^ 
                         ((ecc->data_memory[offset] & DATA_BIT_7)>>6))<<1) 
                        ) );

    // p03 - per spreadsheet model, compute even parity=0 over 8,4,3,2 bits
    codeword |= (P03_BIT & (
                        ((
                          ((ecc->data_memory[offset] & DATA_BIT_2)>>1) ^ 
                          ((ecc->data_memory[offset] & DATA_BIT_3)>>2) ^ 
                          ((ecc->data_memory[offset] & DATA_BIT_4)>>3) ^ 
                          ((ecc->data_memory[offset] & DATA_BIT_8)>>7))<<2) 
                         ) );

    // p04 - per spreadsheet model, compute even parity=0 over 8,7,6,5 bits
    codeword |= (P04_BIT & (
                        ((
                          ((ecc->data_memory[offset] & DATA_BIT_5)>>4) ^ 
                          ((ecc->data_memory[offset] & DATA_BIT_6)>>5) ^ 
                          ((ecc->data_memory[offset] & DATA_BIT_7)>>6) ^ 
                          ((ecc->data_memory[offset] & DATA_BIT_8)>>7))<<3) 
                         ) );

    // pW - per spreadsheet model compute even parity=0 over all bits
    codeword |= (PW_BIT & (
                      (((
                          (ecc->data_memory[offset] & DATA_BIT_1) ^ 
                         ((ecc->data_memory[offset] & DATA_BIT_2)>>1) ^ 
                         ((ecc->data_memory[offset] & DATA_BIT_3)>>2) ^ 
                         ((ecc->data_memory[offset] & DATA_BIT_4)>>3) ^ 
                         ((ecc->data_memory[offset] & DATA_BIT_5)>>4) ^ 
                         ((ecc->data_memory[offset] & DATA_BIT_6)>>5) ^ 
                         ((ecc->data_memory[offset] & DATA_BIT_7)>>6) ^ 
                         ((ecc->data_memory[offset] & DATA_BIT_8)>>7)) ^ 
                         ((codeword & P01_BIT) ^ 
                         ((codeword & P02_BIT)>>1) ^ 
                         ((codeword & P03_BIT)>>2) ^ 
                         ((codeword & P04_BIT)>>3)))<<4) 
                        ) );

    // set the encoded bit
    codeword |= ENCODED_BIT;

    return codeword;

}


void print_code_word(ecc_t *ecc, unsigned char *address) {
    unsigned int offset = address - ecc->data_memory;
    unsigned char codeword = ecc->code_memory[offset];

    write_trace("addr=%p (offset=%d) ", address, offset);
    if(codeword & PW_BIT) write_trace("1"); else write_trace("0");
    if(codeword & P01_BIT) write_trace("1"); else write_trace("0");
    if(codeword & P02_BIT) write_trace("1"); else write_trace("0");
    write_trace("_");
    if(codeword & P03_BIT) write_trace("1"); else write_trace("0");
    write_trace("___");
    if(codeword & P04_BIT) write_trace("1"); else write_trace("0");
    write_trace("____");
    write_trace("\n");
}


void print_data_word(ecc_t *ecc, unsigned char *address)
{
    unsigned int offset = address - ecc->data_memory;
    unsigned char dataword = ecc->data_memory[offset];

    write_trace("addr=%p (offset=%d) ", address, offset);

    write_trace("___");
    if(dataword & DATA_BIT_1) write_trace("1"); else write_trace("0");
    write_trace("_");
    if(dataword & DATA_BIT_2) write_trace("1"); else write_trace("0");
    if(dataword & DATA_BIT_3) write_trace("1"); else write_trace("0");
    if(dataword & DATA_BIT_4) write_trace("1"); else write_trace("0");
    write_trace("_");
    if(dataword & DATA_BIT_5) write_trace("1"); else write_trace("0");
    if(dataword & DATA_BIT_6) write_trace("1"); else write_trace("0");
    if(dataword & DATA_BIT_7) write_trace("1"); else write_trace("0");
    if(dataword & DATA_BIT_8) write_trace("1"); else write_trace("0");

    write_trace("    => ");
    if(dataword & DATA_BIT_8) write_trace("1"); else write_trace("0");
    if(dataword & DATA_BIT_7) write_trace("1"); else write_trace("0");
    if(dataword & DATA_BIT_6) write_trace("1"); else write_trace("0");
    if(dataword & DATA_BIT_5) write_trace("1"); else write_trace("0");
    write_trace(" ");
    if(dataword & DATA_BIT_4) write_trace("1"); else write_trace("0");
    if(dataword & DATA_BIT_3) write_trace("1"); else write_trace("0");
    if(dataword & DATA_BIT_2) write_trace("1"); else write_trace("0");
    if(dataword & DATA_BIT_1) write_trace("1"); else write_trace("0");
    write_trace(" [0x%02X]", dataword);

    write_trace("\n");
}


void print_encoded(ecc_t *ecc, unsigned char *address)
{
    unsigned int offset = address - ecc->data_memory;
    unsigned char codeword = ecc->code_memory[offset];
    unsigned char dataword = ecc->data_memory[offset];

    write_trace("addr=%p (offset=%d) ", address, offset);

    if(codeword & PW_BIT) write_trace("1"); else write_trace("0");
    if(codeword & P01_BIT) write_trace("1"); else write_trace("0");
    if(codeword & P02_BIT) write_trace("1"); else write_trace("0");
    if(dataword & DATA_BIT_1) write_trace("1"); else write_trace("0");
    if(codeword & P03_BIT) write_trace("1"); else write_trace("0");
    if(dataword & DATA_BIT_2) write_trace("1"); else write_trace("0");
    if(dataword & DATA_BIT_3) write_trace("1"); else write_trace("0");
    if(dataword & DATA_BIT_4) write_trace("1"); else write_trace("0");
    if(codeword & P04_BIT) write_trace("1"); else write_trace("0");
    if(dataword & DATA_BIT_5) write_trace("1"); else write_trace("0");
    if(dataword & DATA_BIT_6) write_trace("1"); else write_trace("0");
    if(dataword & DATA_BIT_7) write_trace("1"); else write_trace("0");
    if(dataword & DATA_BIT_8) write_trace("1"); else write_trace("0");

    write_trace("\n");
}

void print_code(unsigned char codeword)
{
    write_trace("codeword=");
    if(codeword & PW_BIT) write_trace("1"); else write_trace("0");
    if(codeword & P01_BIT) write_trace("1"); else write_trace("0");
    if(codeword & P02_BIT) write_trace("1"); else write_trace("0");
    if(codeword & P03_BIT) write_trace("1"); else write_trace("0");
    if(codeword & P04_BIT) write_trace("1"); else write_trace("0");

    write_trace("\n");
}
