#include <assert.h>
#include "ecclib.h"
#include <syslog.h>

void flip_bit(ecc_t *ecc, unsigned char *address, unsigned short bit_to_flip);

int main(void)
{
    ecc_t ECC;
    int i, j;
    unsigned int offset=0; int rc; unsigned char byteToRead;
    unsigned short bitToFlip, bitToFlip2;
    unsigned char *base_addr=enable_ecc_memory(&ECC);

    // NEGATIVE testing - flip a SINGLE bit, read to correct
    traceOn();


    // Loop through 13 SBE cases
    for(i=0; i<13; i++)
    {
        syslog(LOG_CRIT, "test case %d\n", i);
        write_byte(&ECC, base_addr+0, (unsigned char)0xAB);
        bitToFlip=i;
        flip_bit(&ECC, base_addr+0, bitToFlip);
        rc=read_byte(&ECC, base_addr+offset, &byteToRead);
        flip_bit(&ECC, base_addr+0, bitToFlip);
        assert((rc=read_byte(&ECC, base_addr+offset, &byteToRead)) == NO_ERROR);
    }


    // Loop through 78 DBE cases
    for(i=0; i<13; i++)
    {
        write_byte(&ECC, base_addr+0, (unsigned char)0xAB);
        bitToFlip=i;
        flip_bit(&ECC, base_addr+0, bitToFlip);

        for(j=i+1; j<13; j++)
        {
            syslog(LOG_CRIT, "test case %d, %d\n", i, j);
            bitToFlip2=j;
            flip_bit(&ECC, base_addr+0, bitToFlip2);
            rc=read_byte(&ECC, base_addr+offset, &byteToRead);
            flip_bit(&ECC, base_addr+0, bitToFlip2);
        }
    }


    // There are 8192 total bit patterns for 13 bits, so 8100 cases remain which are 3 or more bit MBEs

 
    //Demonstration of specific test cases

    // TEST CASE 1: bit 3 is d01 @ position 0
    syslog(LOG_CRIT, "**** TEST CASE 1: bit 3 is d01 @ position 0 ******\n");
    write_byte(&ECC, base_addr+0, (unsigned char)0xAB);
    bitToFlip=3;
    flip_bit(&ECC, base_addr+0, bitToFlip);
    assert((rc=read_byte(&ECC, base_addr+offset, &byteToRead)) == bitToFlip);
    flip_bit(&ECC, base_addr+0, bitToFlip);
    assert((rc=read_byte(&ECC, base_addr+offset, &byteToRead)) == NO_ERROR);
    syslog(LOG_CRIT, "**** END TEST CASE 1 *****************************\n\n");

    // TEST CASE 2: bit 10 is d05 @ position 10
    syslog(LOG_CRIT, "**** TEST CASE 2: bit 10 is d05 @ position 10 ****\n");
    write_byte(&ECC, base_addr+0, (unsigned char)0x5A);
    bitToFlip=10;
    flip_bit(&ECC, base_addr+0, bitToFlip);
    assert((rc=read_byte(&ECC, base_addr+offset, &byteToRead)) == bitToFlip);
    flip_bit(&ECC, base_addr+0, bitToFlip);
    assert((rc=read_byte(&ECC, base_addr+offset, &byteToRead)) == NO_ERROR);
    syslog(LOG_CRIT, "**** END TEST CASE 2 *****************************\n\n");

    // TEST CASE 3: bit 0 is pW @ position 0
    syslog(LOG_CRIT, "**** TEST CASE 3: bit 0 is pW @ position 0 *******\n");
    write_byte(&ECC, base_addr+0, (unsigned char)0xCC);
    bitToFlip=0;
    flip_bit(&ECC, base_addr+0, bitToFlip);
    assert((rc=read_byte(&ECC, base_addr+offset, &byteToRead)) == PW_ERROR);
    syslog(LOG_CRIT, "**** END TEST CASE 3 *****************************\n\n");

    // TEST CASE 4: bit 1 is p01 @ position 1
    syslog(LOG_CRIT, "**** TEST CASE 4: bit 1 is p01 @ position 1 ******\n");
    write_byte(&ECC, base_addr+0, (unsigned char)0xAB);
    bitToFlip=1;
    flip_bit(&ECC, base_addr+0, bitToFlip);
    assert((rc=read_byte(&ECC, base_addr+offset, &byteToRead)) == bitToFlip);
    syslog(LOG_CRIT, "**** END TEST CASE 4 *****************************\n\n");
    traceOff();


    // POSITIVE testing - do read after write on all locations
    
    // TEST CASE 5: Read after Write all
    syslog(LOG_CRIT, "**** TEST CASE 5: Read after Write all ***********\n");
    for(offset=0; offset < MEM_SIZE; offset++)
        write_byte(&ECC, base_addr+offset, (unsigned char)offset);

    // read all of the locations back without injecting and error
    for(offset=0; offset < MEM_SIZE; offset++)
        assert((rc=read_byte(&ECC, base_addr+offset, &byteToRead)) == NO_ERROR);
    syslog(LOG_CRIT, "**** END TEST CASE 5 *****************************\n\n");


    return NO_ERROR;
}


// flip bit in encoded word: pW p1 p2 d1 p3 d2 d3 d4 p4 d5 d6 d7 d8
// bit position:             00 01 02 03 04 05 06 07 08 09 10 11 12
void flip_bit(ecc_t *ecc, unsigned char *address, unsigned short bit_to_flip) {
    unsigned int offset = address - ecc->data_memory;
    unsigned char byte=0;
    unsigned short data_bit_to_flip=0, parity_bit_to_flip=0;
    int data_flip=1;

    switch(bit_to_flip)
    {
        // parity bit pW, p01 ... p04
        case 0: 
            parity_bit_to_flip = 4;
            data_flip=0;
            break;
        case 1: 
            parity_bit_to_flip = 0;
            data_flip=0;
            break;
        case 2:
            parity_bit_to_flip = 1;
            data_flip=0;
            break;
        case 4:
            data_flip=0;
            parity_bit_to_flip = 2;
            break;
        case 8:
            data_flip=0;
            parity_bit_to_flip = 3;
            break;

        // data bit d01 ... d08
        case 3: 
            data_bit_to_flip = 0;
            break;
        case 5: 
            data_bit_to_flip = 1;
            break;
        case 6: 
            data_bit_to_flip = 2;
            break;
        case 7: 
            data_bit_to_flip = 3;
            break;
        case 9: 
            data_bit_to_flip = 4;
            break;
        case 10: 
            data_bit_to_flip = 5;
            break;
        case 11: 
            data_bit_to_flip = 6;
            break;
        case 12: 
            data_bit_to_flip = 7; 
            break;


        default:
            syslog(LOG_CRIT, "flipped bit OUT OF RANGE\n");
            return;
    }

    if(data_flip)
    {
        syslog(LOG_CRIT, "DATA  : request=%hu\n", bit_to_flip);
        syslog(LOG_CRIT, "DATA  : bit to flip=%hu\n", data_bit_to_flip);

        byte = ecc->data_memory[offset];
        syslog(LOG_CRIT, "DATA  : original byte    = 0x%02X\n", byte);
        byte ^= (1 << (data_bit_to_flip));
        syslog(LOG_CRIT, "DATA  : flipped bit byte = 0x%02X\n\n", byte);
        ecc->data_memory[offset] = byte;

    }
    else
    {
        syslog(LOG_CRIT, "PARITY: request=%hu\n", bit_to_flip);
        syslog(LOG_CRIT, "PARITY: bit to flip=%hu\n", parity_bit_to_flip);

        byte = ecc->code_memory[offset];
        syslog(LOG_CRIT, "PARITY: original byte    = 0x%02X\n", byte);
        byte ^= (1 << (parity_bit_to_flip));
        syslog(LOG_CRIT, "PARITY: flipped bit byte = 0x%02X\n\n", byte);
        ecc->code_memory[offset] = byte;
    }

}

