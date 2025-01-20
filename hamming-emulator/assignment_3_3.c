#include <assert.h>
#include "ecclib.h"

void flip_bit(ecc_t *ecc, unsigned char *address, unsigned short bit_to_flip);

int main(void)
{
    ecc_t ECC;
    int i, j;
    unsigned int offset=0; int rc; unsigned char byteToRead;
    unsigned short bitToFlip, bitToFlip2;
    unsigned char *base_addr=enable_ecc_memory(&ECC);
    int count = 0;//test count

    // NEGATIVE testing - flip a SINGLE bit, read to correct
    traceOn();
    
    //No flips
    write_trace("test case no flip\n");
    write_byte(&ECC, base_addr+0, (unsigned char)0xAB);
    assert((rc=read_byte(&ECC, base_addr+offset, &byteToRead)) == NO_ERROR);
    write_trace("CASES TESTED: %d\n", ++count);

    // Loop through 13 SBE cases
    for(i=0; i<13; i++)
    {
        write_trace("test case %d\n", i);
        write_byte(&ECC, base_addr+0, (unsigned char)0xAB);
        bitToFlip=i;
        flip_bit(&ECC, base_addr+0, bitToFlip);
        rc=read_byte(&ECC, base_addr+offset, &byteToRead);
        write_trace("error: %d\n", rc);
        if(i > 0)
            assert(rc == i);
        else
            assert(rc == PW_ERROR);
        flip_bit(&ECC, base_addr+0, bitToFlip);
        assert((rc=read_byte(&ECC, base_addr+offset, &byteToRead)) == NO_ERROR);
        write_trace("CASES TESTED: %d\n", ++count);
    }


    // Loop through 78 DBE cases
    for(i=0; i<13; i++)
    {
        write_byte(&ECC, base_addr+0, (unsigned char)0xAB);
        bitToFlip=i;
        flip_bit(&ECC, base_addr+0, bitToFlip);

        for(j=i+1; j<13; j++)
        {
            write_trace("test case %d, %d\n", i, j);
            bitToFlip2=j;
            flip_bit(&ECC, base_addr+0, bitToFlip2);
            rc=read_byte(&ECC, base_addr+offset, &byteToRead);
            write_trace("error: %d\n", rc);
            assert(rc == DOUBLE_BIT_ERROR);
            flip_bit(&ECC, base_addr+0, bitToFlip2);
            write_trace("CASES TESTED: %d\n", ++count);
        }
    }

    write_trace("\n\n\nTOTAL CASES TESTED: %d\n\n\n", count);


    // There are 8192 total bit patterns for 13 bits, so 8100 cases remain which are 3 or more bit MBEs

 
    //Demonstration of specific test cases

    // TEST CASE 1: bit 3 is d01 @ position 0
    write_trace("**** TEST CASE 1: bit 3 is d01 @ position 0 ******\n");
    write_byte(&ECC, base_addr+0, (unsigned char)0xAB);
    bitToFlip=3;
    flip_bit(&ECC, base_addr+0, bitToFlip);
    assert((rc=read_byte(&ECC, base_addr+offset, &byteToRead)) == bitToFlip);
    flip_bit(&ECC, base_addr+0, bitToFlip);
    assert((rc=read_byte(&ECC, base_addr+offset, &byteToRead)) == NO_ERROR);
    write_trace("**** END TEST CASE 1 *****************************\n\n");

    // TEST CASE 2: bit 10 is d05 @ position 10
    write_trace("**** TEST CASE 2: bit 10 is d05 @ position 10 ****\n");
    write_byte(&ECC, base_addr+0, (unsigned char)0x5A);
    bitToFlip=10;
    flip_bit(&ECC, base_addr+0, bitToFlip);
    assert((rc=read_byte(&ECC, base_addr+offset, &byteToRead)) == bitToFlip);
    flip_bit(&ECC, base_addr+0, bitToFlip);
    assert((rc=read_byte(&ECC, base_addr+offset, &byteToRead)) == NO_ERROR);
    write_trace("**** END TEST CASE 2 *****************************\n\n");

    // TEST CASE 3: bit 0 is pW @ position 0
    write_trace("**** TEST CASE 3: bit 0 is pW @ position 0 *******\n");
    write_byte(&ECC, base_addr+0, (unsigned char)0xCC);
    bitToFlip=0;
    flip_bit(&ECC, base_addr+0, bitToFlip);
    assert((rc=read_byte(&ECC, base_addr+offset, &byteToRead)) == PW_ERROR);
    write_trace("**** END TEST CASE 3 *****************************\n\n");

    // TEST CASE 4: bit 1 is p01 @ position 1
    write_trace("**** TEST CASE 4: bit 1 is p01 @ position 1 ******\n");
    write_byte(&ECC, base_addr+0, (unsigned char)0xAB);
    bitToFlip=1;
    flip_bit(&ECC, base_addr+0, bitToFlip);
    assert((rc=read_byte(&ECC, base_addr+offset, &byteToRead)) == bitToFlip);
    write_trace("**** END TEST CASE 4 *****************************\n\n");
    traceOff();


    // POSITIVE testing - do read after write on all locations
    
    // TEST CASE 5: Read after Write all
    write_trace("**** TEST CASE 5: Read after Write all ***********\n");
    for(offset=0; offset < MEM_SIZE; offset++)
        write_byte(&ECC, base_addr+offset, (unsigned char)offset);

    // read all of the locations back without injecting and error
    for(offset=0; offset < MEM_SIZE; offset++)
        assert((rc=read_byte(&ECC, base_addr+offset, &byteToRead)) == NO_ERROR);
    write_trace("**** END TEST CASE 5 *****************************\n\n");

    close_file(); //close trace file
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
            write_trace("flipped bit OUT OF RANGE\n");
            return;
    }

    if(data_flip)
    {
        write_trace("DATA  : request=%hu\n", bit_to_flip);
        write_trace("DATA  : bit to flip=%hu\n", data_bit_to_flip);

        byte = ecc->data_memory[offset];
        write_trace("DATA  : original byte    = 0x%02X\n", byte);
        byte ^= (1 << (data_bit_to_flip));
        write_trace("DATA  : flipped bit byte = 0x%02X\n\n", byte);
        ecc->data_memory[offset] = byte;

    }
    else
    {
        write_trace("PARITY: request=%hu\n", bit_to_flip);
        write_trace("PARITY: bit to flip=%hu\n", parity_bit_to_flip);

        byte = ecc->code_memory[offset];
        write_trace("PARITY: original byte    = 0x%02X\n", byte);
        byte ^= (1 << (parity_bit_to_flip));
        write_trace("PARITY: flipped bit byte = 0x%02X\n\n", byte);
        ecc->code_memory[offset] = byte;
    }

}

