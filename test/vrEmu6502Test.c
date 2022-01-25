/*
 * Troy's 6502 Emulator - Test program
 *
 * Copyright (c) 2022 Troy Schrapel
 *
 * This code is licensed under the MIT license
 *
 * https://github.com/visrealm/vrEmu6502
 *
 */


#include "vrEmu6502.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>


/* ------------------------------------------------------------------
 * MEMORY
 */

uint8_t ram[0x10000];

uint8_t MemRead(uint16_t addr, bool isDbg)
{
  return ram[addr];
}

void MemWrite(uint16_t addr, uint8_t val)
{
  ram[addr] = val;
}


/* ------------------------------------------------------------------
 * CONSTANTS
 */

#define OUTPUT_FILTER       1000000   /* set to 1 to show all lines */
#define OUTPUT_RAM_FROM     0
#define OUTPUT_RAM_BYTES    0
#define OUTPUT_DETAIL_FROM  100000000

/* keep track of the number of instructions processed */
int instructionCount = 0;

/*
  * output cpu state
  */
void debug6502(VrEmu6502 *vr6502)
{
  if (instructionCount < OUTPUT_DETAIL_FROM)
  {
    /* only output every instructions to save time*/
    if (++instructionCount % OUTPUT_FILTER != 0)
      return;
  }

  uint8_t buffer[32];
  uint16_t pc = vrEmu6502GetCurrentOpcodeAddr(vr6502);
  vrEmu6502DisassembleInstruction(vr6502, pc, sizeof(buffer), buffer);
  uint8_t a = vrEmu6502GetAcc(vr6502);
  uint8_t x = vrEmu6502GetX(vr6502);
  uint8_t y = vrEmu6502GetY(vr6502);
  uint8_t sp = vrEmu6502GetStackPointer(vr6502);
  uint8_t status = vrEmu6502GetStatus(vr6502);


  printf("%-20s A: $%02x X: $%02x Y: $%02x SP: $%02x F: $%02x %c%c%c%c%c%c  -  ",
    buffer, a, x, y, sp, status,
    status & FlagN ? 'N' : '.',
    status & FlagV ? 'V' : '.',
    status & FlagD ? 'D' : '.',
    status & FlagI ? 'I' : '.',
    status & FlagC ? 'C' : '.',
    status & FlagZ ? 'Z' : '.');

  for (int i = 0; i < OUTPUT_RAM_BYTES; ++i)
  {
    printf("$%02x ", MemRead((OUTPUT_RAM_FROM + i) & 0xffff, 0));
  }
  printf("\n");

}

/*
 * program entry point
 */
int main(int argc, char *argv[])
{
  if (argc < 2)
  {
    printf("Usage: vrEmu6502Test program.hex\n");
    return 1;
  }


 /*
  * load the INTEL HEX file
  */

  FILE *hexFile = NULL;
  fopen_s(&hexFile, argv[1], "r");
  int runAddress = 0;

  if (hexFile)
  {
    char lineBuffer[1024];
    char tmpBuffer[10];

    while (fgets(lineBuffer, sizeof(lineBuffer), hexFile))
    {
      if (lineBuffer[0] != ':') continue;

      strncpy_s(tmpBuffer, sizeof(tmpBuffer), lineBuffer + 1, 2);
      int numBytes = (int)strtol(tmpBuffer, NULL, 16);

      strncpy_s(tmpBuffer, sizeof(tmpBuffer), lineBuffer + 3, 4);
      int destAddr = (int)strtol(tmpBuffer, NULL, 16);

      strncpy_s(tmpBuffer, sizeof(tmpBuffer), lineBuffer + 7, 2);
      int recType = (int)strtol(tmpBuffer, NULL, 16);

      if (recType == 0)
      {
        for (int i = 0; i < numBytes; ++i)
        {
          strncpy_s(tmpBuffer, sizeof(tmpBuffer), lineBuffer + 9 + (i*2), 2);
          ram[destAddr+i] = (uint8_t)strtol(tmpBuffer, NULL, 16);
        }
      }
      else if (recType == 1)
      {
        runAddress = destAddr;
        break;
      }
    }
    
    fclose(hexFile);
  }
  else
  {
    printf("Unable to open HEX file: %s\n", argv[1]);
    return 1;
  }

  if (runAddress == 0)
  {
    printf("Invalid run address: 0x%04x\n  Must be in the range (0x200 - 0xfff0)", runAddress);
    return 1;
  }

 /*
 * output some run info
 */
  printf("vrEmu6502 Test Program\n----------------------\n\n");
  printf("* Loaded %s\n* Starting at $%04x\n", argv[1], runAddress);
  if (OUTPUT_FILTER)
  {
    printf("* Output filtered to every %d instructions\n", OUTPUT_FILTER);
  }
  printf("\n");


 /*
  * build and test the cpu
  */
  VrEmu6502 *vr6502 = vrEmu6502New(CPU_W65C02, MemRead, MemWrite);
  if (vr6502)
  {
    /* reset the cpu (technically don't need to do this as vrEmu6502New does reset it) */
    vrEmu6502Reset(vr6502);

    vrEmu6502SetPC(vr6502, (uint16_t)runAddress);

    while (1)
    {
      if (vrEmu6502GetOpcodeCycle(vr6502) == 0)
      {
        debug6502(vr6502);

        /* break on STP instruction */
        if (vrEmu6502GetCurrentOpcode(vr6502) == 0xdb)
        {
          printf("\n%s finished - success!\n", argv[1]);
          break;
        }
      }
      
      /* call me once for each clock cycle (eg. 1,000,000 times per second for a 1MHz clock) */
      vrEmu6502Tick(vr6502);  
    }

    vrEmu6502Destroy(vr6502);
    vr6502 = NULL;
  }
  else
  {
    printf("Error creating VrEmu6502\n");
    return 1;
  }
  return 0;
}