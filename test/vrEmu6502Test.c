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
 * GLOBALS
 */

/* keep track of the number of instructions processed */
uint64_t instructionCount  = 0;
uint64_t cycleCount = 0;
uint64_t outputCount = 0;
const char *filename       = NULL;

bool outputInstructionCount      = false;
uint64_t filterInstructionCount  = 0;
uint16_t showMemFrom             = 0;
uint16_t showMemBytes            = 0;
uint16_t runAddress              = 0;
bool quietMode                   = false;
uint64_t verboseFrom             = (uint64_t)-1;
bool hasOutput = false;

/*
  * output cpu state
  */
void debug6502(VrEmu6502 *vr6502)
{
  if (instructionCount < verboseFrom)
  {
    if (filterInstructionCount)
    {
      if (instructionCount % filterInstructionCount != 0)
        return;
    }
    else if (quietMode)
    {
      return;
    }
  }

  uint8_t buffer[32];
  uint16_t pc = vrEmu6502GetCurrentOpcodeAddr(vr6502);
  if (pc == 0x24f1)
  {
    int jj = 0;
  }
  vrEmu6502DisassembleInstruction(vr6502, pc, sizeof(buffer), buffer);
  uint8_t a = vrEmu6502GetAcc(vr6502);
  uint8_t x = vrEmu6502GetX(vr6502);
  uint8_t y = vrEmu6502GetY(vr6502);
  uint8_t sp = vrEmu6502GetStackPointer(vr6502);
  uint8_t status = vrEmu6502GetStatus(vr6502);

  if (outputCount++ % 50 == 0)
  {
    printf("\n");

    if (outputInstructionCount)
    {
      printf("Instr #     "    );
    }
    printf("PC     Instruction    Acc    InX    InY    SP        Status    ");
    if (showMemBytes)
    {
      printf("  $%04x", showMemFrom);
      if (showMemBytes > 1)
      {
        printf(" - $%04x", showMemFrom + showMemBytes - 1);
      }
    }

    printf("\n\n");

    hasOutput = true;
  }


  if (outputInstructionCount)
  {
    printf("#%-10lld ", instructionCount);
  }


  printf("%-20s A: $%02x X: $%02x Y: $%02x SP: $%02x F: $%02x %c%c%c%c%c%c  ",
    buffer, a, x, y, sp, status,
    status & FlagN ? 'N' : '.',
    status & FlagV ? 'V' : '.',
    status & FlagD ? 'D' : '.',
    status & FlagI ? 'I' : '.',
    status & FlagC ? 'C' : '.',
    status & FlagZ ? 'Z' : '.');

  for (int i = 0; i < showMemBytes; ++i)
  {
    printf("$%02x ", MemRead((showMemFrom + i) & 0xffff, 0));
  }
  printf("\n");

}

void banner()
{
  printf("\n  -------------------------------------\n");
  printf("          vrEmu6502 Test Runner\n");
  printf("  -------------------------------------\n");
  printf("    Copyright (c) 2022 Troy Schrapel\n");
  printf("  https://github.com/visrealm/vrEmu6502\n");
  printf("  -------------------------------------\n\n");
}


void usage(int status)
{
  if (status == 1)
  {
    printf("ERROR: Invalid option\n\n");
  }

  printf("Usage:\n");
  printf("vrEmu6502Test [OPTION...] <testfile.hex>\n\n");
  printf("Options:\n");
  printf("  -c                output instruction count\n");
  printf("  -f <lines>        filter output to every #<lines> lines\n");
  printf("  -h                output help and exit\n");
  printf("  -m <from>[:<to>]  output given memory address or range\n");
  printf("  -q                quiet mode - only print report\n");
  printf("  -r <addr>         override run address\n");
  printf("  -v [<count>]      verbose output from instruction #<count>\n");

  exit(status);
}

void beginReport()
{
  printf("Running test:                \"%s\"\n\n", filename);
  printf("Options:\n");
  printf("  Output instruction count:  %s\n", outputInstructionCount ? "Yes" : "No");
  printf("  Output filtering:          ");

  if (verboseFrom == (uint64_t)-1)
  {
    if (filterInstructionCount)
    {
      printf("Output every #%lld instructions\n", filterInstructionCount);
    }
    else if (quietMode)
    {
      printf("Quiet mode\n");
    }
    else
    {
      printf("Verbose\n");
    }
  }
  else
  {
    if (filterInstructionCount)
    {
      printf("Output every #%lld instructions until #%lld\n", filterInstructionCount, verboseFrom);
    }
    else
    {
      printf("Quiet until #%lld\n", verboseFrom);
    } 
  }

  if (showMemBytes)
  {
    printf("  Output memory:             $%04x", showMemFrom);
    if (showMemBytes > 1)
    {
      printf(" - $%04x", showMemFrom + showMemBytes - 1);
    }
    printf("\n");
  }
  printf("  Start address:             $%04x\n\n", runAddress);
}

void endReport(int status)
{
  printf("\n  -------------------------------------\n");
  printf("  \"%s\"\n\n", filename);
  printf("  Total instructions executed: %lld\n", instructionCount);
  printf("  Total clock cycles:          %lld\n", cycleCount);

  printf("\n  Test completed:              %s\n\n", status ? "FAILED" : "PASSED");
}


int readHexFile(const char* filename)
{

  /*
   * load the INTEL HEX file
   */

  FILE* hexFile = NULL;
  fopen_s(&hexFile, filename, "r");

  if (hexFile)
  {
    char lineBuffer[1024];
    char tmpBuffer[10];

    int totalBytesRead = 0;

    while (fgets(lineBuffer, sizeof(lineBuffer), hexFile))
    {
      if (lineBuffer[0] != ':') continue;

      strncpy_s(tmpBuffer, sizeof(tmpBuffer), lineBuffer + 1, 2);
      int numBytes = (int)strtol(tmpBuffer, NULL, 16);
      totalBytesRead += numBytes;

      strncpy_s(tmpBuffer, sizeof(tmpBuffer), lineBuffer + 3, 4);
      int destAddr = (int)strtol(tmpBuffer, NULL, 16);

      strncpy_s(tmpBuffer, sizeof(tmpBuffer), lineBuffer + 7, 2);
      int recType = (int)strtol(tmpBuffer, NULL, 16);

      if (recType == 0)
      {
        for (int i = 0; i < numBytes; ++i)
        {
          strncpy_s(tmpBuffer, sizeof(tmpBuffer), lineBuffer + 9 + (i * 2), 2);
          ram[destAddr + i] = (uint8_t)strtol(tmpBuffer, NULL, 16);
        }
      }
      else if (runAddress == 0 && recType == 1)
      {
        runAddress = destAddr;
        break;
      }
    }

    fclose(hexFile);

    if (totalBytesRead == 0)
    {
      printf("ERROR: Invalid Intel HEX file: %s\n", filename);
      return 0;
    }
    else if (runAddress == 0)
    {
      printf("WARNING: Run address not set from Intel HEX file: %s\n", filename);
      return 0;
    }
  }
  else
  {
    printf("ERROR: Unable to open HEX file: %s\n", filename);
    return 0;
  }
  return 1;
}

/*
 * program entry point
 */
int main(int argc, char *argv[])
{
  banner();

  for (int i = 1; i < argc; ++i)
  {
    if (argv[i][0] == '-')
    {
      switch (argv[i][1])
      {
        case 'c':
          outputInstructionCount = true;
          break;

        case 'f':
          if (++i < argc)
          {
            filterInstructionCount = strtol(argv[i], NULL, 0);
          }
          
          if (filterInstructionCount <= 0)
          {
            usage(1);
          }
          break;

        case 'h':
          usage(0);
          break;

        case 'm':
          if (++i < argc)
          {
            char *tok = strchr(argv[i], ':');
            
            if (tok)
            {
              showMemFrom = (uint16_t)strtol(argv[i], NULL, 0);
              showMemFrom = (uint16_t)strtol(argv[i], NULL, 0);
              uint16_t to = (uint16_t)strtol(tok + 1, NULL, 0);
              if (showMemFrom <= to)
              {
                showMemBytes = (to - showMemFrom) + 1;
              }
            }
            else
            {
              showMemFrom = (uint16_t)strtol(argv[i], NULL, 0);
              showMemBytes = 1;
            }
          }

          if (showMemBytes == 0)
          {
            usage(1);
          }

        case 'q':
          quietMode = true;
          break;

        case 'r':
          if (++i < argc)
          {
            runAddress = (uint16_t)strtol(argv[i], NULL, 0);
          }
          else
          {
            usage(1);
          }
          break;

        case 'v':
          verboseFrom = 0;
          if (++i < argc)
          {
            verboseFrom = strtol(argv[i], NULL, 0);
          }
          break;

        default:
          usage(1);
          break;

      }
    }
    else
    {
      filename = argv[i];
    }
  }

  if (!filename)
  {
    usage(2);
  }

  if (!readHexFile(filename))
    return 1;


  beginReport();

  int status = 0;


 /*
  * build and test the cpu
  */
  VrEmu6502 *vr6502 = vrEmu6502New(CPU_W65C02, MemRead, MemWrite);
  if (vr6502)
  {
    /* reset the cpu (technically don't need to do this as vrEmu6502New does reset it) */
    vrEmu6502Reset(vr6502);

    vrEmu6502SetPC(vr6502, (uint16_t)runAddress);

    uint16_t lastPc = 0;

    while (1)
    {
      ++cycleCount;

      if (vrEmu6502GetOpcodeCycle(vr6502) == 0)
      {
        uint16_t pc = vrEmu6502GetCurrentOpcodeAddr(vr6502);
        if (lastPc == pc)
        {
          status = 1;
          break;
        }
        lastPc = pc;


        ++instructionCount;

        debug6502(vr6502);

        /* break on STP instruction */
        if (vrEmu6502GetCurrentOpcode(vr6502) == 0xdb)
        {
          status = 0;
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

  endReport(status);

  return status;
}