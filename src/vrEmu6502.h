/*
 * Troy's 6502 Emulator
 *
 * Copyright (c) 2022 Troy Schrapel
 *
 * This code is licensed under the MIT license
 *
 * https://github.com/visrealm/vrEmu6502
 *
 */

#ifndef _VR_EMU_6502_H_
#define _VR_EMU_6502_H_

#if VR_6502_EMU_COMPILING_DLL
#define VR_EMU_6502_DLLEXPORT __declspec(dllexport)
#elif VR_6502_EMU_STATIC
#define VR_EMU_6502_DLLEXPORT extern
#elif __EMSCRIPTEN__
#include <emscripten.h>
#define VR_EMU_6502_DLLEXPORT EMSCRIPTEN_KEEPALIVE
#else
#define VR_EMU_6502_DLLEXPORT __declspec(dllimport)
#endif

#include <stdint.h>
#include <stdbool.h>  

/* ------------------------------------------------------------------
 * PRIVATE DATA STRUCTURE
 */
struct vrEmu6502_s;
typedef struct vrEmu6502_s VrEmu6502;

/* ------------------------------------------------------------------
 * CONSTANTS
 */
typedef enum
{
  CPU_6502,
  CPU_65C02,
  CPU_W65C02,
  CPU_R65C02
} vrEmu6502Model;

typedef enum
{
  IntRequested,
  IntCleared,
  IntLow = IntRequested,
  IntHigh = IntCleared
} vrEmu6502Interrupt;

typedef enum
{
  BitC = 0,
  BitZ,
  BitI,
  BitD,
  BitB,
  BitU,
  BitV,
  BitN
} vrEmu6502FlagBit;

typedef enum
{
  FlagC = 0x01 << BitC,  /* carry */
  FlagZ = 0x01 << BitZ,  /* zero */
  FlagI = 0x01 << BitI,  /* interrupt */
  FlagD = 0x01 << BitD,  /* decimal */
  FlagB = 0x01 << BitB,  /* brk */
  FlagU = 0x01 << BitU,  /* undefined */
  FlagV = 0x01 << BitV,  /* oVerflow */
  FlagN = 0x01 << BitN   /* negative */
} vrEmu6502Flag;


typedef enum
{
  AddrModeAbs,
  AddrModeAbsX,
  AddrModeAbsY,
  AddrModeAcc,
  AddrModeImm,
  AddrModeImp,
  AddrModeAbsInd,
  AddrModeAbsIndX,
  AddrModeIndX,
  AddrModeIndY,
  AddrModeRel,
  AddrModeZP,
  AddrModeZPI,
  AddrModeZPX,
  AddrModeZPY,
} vrEmu6502AddrMode;

/* ------------------------------------------------------------------
 * PUBLIC INTERFACE
 */

 /*
  * memory write function pointer
  */
typedef void(*vrEmu6502MemWrite)(uint16_t addr, uint8_t val);

/*
  * memory read function pointer
  * 
  * isDbg: some devices change their state when read 
  *        (eg. TMS9918 increments its address pointer)
  *        this flag will be false when the cpu is running
  *        however it can be true when querying the memory 
  *        for other purposes. devices should NOT change state
  *        when isDbg is true.
  *        
  */
typedef uint8_t(*vrEmu6502MemRead)(uint16_t addr, bool isDbg);


/*
 * create a new 6502
 */
VR_EMU_6502_DLLEXPORT VrEmu6502* vrEmu6502New(
                                    vrEmu6502Model model,
                                    vrEmu6502MemRead readFn,
                                    vrEmu6502MemWrite writeFn);

/* ------------------------------------------------------------------
 *
 * destroy a 6502
 */
VR_EMU_6502_DLLEXPORT void vrEmu6502Destroy(VrEmu6502* vr6502);

/* ------------------------------------------------------------------
 *
 * reset the 6502
 */
VR_EMU_6502_DLLEXPORT void vrEmu6502Reset(VrEmu6502* vr6502);

/* ------------------------------------------------------------------
 *
 * a single clock tick
 */
VR_EMU_6502_DLLEXPORT void vrEmu6502Tick(VrEmu6502* vr6502);

/* ------------------------------------------------------------------
 *
 * returns a pointer to the interrupt signal.
 * externally, you can modify it to set/reset the interrupt signal
 */
VR_EMU_6502_DLLEXPORT vrEmu6502Interrupt *vrEmu6502Int(VrEmu6502* vr6502);

/* ------------------------------------------------------------------
 *
 * returns a pointer to the nmi signal.
 * externally, you can modify it to set/reset the interrupt signal
 */
VR_EMU_6502_DLLEXPORT vrEmu6502Interrupt *vrEmu6502Nmi(VrEmu6502* vr6502);

/* ------------------------------------------------------------------
 *
 * return the program counter
 */
VR_EMU_6502_DLLEXPORT uint16_t vrEmu6502GetPC(VrEmu6502* vr6502);

/* ------------------------------------------------------------------
 *
 * set the program counter
 */
VR_EMU_6502_DLLEXPORT void vrEmu6502SetPC(VrEmu6502* vr6502, uint16_t pc);

/* ------------------------------------------------------------------
 *
 * return the accumulator
 */
VR_EMU_6502_DLLEXPORT uint8_t vrEmu6502GetAcc(VrEmu6502* vr6502);

/* ------------------------------------------------------------------
 *
 * return the x index register
 */
VR_EMU_6502_DLLEXPORT uint8_t vrEmu6502GetX(VrEmu6502* vr6502);

/* ------------------------------------------------------------------
 *
 * return the y index register
 */
VR_EMU_6502_DLLEXPORT uint8_t vrEmu6502GetY(VrEmu6502* vr6502);

/* ------------------------------------------------------------------
 *
 * return the processor status register
 */
VR_EMU_6502_DLLEXPORT uint8_t vrEmu6502GetStatus(VrEmu6502* vr6502);

/* ------------------------------------------------------------------
 *
 * return the stack pointer register
 */
VR_EMU_6502_DLLEXPORT uint8_t vrEmu6502GetStackPointer(VrEmu6502* vr6502);

/* ------------------------------------------------------------------
 *
 * return the current opcode
 */
VR_EMU_6502_DLLEXPORT uint8_t vrEmu6502GetCurrentOpcode(VrEmu6502* vr6502);

/* ------------------------------------------------------------------
 *
 * return the current opcode address
 */
VR_EMU_6502_DLLEXPORT uint16_t vrEmu6502GetCurrentOpcodeAddr(VrEmu6502* vr6502);

/* ------------------------------------------------------------------
 *
 * return the next opcode
 */
VR_EMU_6502_DLLEXPORT uint8_t vrEmu6502GetNextOpcode(VrEmu6502* vr6502);

/* ------------------------------------------------------------------
 *
 * return the opcode cycle
 */
VR_EMU_6502_DLLEXPORT uint8_t vrEmu6502GetOpcodeCycle(VrEmu6502* vr6502);

/* ------------------------------------------------------------------
 *
 * peek the top word on the stack
 */
VR_EMU_6502_DLLEXPORT uint16_t vrEmu6502Peek16(VrEmu6502* vr6502);

/* ------------------------------------------------------------------
 *
 * return the opcode mnemonic string
 */
VR_EMU_6502_DLLEXPORT
const char* vrEmu6502OpcodeToMnemonicStr(VrEmu6502* vr6502, uint8_t opcode);

/* ------------------------------------------------------------------
 *
 * return the opcode address mode
 */
VR_EMU_6502_DLLEXPORT
vrEmu6502AddrMode vrEmu6502GetOpcodeAddrMode(VrEmu6502* vr6502, uint8_t opcode);

/* ------------------------------------------------------------------
 *
 * return disassembled instruction as a string
 */
VR_EMU_6502_DLLEXPORT
int vrEmu6502DisassembleInstruction(VrEmu6502* vr6502, uint16_t addr, int bufferSize, char *buffer);



#endif // _VR_EMU_6502_CORE_H_
