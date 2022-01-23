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

#include "vrEmu6502.h"

#include <stdlib.h>
#include <memory.h>
#include <math.h>


 /*
  * instruction prototype
  */
typedef void(*vrEmu6502Instruction)(VrEmu6502*, uint16_t);

/*
 * address mode prototype
 */
typedef uint16_t(*vrEmu6502AddrMode)(VrEmu6502*);


struct vrEmu6502Opcode
{
  const vrEmu6502Instruction  instruction;
  const vrEmu6502AddrMode     addrMode;
  const uint8_t               cycles;
};
typedef struct vrEmu6502Opcode vrEmu6502Opcode;


/* ------------------------------------------------------------------
 * PRIVATE DATA STRUCTURE
 */
struct vrEmu6502_s
{
  vrEmu6502Model model;
  
  vrEmu6502MemRead readFn;
  vrEmu6502MemWrite writeFn;
  
  vrEmu6502Interrupt intPin;
  vrEmu6502Interrupt nmiPin;

  uint8_t step;
  uint8_t inNmi;
  uint8_t currentOpcode;
  uint8_t wai;

  uint16_t pc;

  uint8_t  ac;
  uint8_t  ix;
  uint8_t  iy;
  uint8_t  sp;

  union {
    struct {
      uint8_t car : 1;
      uint8_t zer : 1;
      uint8_t irq : 1;
      uint8_t dec : 1;
      uint8_t brk : 1;
      uint8_t usr : 1;
      uint8_t ovr : 1;
      uint8_t neg : 1;
    } ps;
    uint8_t psreg;
  };

  const vrEmu6502Opcode *opcodes;
};


static const vrEmu6502Opcode standard6502[256];
static const vrEmu6502Opcode standard65c02[256];
static const vrEmu6502Opcode wdc65c02[256];
static const vrEmu6502Opcode r65c02[256];



/* ------------------------------------------------------------------
 *  HELPER FUNCTIONS
 * ----------------------------------------------------------------*/

inline static void push(VrEmu6502* vr6502, uint8_t val)
{
  vr6502->writeFn(0x100 + vr6502->sp--, val);
}

inline static uint8_t pop(VrEmu6502* vr6502)
{
  return vr6502->readFn(0x100 + (++vr6502->sp));
}

inline static uint16_t read16(VrEmu6502* vr6502, uint16_t addr)
{
  return vr6502->readFn(addr) + (vr6502->readFn(addr + 1) << 8);
}

inline static uint16_t read16Bug(VrEmu6502* vr6502, uint16_t addr)
{
  if ((addr & 0xff) == 0xff) /* 6502 bug */
  {
    return vr6502->readFn(addr) + (vr6502->readFn(addr & 0xff00) << 8);
  }
  return vr6502->readFn(addr) + (vr6502->readFn(addr + 1) << 8);
}

inline static uint16_t read16Wrapped(VrEmu6502* vr6502, uint16_t addr)
{
  return vr6502->readFn(addr) + (vr6502->readFn(addr + 1) << 8);
}

inline static void pageBoundary(VrEmu6502* vr6502, uint16_t addr1, uint16_t addr2)
{
  vr6502->step += (addr1 & 0xff00) != (addr2 & 0xff00);
}

inline static void setNZ(VrEmu6502* vr6502, uint8_t val)
{
  vr6502->ps.neg = !!(val & 0x80);
  vr6502->ps.zer = !val;
}




/* ------------------------------------------------------------------
 *
 * create a new 6502
 */
VR_EMU_6502_DLLEXPORT VrEmu6502* vrEmu6502New(
  vrEmu6502Model model,
  vrEmu6502MemRead readFn,
  vrEmu6502MemWrite writeFn)
{
  VrEmu6502* vr6502 = (VrEmu6502*)malloc(sizeof(VrEmu6502));
  if (vr6502 != NULL)
  {
    vr6502->model = model;
    vr6502->readFn = readFn;
    vr6502->writeFn = writeFn;

    switch (model)
    {
      case CPU_65C02:
        vr6502->opcodes = standard65c02;
        break;

      case CPU_W65C02:
        vr6502->opcodes = wdc65c02;
        break;

      case CPU_R65C02:
        vr6502->opcodes = r65c02;
        break;

      default:
        vr6502->opcodes = standard6502;
        break;
    }

    vrEmu6502Reset(vr6502);
 }

  return vr6502;
}

/* ------------------------------------------------------------------
 *
 * destroy a 6502
 */
VR_EMU_6502_DLLEXPORT void vrEmu6502Destroy(VrEmu6502* vr6502)
{
  if (vr6502)
  {
    /* destruction */
    free(vr6502);
  }
}

/* ------------------------------------------------------------------
 *
 * reset the 6502
 */
VR_EMU_6502_DLLEXPORT void vrEmu6502Reset(VrEmu6502* vr6502)
{
  if (vr6502)
  {
    /* initialization */
    vr6502->intPin = IntCleared;
    vr6502->nmiPin = IntCleared;
    vr6502->inNmi = 0;
    vr6502->pc = read16(vr6502, 0xfffc);
    vr6502->step = 0;
    vr6502->wai = 0;
  }
}

static void beginInterrupt(VrEmu6502* vr6502, uint16_t addr)
{
  push(vr6502, vr6502->pc >> 8);
  push(vr6502, vr6502->pc & 0xff);
  push(vr6502, vr6502->psreg | 0x30);
  vr6502->ps.irq = 1;
  vr6502->wai = 0;
  vr6502->pc = read16(vr6502, addr);
}

/* ------------------------------------------------------------------
 *
 * a single clock tick
 */
VR_EMU_6502_DLLEXPORT void vrEmu6502Tick(VrEmu6502* vr6502)
{
  if (vr6502->step == 0)
  {
    if (vr6502->nmiPin == IntRequested && !vr6502->inNmi)
    {
      vr6502->inNmi = 1;
      beginInterrupt(vr6502, 0xfffa);
      return;
    }

    if (vr6502->intPin == IntRequested)
    {
      if (!vr6502->ps.irq)
      {
        beginInterrupt(vr6502, 0xfffe);
        return;
      }
      else if (vr6502->wai)
      {
        vr6502->wai = 0;
      }
    }
    
    if (!vr6502->wai)
    {
      vr6502->currentOpcode = vr6502->readFn(vr6502->pc++);

      /* find the instruction in the table */
      const vrEmu6502Opcode* opcode = &vr6502->opcodes[vr6502->currentOpcode];

      /* set cycles here as they may be adjusted by addressing mode */
      vr6502->step = opcode->cycles;

      /* execute the instruction */
      opcode->instruction(vr6502, opcode->addrMode(vr6502));
    }
  }

  if (vr6502->step) --vr6502->step;
}


/* ------------------------------------------------------------------
 *
 * returns a pointer to the interrupt signal.
 * externally, you can modify it to set/reset the interrupt signal
 */
VR_EMU_6502_DLLEXPORT vrEmu6502Interrupt* vrEmu6502Int(VrEmu6502* vr6502)
{
  if (vr6502)
  {
    return &vr6502->intPin;
  }
  return NULL;
}

/* ------------------------------------------------------------------
 *
 * returns a pointer to the nmi signal.
 * externally, you can modify it to set/reset the interrupt signal
 */
VR_EMU_6502_DLLEXPORT vrEmu6502Interrupt* vrEmu6502Nmi(VrEmu6502* vr6502)
{
  if (vr6502)
  {
    return &vr6502->nmiPin;
  }
  return NULL;
}

/* ------------------------------------------------------------------
 *
 * return the program counter
 */
VR_EMU_6502_DLLEXPORT uint16_t vrEmu6502GetPC(VrEmu6502* vr6502)
{
  return vr6502->pc;
}

/* ------------------------------------------------------------------
 *
 * return the accumulator
 */
VR_EMU_6502_DLLEXPORT uint8_t vrEmu6502GetAcc(VrEmu6502* vr6502)
{
  return vr6502->ac;
}

/* ------------------------------------------------------------------
 *
 * return the x index register
 */
VR_EMU_6502_DLLEXPORT uint8_t vrEmu6502GetX(VrEmu6502* vr6502)
{
  return vr6502->ix;
}

/* ------------------------------------------------------------------
 *
 * return the y index register
 */
VR_EMU_6502_DLLEXPORT uint8_t vrEmu6502GetY(VrEmu6502* vr6502)
{
  return vr6502->iy;
}

/* ------------------------------------------------------------------
 *
 * return the processor status register
 */
VR_EMU_6502_DLLEXPORT uint8_t vrEmu6502GetStatus(VrEmu6502* vr6502)
{
  return vr6502->psreg;
}

/* ------------------------------------------------------------------
 *
 * return the stack pointer register
 */
VR_EMU_6502_DLLEXPORT uint8_t vrEmu6502GetStackPointer(VrEmu6502* vr6502)
{
  return vr6502->sp;
}

/* ------------------------------------------------------------------
 *
 * return the current opcode
 */
VR_EMU_6502_DLLEXPORT uint8_t vrEmu6502GetCurrentOpcode(VrEmu6502* vr6502)
{
  return vr6502->currentOpcode;
}

/* ------------------------------------------------------------------
 *
 * return the opcode cycle
 */
VR_EMU_6502_DLLEXPORT uint8_t vrEmu6502GetOpcodeCycle(VrEmu6502* vr6502)
{
  return vr6502->step;
}


/* ------------------------------------------------------------------
 *  ADDRESS MODES
 * ----------------------------------------------------------------*/

static uint16_t acc(VrEmu6502* vr6502)
{
  return 0xffff;
}

static uint16_t ab(VrEmu6502* vr6502)
{
  uint16_t addr = read16(vr6502, vr6502->pc++);
  ++vr6502->pc;
  return addr;
}

static uint16_t abx(VrEmu6502* vr6502)
{
  uint16_t base = read16(vr6502, vr6502->pc++); ++vr6502->pc;
  uint16_t addr = base + vr6502->ix;
  return addr;
}

static uint16_t aby(VrEmu6502* vr6502)
{
  uint16_t base = read16(vr6502, vr6502->pc++); ++vr6502->pc;
  uint16_t addr = base + vr6502->iy;
  return addr;
}

static uint16_t axp(VrEmu6502* vr6502)
{
  uint16_t base = read16(vr6502, vr6502->pc++); ++vr6502->pc;
  uint16_t addr = base + vr6502->ix;
  pageBoundary(vr6502, addr, base);
  return addr;
}

static uint16_t ayp(VrEmu6502* vr6502)
{
  uint16_t base = read16(vr6502, vr6502->pc++); ++vr6502->pc;
  uint16_t addr = base + vr6502->iy;
  pageBoundary(vr6502, addr, base);
  return addr;
}

static uint16_t imm(VrEmu6502* vr6502)
{
  return vr6502->pc++;
}

static uint16_t imp(VrEmu6502* vr6502)
{
  return 0;
}

static uint16_t ind(VrEmu6502* vr6502)
{
  if (vr6502->model == CPU_6502)
  {
    /* 6502 had a bug where if the low byte was $ff, the high address would come from 
       the same page rather than the next page*/
    uint16_t addr = read16Bug(vr6502, vr6502->pc++);
    return read16(vr6502, addr);
  }

  uint16_t addr = read16(vr6502, vr6502->pc++);
  return read16(vr6502, addr);
}

static uint16_t rel(VrEmu6502* vr6502)
{
  int8_t offset = (int8_t)vr6502->readFn(vr6502->pc);
  return vr6502->pc++ + offset + 1;
}

static uint16_t xin(VrEmu6502* vr6502)
{
  return read16Wrapped(vr6502, vr6502->readFn(vr6502->pc++) + vr6502->ix & 0xff);
}

static uint16_t yin(VrEmu6502* vr6502)
{
  uint16_t base = read16Wrapped(vr6502, vr6502->readFn(vr6502->pc++));
  uint16_t addr = base + vr6502->iy;
  return addr;
}

static uint16_t yip(VrEmu6502* vr6502)
{
  uint16_t base = read16Wrapped(vr6502, vr6502->readFn(vr6502->pc++));
  uint16_t addr = base + vr6502->iy;
  pageBoundary(vr6502, base, addr);
  return addr;
}

static uint16_t zp(VrEmu6502* vr6502)
{
  return vr6502->readFn(vr6502->pc++);
}

static uint16_t zpi(VrEmu6502* vr6502)
{
  return read16Wrapped(vr6502, vr6502->readFn(vr6502->pc++));
}

static uint16_t zpx(VrEmu6502* vr6502)
{
  return (vr6502->readFn(vr6502->pc++) + vr6502->ix) & 0xff;
}

static uint16_t zpy(VrEmu6502* vr6502)
{
  return zpi(vr6502) + vr6502->iy;
}



/* ------------------------------------------------------------------
 *  INSTRUCTIONS
 * ----------------------------------------------------------------*/

static void err(VrEmu6502* vr6502, uint16_t addr)
{
  /* invalid opcode */
}

static void adcd(VrEmu6502* vr6502, uint16_t addr)
{
  uint8_t value = vr6502->readFn(addr);
  uint16_t units = ((uint16_t)vr6502->ac & 0x0f) + (value & 0x0f) + vr6502->ps.car;
  uint16_t tens = ((uint16_t)vr6502->ac & 0xf0) + (value & 0xf0);

  if (units > 0x09)
  {
    tens += 0x10;
    units += 0x06;
  }
  if (tens > 0x90)
  {
    tens += 0x60;
  }

  vr6502->ps.car = (tens & 0xff00) ? 1 : 0;
 
  /* 65c02 takes one extra cycle in decimal mode */
  if (vr6502->model != CPU_6502) ++vr6502->step;
 
  setNZ(vr6502, vr6502->ac = (uint8_t)(tens & 0xf0) | (units & 0x0f));
}

static void adc(VrEmu6502* vr6502, uint16_t addr)
{
  if (vr6502->ps.dec)
  {
    adcd(vr6502, addr);
  }
  else
  {
    uint8_t opr = vr6502->readFn(addr);
    uint16_t result = vr6502->ac + opr + vr6502->ps.car;

    vr6502->ps.car = result > 255;
    vr6502->ps.ovr = ((vr6502->ac ^ result) & (opr ^ result) & 0x80) != 0;

    setNZ(vr6502, vr6502->ac = (uint8_t)result);
  }
}

static void and(VrEmu6502* vr6502, uint16_t addr)
{
  setNZ(vr6502, vr6502->ac &= vr6502->readFn(addr));
}

static void asl(VrEmu6502* vr6502, uint16_t addr)
{
  if (addr == 0xffff)
  {
    vr6502->ps.car = !!(vr6502->ac & 0x80);
    setNZ(vr6502, vr6502->ac <<= 1);
  }
  else
  {
    uint8_t result = vr6502->readFn(addr);
    vr6502->ps.car = !!(result & 0x80);
    setNZ(vr6502, result <<= 1);
    vr6502->writeFn(addr, result);
  }
}

static void bra(VrEmu6502* vr6502, uint16_t addr)
{
  ++vr6502->step; /* extra because we branched */
  pageBoundary(vr6502, vr6502->pc, addr);
  vr6502->pc = addr;
}

static void bcc(VrEmu6502* vr6502, uint16_t addr)
{
  if (!vr6502->ps.car) bra(vr6502, addr);
}

static void bcs(VrEmu6502* vr6502, uint16_t addr)
{
  if (vr6502->ps.car) bra(vr6502, addr);
}

static void beq(VrEmu6502* vr6502, uint16_t addr)
{
  if (vr6502->ps.zer) bra(vr6502, addr);
}

static void bit(VrEmu6502* vr6502, uint16_t addr)
{
  uint8_t val = vr6502->readFn(addr);
  vr6502->ps.ovr = !!(val & 0x40);
  vr6502->ps.neg = !!(val & 0x80);
  vr6502->ps.zer = !(val & vr6502->ac);
}

/* BIT - immediate mode (only affects Z flag) */
static void bti(VrEmu6502* vr6502, uint16_t addr)
{
  uint8_t val = vr6502->readFn(addr);
  vr6502->ps.zer = !(val & vr6502->ac);
}

static void bmi(VrEmu6502* vr6502, uint16_t addr)
{
  if (vr6502->ps.neg) bra(vr6502, addr);
}

static void bne(VrEmu6502* vr6502, uint16_t addr)
{
  if (!vr6502->ps.zer) bra(vr6502, addr);
}

static void bpl(VrEmu6502* vr6502, uint16_t addr)
{
  if (!vr6502->ps.neg) bra(vr6502, addr);
}

static void brk(VrEmu6502* vr6502, uint16_t addr)
{
  push(vr6502, vr6502->pc >> 8);
  push(vr6502, vr6502->pc & 255);
  push(vr6502, vr6502->psreg | 0x30);
  vr6502->ps.irq = 1;

  if (vr6502->model != CPU_6502)
  {
    vr6502->ps.dec = 0;
  }

  vr6502->pc = read16(vr6502, 0xFFFE);
}

static void bvc(VrEmu6502* vr6502, uint16_t addr)
{
  if (!vr6502->ps.ovr) bra(vr6502, addr);
}

static void bvs(VrEmu6502* vr6502, uint16_t addr)
{
  if (vr6502->ps.ovr) bra(vr6502, addr);
}

static void clc(VrEmu6502* vr6502, uint16_t addr)
{
  vr6502->ps.car = 0;
}

static void cld(VrEmu6502* vr6502, uint16_t addr)
{
  vr6502->ps.dec = 0;
}

static void cli(VrEmu6502* vr6502, uint16_t addr)
{
  vr6502->ps.irq = 0;
}

static void clv(VrEmu6502* vr6502, uint16_t addr)
{
  vr6502->ps.ovr = 0;
}

static void cmp(VrEmu6502* vr6502, uint16_t addr)
{
  uint8_t opr = ~vr6502->readFn(addr);
  uint16_t result = vr6502->ac + opr + 1;

  vr6502->ps.car = result > 255;
  setNZ(vr6502, (uint8_t)result);
}

static void cpx(VrEmu6502* vr6502, uint16_t addr)
{
  uint8_t opr = ~vr6502->readFn(addr);
  uint16_t result = vr6502->ix + opr + 1;

  vr6502->ps.car = result > 255;
  setNZ(vr6502, (uint8_t)result);
}

static void cpy(VrEmu6502* vr6502, uint16_t addr)
{
  uint8_t opr = ~vr6502->readFn(addr);
  uint16_t result = vr6502->iy + opr + 1;

  vr6502->ps.car = result > 255;
  setNZ(vr6502, (uint8_t)result);
}

static void dec(VrEmu6502* vr6502, uint16_t addr)
{
  if (addr == 0xffff)
  {
    setNZ(vr6502, --vr6502->ac);
  }
  else
  {
    uint8_t val = vr6502->readFn(addr) - 1;
    setNZ(vr6502, val);
    vr6502->writeFn(addr, val);
  }
}

static void dex(VrEmu6502* vr6502, uint16_t addr)
{
  setNZ(vr6502, --vr6502->ix);
}

static void dey(VrEmu6502* vr6502, uint16_t addr)
{
  setNZ(vr6502, --vr6502->iy);
}

static void eor(VrEmu6502* vr6502, uint16_t addr)
{
  setNZ(vr6502, vr6502->ac ^= vr6502->readFn(addr));
}

static void inc(VrEmu6502* vr6502, uint16_t addr)
{
  if (addr == 0xffff)
  {
    setNZ(vr6502, ++vr6502->ac);
  }
  else
  {
    uint8_t val = vr6502->readFn(addr) + 1;
    setNZ(vr6502, val);
    vr6502->writeFn(addr, val);
  }
}

static void inx(VrEmu6502* vr6502, uint16_t addr)
{
  setNZ(vr6502, ++vr6502->ix);
}

static void iny(VrEmu6502* vr6502, uint16_t addr)
{
  setNZ(vr6502, ++vr6502->iy);
}

static void jmp(VrEmu6502* vr6502, uint16_t addr)
{
  vr6502->pc = addr;
}

static void jsr(VrEmu6502* vr6502, uint16_t addr)
{
  push(vr6502, --vr6502->pc >> 8);
  push(vr6502, vr6502->pc & 0xff);
  vr6502->pc = addr;
}

static void lda(VrEmu6502* vr6502, uint16_t addr)
{
  setNZ(vr6502, vr6502->ac = vr6502->readFn(addr));
}

static void ldx(VrEmu6502* vr6502, uint16_t addr)
{
  setNZ(vr6502, vr6502->ix = vr6502->readFn(addr));
}

static void ldy(VrEmu6502* vr6502, uint16_t addr)
{
  setNZ(vr6502, vr6502->iy = vr6502->readFn(addr));
}

static void lsr(VrEmu6502* vr6502, uint16_t addr)
{
  if (addr == 0xffff)
  {
    vr6502->ps.car = vr6502->ac & 0x01;
    setNZ(vr6502, vr6502->ac >>= 1);
  }
  else
  {
    uint8_t result = vr6502->readFn(addr);
    vr6502->ps.car = result & 0x01;
    setNZ(vr6502, result >>= 1);
    vr6502->writeFn(addr, result);
  }
}

static void nop(VrEmu6502* vr6502, uint16_t addr)
{
  /* nop */
}

static void ora(VrEmu6502* vr6502, uint16_t addr)
{
  setNZ(vr6502, vr6502->ac |= vr6502->readFn(addr));
}

static void pha(VrEmu6502* vr6502, uint16_t addr)
{
  push(vr6502, vr6502->ac);
}

static void php(VrEmu6502* vr6502, uint16_t addr)
{
  push(vr6502, vr6502->psreg);
}

static void phx(VrEmu6502* vr6502, uint16_t addr)
{
  push(vr6502, vr6502->ix);
}

static void phy(VrEmu6502* vr6502, uint16_t addr)
{
  push(vr6502, vr6502->iy);
}

static void pla(VrEmu6502* vr6502, uint16_t addr)
{
  setNZ(vr6502, vr6502->ac = pop(vr6502));
}

static void plp(VrEmu6502* vr6502, uint16_t addr)
{
  vr6502->psreg = pop(vr6502);
}

static void plx(VrEmu6502* vr6502, uint16_t addr)
{
  setNZ(vr6502, vr6502->ix = pop(vr6502));
}

static void ply(VrEmu6502* vr6502, uint16_t addr)
{
  setNZ(vr6502, vr6502->iy = pop(vr6502));
}

static void rol(VrEmu6502* vr6502, uint16_t addr)
{
  if (addr == 0xffff)
  {
    uint8_t tc = !!(vr6502->ac & 0x80);
    vr6502->ac = (vr6502->ac << 1) | vr6502->ps.car;
    vr6502->ps.car = tc;
    setNZ(vr6502, vr6502->ac);
  }
  else
  {
    uint8_t val = vr6502->readFn(addr);
    uint8_t tc = !!(val & 0x80);
    val = (val << 1) | vr6502->ps.car;
    vr6502->ps.car = tc;
    setNZ(vr6502, val);
    vr6502->writeFn(addr, val);
  }
}

static void ror(VrEmu6502* vr6502, uint16_t addr)
{
  if (addr == 0xffff)
  {
    uint8_t tc = vr6502->ac & 0x01;
    vr6502->ac = (vr6502->ac >> 1) | (vr6502->ps.car ? 0x80 : 0x00);
    vr6502->ps.car = tc;
    setNZ(vr6502, vr6502->ac);
  }
  else
  {
    uint8_t val = vr6502->readFn(addr);
    uint8_t tc = val & 0x01;
    val = (val >> 1) | (vr6502->ps.car ? 0x80 : 0x00);
    vr6502->ps.car = tc;
    setNZ(vr6502, val);
    vr6502->writeFn(addr, val);
  }
}

static void rti(VrEmu6502* vr6502, uint16_t addr)
{
  vr6502->inNmi = 0;
  vr6502->psreg = pop(vr6502);
  vr6502->pc = pop(vr6502);
  vr6502->pc |= pop(vr6502) << 8;
}

static void rts(VrEmu6502* vr6502, uint16_t addr)
{
  vr6502->pc = pop(vr6502);
  vr6502->pc |= pop(vr6502) << 8;
  ++vr6502->pc;
}

static void sbcd(VrEmu6502* vr6502, uint16_t addr)
{
  uint8_t value = vr6502->readFn(addr);
  uint16_t result = (uint16_t)vr6502->ac - (value & 0x0f) - !vr6502->ps.car;
  if ((result & 0x0f) > (vr6502->ac & 0x0f))
  {
    result -= 0x06;
  }

  result -= (value & 0xf0);
  if ((result & 0xfff0) > ((uint16_t)vr6502->ac & 0xf0))
  {
    result -= 0x60;
  }
  vr6502->ps.car = result <= (uint16_t)vr6502->ac;
  vr6502->ac= result & 0xff;

  /* 65c02 takes one extra cycle in decimal mode */
  if (vr6502->model != CPU_6502) ++vr6502->step;

  setNZ(vr6502, (uint8_t)result);
}

static void sbc(VrEmu6502* vr6502, uint16_t addr)
{
  if (vr6502->ps.dec)
  {
    sbcd(vr6502, addr);
  }
  else
  {
    uint8_t opr = ~vr6502->readFn(addr);
    uint16_t result = vr6502->ac + opr + vr6502->ps.car;

    vr6502->ps.car = result > 255;
    vr6502->ps.ovr = ((vr6502->ac ^ result) & (opr ^ result) & 0x80) != 0;
    vr6502->ac = (uint8_t)result;

    setNZ(vr6502, (uint8_t)result);
  }
}

static void sec(VrEmu6502* vr6502, uint16_t addr)
{
  vr6502->ps.car = 1;
}

static void sed(VrEmu6502* vr6502, uint16_t addr)
{
  vr6502->ps.dec = 1;
}

static void sei(VrEmu6502* vr6502, uint16_t addr)
{
  vr6502->ps.irq = 1;
}

static void sta(VrEmu6502* vr6502, uint16_t addr)
{
  vr6502->writeFn(addr, vr6502->ac);
}

static void stx(VrEmu6502* vr6502, uint16_t addr)
{
  vr6502->writeFn(addr, vr6502->ix);
}

static void sty(VrEmu6502* vr6502, uint16_t addr)
{
  vr6502->writeFn(addr, vr6502->iy);
}

static void stz(VrEmu6502* vr6502, uint16_t addr)
{
  vr6502->writeFn(addr, 0);
}

static void tax(VrEmu6502* vr6502, uint16_t addr)
{
  setNZ(vr6502, vr6502->ix = vr6502->ac);
}

static void tay(VrEmu6502* vr6502, uint16_t addr)
{
  setNZ(vr6502, vr6502->iy = vr6502->ac);
}

static void tsx(VrEmu6502* vr6502, uint16_t addr)
{
  setNZ(vr6502, vr6502->ix = vr6502->sp);
}

static void txa(VrEmu6502* vr6502, uint16_t addr)
{
  setNZ(vr6502, vr6502->ac = vr6502->ix);
}

static void txs(VrEmu6502* vr6502, uint16_t addr)
{
  setNZ(vr6502, vr6502->sp = vr6502->ix);
}

static void tya(VrEmu6502* vr6502, uint16_t addr)
{
  setNZ(vr6502, vr6502->ac = vr6502->iy);
}

static void trb(VrEmu6502* vr6502, uint16_t addr)
{
  uint8_t temp = vr6502->readFn(addr);
  vr6502->writeFn(addr, temp & ~vr6502->ac);
  vr6502->ps.zer = !(temp & vr6502->ac);
}

static void tsb(VrEmu6502* vr6502, uint16_t addr)
{
  uint8_t temp = vr6502->readFn(addr);
  vr6502->writeFn(addr, temp | vr6502->ac);
  vr6502->ps.zer = !(temp & vr6502->ac);
}

static void rmb(VrEmu6502* vr6502, uint16_t addr, int bitIndex)
{
  vr6502->writeFn(addr, vr6502->readFn(addr) & ~(0x01 << bitIndex));
}

static void rmb0(VrEmu6502* vr6502, uint16_t addr) { rmb(vr6502, addr, 0); }
static void rmb1(VrEmu6502* vr6502, uint16_t addr) { rmb(vr6502, addr, 1); }
static void rmb2(VrEmu6502* vr6502, uint16_t addr) { rmb(vr6502, addr, 2); }
static void rmb3(VrEmu6502* vr6502, uint16_t addr) { rmb(vr6502, addr, 3); }
static void rmb4(VrEmu6502* vr6502, uint16_t addr) { rmb(vr6502, addr, 4); }
static void rmb5(VrEmu6502* vr6502, uint16_t addr) { rmb(vr6502, addr, 5); }
static void rmb6(VrEmu6502* vr6502, uint16_t addr) { rmb(vr6502, addr, 6); }
static void rmb7(VrEmu6502* vr6502, uint16_t addr) { rmb(vr6502, addr, 7); }


static void smb(VrEmu6502* vr6502, uint16_t addr, int bitIndex)
{
  vr6502->writeFn(addr, vr6502->readFn(addr) | (0x01 << bitIndex));
}

static void smb0(VrEmu6502* vr6502, uint16_t addr) { smb(vr6502, addr, 0); }
static void smb1(VrEmu6502* vr6502, uint16_t addr) { smb(vr6502, addr, 1); }
static void smb2(VrEmu6502* vr6502, uint16_t addr) { smb(vr6502, addr, 2); }
static void smb3(VrEmu6502* vr6502, uint16_t addr) { smb(vr6502, addr, 3); }
static void smb4(VrEmu6502* vr6502, uint16_t addr) { smb(vr6502, addr, 4); }
static void smb5(VrEmu6502* vr6502, uint16_t addr) { smb(vr6502, addr, 5); }
static void smb6(VrEmu6502* vr6502, uint16_t addr) { smb(vr6502, addr, 6); }
static void smb7(VrEmu6502* vr6502, uint16_t addr) { smb(vr6502, addr, 7); }

static void bbr(VrEmu6502* vr6502, uint16_t addr, int bitIndex)
{
  uint8_t val = vr6502->readFn(addr);
  if (!(val & (0x01 << bitIndex)))
  {
    vr6502->pc = rel(vr6502);
  }
  else
  {
    ++vr6502->pc;
  }
}

static void bbr0(VrEmu6502* vr6502, uint16_t addr) { bbr(vr6502, addr, 0); }
static void bbr1(VrEmu6502* vr6502, uint16_t addr) { bbr(vr6502, addr, 1); }
static void bbr2(VrEmu6502* vr6502, uint16_t addr) { bbr(vr6502, addr, 2); }
static void bbr3(VrEmu6502* vr6502, uint16_t addr) { bbr(vr6502, addr, 3); }
static void bbr4(VrEmu6502* vr6502, uint16_t addr) { bbr(vr6502, addr, 4); }
static void bbr5(VrEmu6502* vr6502, uint16_t addr) { bbr(vr6502, addr, 5); }
static void bbr6(VrEmu6502* vr6502, uint16_t addr) { bbr(vr6502, addr, 6); }
static void bbr7(VrEmu6502* vr6502, uint16_t addr) { bbr(vr6502, addr, 7); }

static void bbs(VrEmu6502* vr6502, uint16_t addr, int bitIndex)
{
  uint8_t val = vr6502->readFn(addr);
  if ((val & (0x01 << bitIndex)))
  {
    vr6502->pc = rel(vr6502);
  }
  else
  {
    ++vr6502->pc;
  }
}

static void bbs0(VrEmu6502* vr6502, uint16_t addr) { bbs(vr6502, addr, 0); }
static void bbs1(VrEmu6502* vr6502, uint16_t addr) { bbs(vr6502, addr, 1); }
static void bbs2(VrEmu6502* vr6502, uint16_t addr) { bbs(vr6502, addr, 2); }
static void bbs3(VrEmu6502* vr6502, uint16_t addr) { bbs(vr6502, addr, 3); }
static void bbs4(VrEmu6502* vr6502, uint16_t addr) { bbs(vr6502, addr, 4); }
static void bbs5(VrEmu6502* vr6502, uint16_t addr) { bbs(vr6502, addr, 5); }
static void bbs6(VrEmu6502* vr6502, uint16_t addr) { bbs(vr6502, addr, 6); }
static void bbs7(VrEmu6502* vr6502, uint16_t addr) { bbs(vr6502, addr, 7); }

static void wai(VrEmu6502* vr6502, uint16_t addr)
{
  vr6502->wai = 1;
}


#define invalid  { err, imp, 1 }

/* 65c02 guaranteed nops - differed lengths and cycle times */
#define unnop11 { nop, imp, 1 }
#define unnop22 { nop, imm, 2 }
#define unnop23 { nop, imm, 3 }
#define unnop24 { nop, imm, 4 }
#define unnop34 { nop,  ab, 4 }
#define unnop38 { nop,  ab, 8 }


static const vrEmu6502Opcode standard6502[256] = {
/*      |      _0      |      _1      |      _2      |      _3      |      _4      |      _5      |      _6      |      _7      |      _8      |      _9      |      _A      |      _B      |      _C      |      _D      |      _E      |      _F      | */
/* 0_ */ {brk, imp, 7}, {ora, xin, 6},    invalid   ,    invalid   ,    invalid   , {ora,  zp, 3}, {asl,  zp, 5},    invalid   , {php, imp, 3}, {ora, imm, 2}, {asl, acc, 2},    invalid   ,    invalid   , {ora,  ab, 4}, {asl,  ab, 6},    invalid   ,
/* 1_ */ {bpl, rel, 2}, {ora, yip, 5},    invalid   ,    invalid   ,    invalid   , {ora, zpx, 4}, {asl, zpx, 6},    invalid   , {clc, imp, 2}, {ora, ayp, 4},    invalid   ,    invalid   ,    invalid   , {ora, axp, 4}, {asl, abx, 7},    invalid   ,
/* 2_ */ {jsr,  ab, 6}, {and, xin, 6},    invalid   ,    invalid   , {bit,  zp, 3}, {and,  zp, 3}, {rol,  zp, 5},    invalid   , {plp, imp, 4}, {and, imm, 2}, {rol, acc, 2},    invalid   , {bit,  ab, 4}, {and,  ab, 4}, {rol,  ab, 6},    invalid   ,
/* 3_ */ {bmi, rel, 2}, {and, yip, 5},    invalid   ,    invalid   ,    invalid   , {and, zpx, 4}, {rol, zpx, 6},    invalid   , {sec, imp, 2}, {and, ayp, 4},    invalid   ,    invalid   ,    invalid   , {and, axp, 4}, {rol, abx, 7},    invalid   ,
/* 4_ */ {rti, imp, 6}, {eor, xin, 6},    invalid   ,    invalid   ,    invalid   , {eor,  zp, 3}, {lsr,  zp, 5},    invalid   , {pha, imp, 3}, {eor, imm, 2}, {lsr, acc, 2},    invalid   , {jmp,  ab, 3}, {eor,  ab, 4}, {lsr,  ab, 6},    invalid   ,
/* 5_ */ {bvc, rel, 2}, {eor, yip, 5},    invalid   ,    invalid   ,    invalid   , {eor, zpx, 4}, {lsr, zpx, 6},    invalid   , {cli, imp, 2}, {eor, ayp, 4},    invalid   ,    invalid   ,    invalid   , {eor, axp, 4}, {lsr, abx, 7},    invalid   ,
/* 6_ */ {rts, imp, 6}, {adc, xin, 6},    invalid   ,    invalid   ,    invalid   , {adc,  zp, 3}, {ror,  zp, 5},    invalid   , {pla, imp, 4}, {adc, imm, 2}, {ror, acc, 2},    invalid   , {jmp, ind, 5}, {adc,  ab, 4}, {ror,  ab, 6},    invalid   ,
/* 7_ */ {bvs, rel, 2}, {adc, yip, 5},    invalid   ,    invalid   ,    invalid   , {adc, zpx, 4}, {ror, zpx, 6},    invalid   , {sei, imp, 2}, {adc, ayp, 4},    invalid   ,    invalid   ,    invalid   , {adc, axp, 4}, {ror, abx, 7},    invalid   ,
/* 8_ */    invalid   , {sta, xin, 6},    invalid   ,    invalid   , {sty,  zp, 3}, {sta,  zp, 3}, {stx,  zp, 3},    invalid   , {dey, imp, 2},    invalid   , {txa, imp, 2},    invalid   , {sty,  ab, 4}, {sta,  ab, 4}, {stx,  ab, 4},    invalid   ,
/* 9_ */ {bcc, rel, 2}, {sta, yin, 6},    invalid   ,    invalid   , {sty, zpx, 4}, {sta, zpx, 4}, {stx, zpy, 4},    invalid   , {tya, imp, 2}, {sta, aby, 5}, {txs, imp, 2},    invalid   ,    invalid   , {sta, abx, 5},    invalid   ,    invalid   ,
/* A_ */ {ldy, imm, 2}, {lda, xin, 6}, {ldx, imm, 2},    invalid   , {ldy,  zp, 3}, {lda,  zp, 3}, {ldx,  zp, 3},    invalid   , {tay, imp, 2}, {lda, imm, 2}, {tax, imp, 2},    invalid   , {ldy,  ab, 4}, {lda,  ab, 4}, {ldx,  ab, 4},    invalid   ,
/* B_ */ {bcs, rel, 2}, {lda, yip, 5},    invalid   ,    invalid   , {ldy, zpx, 4}, {lda, zpx, 4}, {ldx, zpy, 4},    invalid   , {clv, imp, 2}, {lda, ayp, 4}, {tsx, imp, 2},    invalid   , {ldy, axp, 4}, {lda, axp, 4}, {ldx, ayp, 4},    invalid   ,
/* C_ */ {cpy, imm, 2}, {cmp, xin, 6},    invalid   ,    invalid   , {cpy,  zp, 3}, {cmp,  zp, 3}, {dec,  zp, 5},    invalid   , {iny, imp, 2}, {cmp, imm, 2}, {dex, imp, 2},    invalid   , {cpy,  ab, 4}, {cmp,  ab, 4}, {dec,  ab, 6},    invalid   ,
/* D_ */ {bne, rel, 2}, {cmp, yip, 5},    invalid   ,    invalid   ,    invalid   , {cmp, zpx, 4}, {dec, zpx, 6},    invalid   , {cld, imp, 2}, {cmp, ayp, 4},    invalid   ,    invalid   ,    invalid   , {cmp, ayp, 4}, {dec, abx, 7},    invalid   ,
/* E_ */ {cpx, imm, 2}, {sbc, xin, 6},    invalid   ,    invalid   , {cpx,  zp, 3}, {sbc,  zp, 3}, {inc,  zp, 5},    invalid   , {inx, imp, 2}, {sbc, imm, 2}, {nop, imp, 2},    invalid   , {cpx,  ab, 4}, {sbc,  ab, 4}, {inc,  ab, 6},    invalid   ,
/* F_ */ {beq, rel, 2}, {sbc, yip, 5},    invalid   ,    invalid   ,    invalid   , {sbc, zpx, 4}, {inc, zpx, 6},    invalid   , {sed, imp, 2}, {sbc, ayp, 4},    invalid   ,    invalid   ,    invalid   , {sbc, axp, 4}, {inc, abx, 7},    invalid   };

static const vrEmu6502Opcode standard65c02[256] = {
/*      |      _0      |      _1      |      _2      |      _3      |      _4      |      _5      |      _6      |      _7      |      _8      |      _9      |      _A      |      _B      |      _C      |      _D      |      _E      |      _F      | */
/* 0_ */ {brk, imp, 7}, {ora, xin, 6},    unnop22   ,    unnop11   , {tsb,  zp, 5}, {ora,  zp, 3}, {asl,  zp, 5},    unnop11   , {php, imp, 3}, {ora, imm, 2}, {asl, acc, 2},    unnop11   , {tsb,  ab, 6}, {ora,  ab, 4}, {asl,  ab, 6},    unnop11   ,
/* 1_ */ {bpl, rel, 2}, {ora, yip, 5}, {ora, zpi, 5},    unnop11   , {trb,  zp, 5}, {ora, zpx, 4}, {asl, zpx, 6},    unnop11   , {clc, imp, 2}, {ora, ayp, 4}, {inc, acc, 2},    unnop11   , {trb,  ab, 6}, {ora, axp, 4}, {asl, axp, 6},    unnop11   ,
/* 2_ */ {jsr,  ab, 6}, {and, xin, 6},    unnop22   ,    unnop11   , {bit,  zp, 3}, {and,  zp, 3}, {rol,  zp, 5},    unnop11   , {plp, imp, 4}, {and, imm, 2}, {rol, acc, 2},    unnop11   , {bit,  ab, 4}, {and,  ab, 4}, {rol,  ab, 6},    unnop11   ,
/* 3_ */ {bmi, rel, 2}, {and, yip, 5}, {and, zpi, 5},    unnop11   , {bit, zpx, 4}, {and, zpx, 4}, {rol, zpx, 6},    unnop11   , {sec, imp, 2}, {and, ayp, 4}, {dec, acc, 2},    unnop11   , {bit, abx, 4}, {and, axp, 4}, {rol, axp, 6},    unnop11   ,
/* 4_ */ {rti, imp, 6}, {eor, xin, 6},    unnop22   ,    unnop11   ,    unnop23   , {eor,  zp, 3}, {lsr,  zp, 5},    unnop11   , {pha, imp, 3}, {eor, imm, 2}, {lsr, acc, 2},    unnop11   , {jmp,  ab, 3}, {eor,  ab, 4}, {lsr,  ab, 6},    unnop11   ,
/* 5_ */ {bvc, rel, 2}, {eor, yip, 5}, {eor, zpi, 5},    unnop11   ,    unnop24   , {eor, zpx, 4}, {lsr, zpx, 6},    unnop11   , {cli, imp, 2}, {eor, ayp, 4}, {phy, imp, 3},    unnop11   ,    unnop38   , {eor, axp, 4}, {lsr, axp, 6},    unnop11   ,
/* 6_ */ {rts, imp, 6}, {adc, xin, 6},    unnop22   ,    unnop11   , {stz,  zp, 3}, {adc,  zp, 3}, {ror,  zp, 5},    unnop11   , {pla, imp, 4}, {adc, imm, 2}, {ror, acc, 2},    unnop11   , {jmp, ind, 6}, {adc,  ab, 4}, {ror,  ab, 6},    unnop11   ,
/* 7_ */ {bvs, rel, 2}, {adc, yip, 5}, {adc, zpi, 5},    unnop11   , {stz, zpx, 4}, {adc, zpx, 4}, {ror, zpx, 6},    unnop11   , {sei, imp, 2}, {adc, ayp, 4}, {ply, imp, 4},    unnop11   , {jmp, abx, 6}, {adc, axp, 4}, {ror, axp, 6},    unnop11   ,
/* 8_ */ {bra, rel, 2}, {sta, xin, 6},    unnop22   ,    unnop11   , {sty,  zp, 3}, {sta,  zp, 3}, {stx,  zp, 3},    unnop11   , {dey, imp, 2}, {bti, imm, 2}, {txa, imp, 2},    unnop11   , {sty,  ab, 4}, {sta,  ab, 4}, {stx,  ab, 4},    unnop11   ,
/* 9_ */ {bcc, rel, 2}, {sta, yin, 6}, {sta, zpi, 5},    unnop11   , {sty, zpx, 4}, {sta, zpx, 4}, {stx, zpy, 4},    unnop11   , {tya, imp, 2}, {sta, aby, 5}, {txs, imp, 2},    unnop11   , {stz,  ab, 4}, {sta, abx, 5}, {stz, abx, 5},    unnop11   ,
/* A_ */ {ldy, imm, 2}, {lda, xin, 6}, {ldx, imm, 2},    unnop11   , {ldy,  zp, 3}, {lda,  zp, 3}, {ldx,  zp, 3},    unnop11   , {tay, imp, 2}, {lda, imm, 2}, {tax, imp, 2},    unnop11   , {ldy,  ab, 4}, {lda,  ab, 4}, {ldx,  ab, 4},    unnop11   ,
/* B_ */ {bcs, rel, 2}, {lda, yip, 5}, {lda, zpi, 5},    unnop11   , {ldy, zpx, 4}, {lda, zpx, 4}, {ldx, zpy, 4},    unnop11   , {clv, imp, 2}, {lda, ayp, 4}, {tsx, imp, 2},    unnop11   , {ldy, axp, 4}, {lda, axp, 4}, {ldx, ayp, 4},    unnop11   ,
/* C_ */ {cpy, imm, 2}, {cmp, xin, 6},    unnop22   ,    unnop11   , {cpy,  zp, 3}, {cmp,  zp, 3}, {dec,  zp, 5},    unnop11   , {iny, imp, 2}, {cmp, imm, 2}, {dex, imp, 2},    unnop11   , {cpy,  ab, 4}, {cmp,  ab, 4}, {dec,  ab, 6},    unnop11   ,
/* D_ */ {bne, rel, 2}, {cmp, yip, 5}, {cmp, zpi, 5},    unnop11   ,    unnop24   , {cmp, zpx, 4}, {dec, zpx, 6},    unnop11   , {cld, imp, 2}, {cmp, ayp, 4}, {phx, imp, 3},    unnop11   ,    unnop34   , {cmp, ayp, 4}, {dec, abx, 7},    unnop11   ,
/* E_ */ {cpx, imm, 2}, {sbc, xin, 6},    unnop22   ,    unnop11   , {cpx,  zp, 3}, {sbc,  zp, 3}, {inc,  zp, 5},    unnop11   , {inx, imp, 2}, {sbc, imm, 2}, {nop, imp, 2},    unnop11   , {cpx,  ab, 4}, {sbc,  ab, 4}, {inc,  ab, 6},    unnop11   ,
/* F_ */ {beq, rel, 2}, {sbc, yip, 5}, {sbc, zpi, 5},    unnop11   ,    unnop24   , {sbc, zpx, 4}, {inc, zpx, 6},    unnop11   , {sed, imp, 2}, {sbc, ayp, 4}, {plx, imp, 4},    unnop11   ,    unnop34   , {sbc, axp, 4}, {inc, abx, 7},    unnop11   };

static const vrEmu6502Opcode wdc65c02[256] = {
/*      |      _0      |      _1      |      _2      |      _3      |      _4      |      _5      |      _6      |      _7      |      _8      |      _9      |      _A      |      _B      |      _C      |      _D      |      _E      |      _F      | */
/* 0_ */ {brk, imp, 7}, {ora, xin, 6},    unnop22   ,    unnop11   , {tsb,  zp, 5}, {ora,  zp, 3}, {asl,  zp, 5}, {rmb0, zp, 5}, {php, imp, 3}, {ora, imm, 2}, {asl, acc, 2},    unnop11   , {tsb,  ab, 6}, {ora,  ab, 4}, {asl,  ab, 6}, {bbr0, zp, 5},
/* 1_ */ {bpl, rel, 2}, {ora, yip, 5}, {ora, zpi, 5},    unnop11   , {trb,  zp, 5}, {ora, zpx, 4}, {asl, zpx, 6}, {rmb1, zp, 5}, {clc, imp, 2}, {ora, ayp, 4}, {inc, acc, 2},    unnop11   , {trb,  ab, 6}, {ora, axp, 4}, {asl, axp, 6}, {bbr1, zp, 5},
/* 2_ */ {jsr,  ab, 6}, {and, xin, 6},    unnop22   ,    unnop11   , {bit,  zp, 3}, {and,  zp, 3}, {rol,  zp, 5}, {rmb2, zp, 5}, {plp, imp, 4}, {and, imm, 2}, {rol, acc, 2},    unnop11   , {bit,  ab, 4}, {and,  ab, 4}, {rol,  ab, 6}, {bbr2, zp, 5},
/* 3_ */ {bmi, rel, 2}, {and, yip, 5}, {and, zpi, 5},    unnop11   , {bit, zpx, 4}, {and, zpx, 4}, {rol, zpx, 6}, {rmb3, zp, 5}, {sec, imp, 2}, {and, ayp, 4}, {dec, acc, 2},    unnop11   , {bit, abx, 4}, {and, axp, 4}, {rol, axp, 6}, {bbr3, zp, 5},
/* 4_ */ {rti, imp, 6}, {eor, xin, 6},    unnop22   ,    unnop11   ,    unnop23   , {eor,  zp, 3}, {lsr,  zp, 5}, {rmb4, zp, 5}, {pha, imp, 3}, {eor, imm, 2}, {lsr, acc, 2},    unnop11   , {jmp,  ab, 3}, {eor,  ab, 4}, {lsr,  ab, 6}, {bbr4, zp, 5},
/* 5_ */ {bvc, rel, 2}, {eor, yip, 5}, {eor, zpi, 5},    unnop11   ,    unnop24   , {eor, zpx, 4}, {lsr, zpx, 6}, {rmb5, zp, 5}, {cli, imp, 2}, {eor, ayp, 4}, {phy, imp, 3},    unnop11   ,    unnop38   , {eor, axp, 4}, {lsr, axp, 6}, {bbr5, zp, 5},
/* 6_ */ {rts, imp, 6}, {adc, xin, 6},    unnop22   ,    unnop11   , {stz,  zp, 3}, {adc,  zp, 3}, {ror,  zp, 5}, {rmb6, zp, 5}, {pla, imp, 4}, {adc, imm, 2}, {ror, acc, 2},    unnop11   , {jmp, ind, 6}, {adc,  ab, 4}, {ror,  ab, 6}, {bbr6, zp, 5},
/* 7_ */ {bvs, rel, 2}, {adc, yip, 5}, {adc, zpi, 5},    unnop11   , {stz, zpx, 4}, {adc, zpx, 4}, {ror, zpx, 6}, {rmb7, zp, 5}, {sei, imp, 2}, {adc, ayp, 4}, {ply, imp, 4},    unnop11   , {jmp, abx, 6}, {adc, axp, 4}, {ror, axp, 6}, {bbr7, zp, 5},
/* 8_ */ {bra, rel, 2}, {sta, xin, 6},    unnop22   ,    unnop11   , {sty,  zp, 3}, {sta,  zp, 3}, {stx,  zp, 3}, {smb0, zp, 5}, {dey, imp, 2}, {bti, imm, 2}, {txa, imp, 2},    unnop11   , {sty,  ab, 4}, {sta,  ab, 4}, {stx,  ab, 4}, {bbs0, zp, 5},
/* 9_ */ {bcc, rel, 2}, {sta, yin, 6}, {sta, zpi, 5},    unnop11   , {sty, zpx, 4}, {sta, zpx, 4}, {stx, zpy, 4}, {smb1, zp, 5}, {tya, imp, 2}, {sta, aby, 5}, {txs, imp, 2},    unnop11   , {stz,  ab, 4}, {sta, abx, 5}, {stz, abx, 5}, {bbs1, zp, 5},
/* A_ */ {ldy, imm, 2}, {lda, xin, 6}, {ldx, imm, 2},    unnop11   , {ldy,  zp, 3}, {lda,  zp, 3}, {ldx,  zp, 3}, {smb2, zp, 5}, {tay, imp, 2}, {lda, imm, 2}, {tax, imp, 2},    unnop11   , {ldy,  ab, 4}, {lda,  ab, 4}, {ldx,  ab, 4}, {bbs2, zp, 5},
/* B_ */ {bcs, rel, 2}, {lda, yip, 5}, {lda, zpi, 5},    unnop11   , {ldy, zpx, 4}, {lda, zpx, 4}, {ldx, zpy, 4}, {smb3, zp, 5}, {clv, imp, 2}, {lda, ayp, 4}, {tsx, imp, 2},    unnop11   , {ldy, axp, 4}, {lda, axp, 4}, {ldx, ayp, 4}, {bbs3, zp, 5},
/* C_ */ {cpy, imm, 2}, {cmp, xin, 6},    unnop22   ,    unnop11   , {cpy,  zp, 3}, {cmp,  zp, 3}, {dec,  zp, 5}, {smb4, zp, 5}, {iny, imp, 2}, {cmp, imm, 2}, {dex, imp, 2}, {wai, imp, 3}, {cpy,  ab, 4}, {cmp,  ab, 4}, {dec,  ab, 6}, {bbs4, zp, 5},
/* D_ */ {bne, rel, 2}, {cmp, yip, 5}, {cmp, zpi, 5},    unnop11   ,    unnop24   , {cmp, zpx, 4}, {dec, zpx, 6}, {smb5, zp, 5}, {cld, imp, 2}, {cmp, ayp, 4}, {phx, imp, 3},    unnop11   ,    unnop34   , {cmp, ayp, 4}, {dec, abx, 7}, {bbs5, zp, 5},
/* E_ */ {cpx, imm, 2}, {sbc, xin, 6},    unnop22   ,    unnop11   , {cpx,  zp, 3}, {sbc,  zp, 3}, {inc,  zp, 5}, {smb6, zp, 5}, {inx, imp, 2}, {sbc, imm, 2}, {nop, imp, 2},    unnop11   , {cpx,  ab, 4}, {sbc,  ab, 4}, {inc,  ab, 6}, {bbs6, zp, 5},
/* F_ */ {beq, rel, 2}, {sbc, yip, 5}, {sbc, zpi, 5},    unnop11   ,    unnop24   , {sbc, zpx, 4}, {inc, zpx, 6}, {smb7, zp, 5}, {sed, imp, 2}, {sbc, ayp, 4}, {plx, imp, 4},    unnop11   ,    unnop34   , {sbc, axp, 4}, {inc, abx, 7}, {bbs7, zp, 5}};

static const vrEmu6502Opcode r65c02[256] = {
/*      |      _0      |      _1      |      _2      |      _3      |      _4      |      _5      |      _6      |      _7      |      _8      |      _9      |      _A      |      _B      |      _C      |      _D      |      _E      |      _F      | */
/* 0_ */ {brk, imp, 7}, {ora, xin, 6},    unnop22   ,    unnop11   , {tsb,  zp, 5}, {ora,  zp, 3}, {asl,  zp, 5}, {rmb0, zp, 5}, {php, imp, 3}, {ora, imm, 2}, {asl, acc, 2},    unnop11   , {tsb,  ab, 6}, {ora,  ab, 4}, {asl,  ab, 6}, {bbr0, zp, 5},
/* 1_ */ {bpl, rel, 2}, {ora, yip, 5}, {ora, zpi, 5},    unnop11   , {trb,  zp, 5}, {ora, zpx, 4}, {asl, zpx, 6}, {rmb1, zp, 5}, {clc, imp, 2}, {ora, ayp, 4}, {inc, acc, 2},    unnop11   , {trb,  ab, 6}, {ora, axp, 4}, {asl, axp, 6}, {bbr1, zp, 5},
/* 2_ */ {jsr,  ab, 6}, {and, xin, 6},    unnop22   ,    unnop11   , {bit,  zp, 3}, {and,  zp, 3}, {rol,  zp, 5}, {rmb2, zp, 5}, {plp, imp, 4}, {and, imm, 2}, {rol, acc, 2},    unnop11   , {bit,  ab, 4}, {and,  ab, 4}, {rol,  ab, 6}, {bbr2, zp, 5},
/* 3_ */ {bmi, rel, 2}, {and, yip, 5}, {and, zpi, 5},    unnop11   , {bit, zpx, 4}, {and, zpx, 4}, {rol, zpx, 6}, {rmb3, zp, 5}, {sec, imp, 2}, {and, ayp, 4}, {dec, acc, 2},    unnop11   , {bit, abx, 4}, {and, axp, 4}, {rol, axp, 6}, {bbr3, zp, 5},
/* 4_ */ {rti, imp, 6}, {eor, xin, 6},    unnop22   ,    unnop11   ,    unnop23   , {eor,  zp, 3}, {lsr,  zp, 5}, {rmb4, zp, 5}, {pha, imp, 3}, {eor, imm, 2}, {lsr, acc, 2},    unnop11   , {jmp,  ab, 3}, {eor,  ab, 4}, {lsr,  ab, 6}, {bbr4, zp, 5},
/* 5_ */ {bvc, rel, 2}, {eor, yip, 5}, {eor, zpi, 5},    unnop11   ,    unnop24   , {eor, zpx, 4}, {lsr, zpx, 6}, {rmb5, zp, 5}, {cli, imp, 2}, {eor, ayp, 4}, {phy, imp, 3},    unnop11   ,    unnop38   , {eor, axp, 4}, {lsr, axp, 6}, {bbr5, zp, 5},
/* 6_ */ {rts, imp, 6}, {adc, xin, 6},    unnop22   ,    unnop11   , {stz,  zp, 3}, {adc,  zp, 3}, {ror,  zp, 5}, {rmb6, zp, 5}, {pla, imp, 4}, {adc, imm, 2}, {ror, acc, 2},    unnop11   , {jmp, ind, 6}, {adc,  ab, 4}, {ror,  ab, 6}, {bbr6, zp, 5},
/* 7_ */ {bvs, rel, 2}, {adc, yip, 5}, {adc, zpi, 5},    unnop11   , {stz, zpx, 4}, {adc, zpx, 4}, {ror, zpx, 6}, {rmb7, zp, 5}, {sei, imp, 2}, {adc, ayp, 4}, {ply, imp, 4},    unnop11   , {jmp, abx, 6}, {adc, axp, 4}, {ror, axp, 6}, {bbr7, zp, 5},
/* 8_ */ {bra, rel, 2}, {sta, xin, 6},    unnop22   ,    unnop11   , {sty,  zp, 3}, {sta,  zp, 3}, {stx,  zp, 3}, {smb0, zp, 5}, {dey, imp, 2}, {bti, imm, 2}, {txa, imp, 2},    unnop11   , {sty,  ab, 4}, {sta,  ab, 4}, {stx,  ab, 4}, {bbs0, zp, 5},
/* 9_ */ {bcc, rel, 2}, {sta, yin, 6}, {sta, zpi, 5},    unnop11   , {sty, zpx, 4}, {sta, zpx, 4}, {stx, zpy, 4}, {smb1, zp, 5}, {tya, imp, 2}, {sta, aby, 5}, {txs, imp, 2},    unnop11   , {stz,  ab, 4}, {sta, abx, 5}, {stz, abx, 5}, {bbs1, zp, 5},
/* A_ */ {ldy, imm, 2}, {lda, xin, 6}, {ldx, imm, 2},    unnop11   , {ldy,  zp, 3}, {lda,  zp, 3}, {ldx,  zp, 3}, {smb2, zp, 5}, {tay, imp, 2}, {lda, imm, 2}, {tax, imp, 2},    unnop11   , {ldy,  ab, 4}, {lda,  ab, 4}, {ldx,  ab, 4}, {bbs2, zp, 5},
/* B_ */ {bcs, rel, 2}, {lda, yip, 5}, {lda, zpi, 5},    unnop11   , {ldy, zpx, 4}, {lda, zpx, 4}, {ldx, zpy, 4}, {smb3, zp, 5}, {clv, imp, 2}, {lda, ayp, 4}, {tsx, imp, 2},    unnop11   , {ldy, axp, 4}, {lda, axp, 4}, {ldx, ayp, 4}, {bbs3, zp, 5},
/* C_ */ {cpy, imm, 2}, {cmp, xin, 6},    unnop22   ,    unnop11   , {cpy,  zp, 3}, {cmp,  zp, 3}, {dec,  zp, 5}, {smb4, zp, 5}, {iny, imp, 2}, {cmp, imm, 2}, {dex, imp, 2},    unnop11   , {cpy,  ab, 4}, {cmp,  ab, 4}, {dec,  ab, 6}, {bbs4, zp, 5},
/* D_ */ {bne, rel, 2}, {cmp, yip, 5}, {cmp, zpi, 5},    unnop11   ,    unnop24   , {cmp, zpx, 4}, {dec, zpx, 6}, {smb5, zp, 5}, {cld, imp, 2}, {cmp, ayp, 4}, {phx, imp, 3},    unnop11   ,    unnop34   , {cmp, ayp, 4}, {dec, abx, 7}, {bbs5, zp, 5},
/* E_ */ {cpx, imm, 2}, {sbc, xin, 6},    unnop22   ,    unnop11   , {cpx,  zp, 3}, {sbc,  zp, 3}, {inc,  zp, 5}, {smb6, zp, 5}, {inx, imp, 2}, {sbc, imm, 2}, {nop, imp, 2},    unnop11   , {cpx,  ab, 4}, {sbc,  ab, 4}, {inc,  ab, 6}, {bbs6, zp, 5},
/* F_ */ {beq, rel, 2}, {sbc, yip, 5}, {sbc, zpi, 5},    unnop11   ,    unnop24   , {sbc, zpx, 4}, {inc, zpx, 6}, {smb7, zp, 5}, {sed, imp, 2}, {sbc, ayp, 4}, {plx, imp, 4},    unnop11   ,    unnop34   , {sbc, axp, 4}, {inc, abx, 7}, {bbs7, zp, 5}};
