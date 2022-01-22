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
  uint8_t lastOpcode;
  uint8_t currentOpcode;

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

  vrEmu6502Opcode *opcodes;
};


static vrEmu6502Opcode  standard6502[256];
static vrEmu6502Opcode  standard65c02[256];
static vrEmu6502Opcode  wdc65c02[256];
static vrEmu6502Opcode  r65c02[256];



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
    vr6502->opcodes = standard65c02;
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
  }
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
      push(vr6502, vr6502->pc >> 8);
      push(vr6502, vr6502->pc & 0xff);
      push(vr6502, vr6502->psreg | 0x30);
      vr6502->ps.irq = 1;
      vr6502->pc = read16(vr6502, 0xfffa);
    }
    else if (vr6502->intPin == IntRequested && !vr6502->ps.irq)
    {
      push(vr6502, vr6502->pc >> 8);
      push(vr6502, vr6502->pc & 0xff);
      push(vr6502, vr6502->psreg | 0x30);
      vr6502->ps.irq = 1;
      vr6502->pc = read16(vr6502, 0xfffe);
    }

    vr6502->lastOpcode = vr6502->currentOpcode;
    vr6502->currentOpcode = vr6502->readFn(vr6502->pc++);

    if (vr6502->currentOpcode == 0x20)
    {
      int jj = 0;
    }

    vrEmu6502Opcode* opcode = &vr6502->opcodes[vr6502->currentOpcode];

    opcode->instruction(vr6502, opcode->addrMode(vr6502));
    vr6502->step = opcode->cycles;
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
 *
 * return the last opcode
 */
VR_EMU_6502_DLLEXPORT uint8_t vrEmu6502GetLastOpcode(VrEmu6502* vr6502)
{
  return vr6502->lastOpcode;
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
  pageBoundary(vr6502, addr, base);
  return addr;
}

static uint16_t aby(VrEmu6502* vr6502)
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
  return (vr6502->readFn(vr6502->pc++) + vr6502->iy) & 0xff;
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

#define invalid { err, imp, 1 }


vrEmu6502Opcode  standard6502[256] = {
/*      |      _0      |      _1      |      _2      |      _3      |      _4      |      _5      |      _6      |      _7      |      _8      |      _9      |      _A      |      _B      |      _C      |      _D      |      _E      |      _F      | */
/* 0_ */ {brk, imp, 3}, {ora, xin, 3},    invalid   ,    invalid   , {err, imp, 3}, {ora,  zp, 3}, {asl,  zp, 3},    invalid   , {php, imp, 3}, {ora, imm, 3}, {asl, acc, 3},    invalid   ,    invalid   , {ora,  ab, 3}, {asl,  ab, 3},    invalid   ,
/* 1_ */ {bpl, rel, 3}, {ora, yin, 3},    invalid   ,    invalid   , {err, imp, 3}, {ora, zpx, 3}, {asl, zpx, 3},    invalid   , {clc, imp, 3}, {ora, aby, 3}, {err, imp, 3},    invalid   ,    invalid   , {ora, abx, 3}, {asl, abx, 3},    invalid   ,
/* 2_ */ {jsr,  ab, 3}, {and, xin, 3},    invalid   ,    invalid   , {bit,  zp, 3}, {and,  zp, 3}, {rol,  zp, 3},    invalid   , {plp, imp, 3}, {and, imm, 3}, {rol, acc, 1},    invalid   , {bit,  ab, 1}, {and,  ab, 3}, {rol,  ab, 3},    invalid   ,
/* 3_ */ {bmi, rel, 3}, {and, yin, 3},    invalid   ,    invalid   ,    invalid   , {and, zpx, 3}, {rol, zpx, 3},    invalid   , {sec, imp, 3}, {and, aby, 3},    invalid   ,    invalid   ,    invalid   , {and, abx, 3}, {rol, abx, 3},    invalid   ,
/* 4_ */ {rti, imp, 3}, {eor, xin, 3},    invalid   ,    invalid   ,    invalid   , {eor,  zp, 3}, {lsr,  zp, 3},    invalid   , {pha, imp, 3}, {eor, imm, 3}, {lsr, acc, 1},    invalid   , {jmp,  ab, 1}, {eor,  ab, 3}, {lsr,  ab, 3},    invalid   ,
/* 5_ */ {bvc, rel, 3}, {eor, yin, 3},    invalid   ,    invalid   ,    invalid   , {eor, zpx, 3}, {lsr, zpx, 3},    invalid   , {cli, imp, 3}, {eor, aby, 3},    invalid   ,    invalid   ,    invalid   , {eor, abx, 3}, {lsr, abx, 3},    invalid   ,
/* 6_ */ {rts, imp, 3}, {adc, xin, 3},    invalid   ,    invalid   ,    invalid   , {adc,  zp, 3}, {ror,  zp, 3},    invalid   , {pla, imp, 3}, {adc, imm, 3}, {ror, acc, 1},    invalid   , {jmp, ind, 1}, {adc,  ab, 3}, {ror,  ab, 3},    invalid   ,
/* 7_ */ {bvs, rel, 3}, {adc, yin, 3},    invalid   ,    invalid   ,    invalid   , {adc, zpx, 3}, {ror, zpx, 3},    invalid   , {sei, imp, 3}, {adc, aby, 3},    invalid   ,    invalid   ,    invalid   , {adc, abx, 3}, {ror, abx, 3},    invalid   ,
/* 8_ */ {err, imp, 3}, {sta, xin, 3},    invalid   ,    invalid   , {sty,  zp, 1}, {sta,  zp, 3}, {stx,  zp, 3},    invalid   , {dey, imp, 3}, {err, imp, 3}, {txa, imp, 2},    invalid   , {sty,  ab, 1}, {sta,  ab, 3}, {stx,  ab, 3},    invalid   ,
/* 9_ */ {bcc, rel, 3}, {sta, yin, 3},    invalid   ,    invalid   , {sty, zpx, 1}, {sta, zpx, 3}, {stx, zpy, 3},    invalid   , {tya, imp, 2}, {sta, aby, 3}, {txs, imp, 2},    invalid   ,    invalid   , {sta, abx, 3}, {err, imp, 3},    invalid   ,
/* A_ */ {ldy, imm, 3}, {lda, xin, 3}, {ldx, imm, 1},    invalid   , {ldy,  zp, 1}, {lda,  zp, 3}, {ldx,  zp, 3},    invalid   , {tay, imp, 2}, {lda, imm, 3}, {tax, imp, 2},    invalid   , {ldy,  ab, 1}, {lda,  ab, 3}, {ldx,  ab, 3},    invalid   ,
/* B_ */ {bcs, rel, 3}, {lda, yin, 3},    invalid   ,    invalid   , {ldy, zpx, 1}, {lda, zpx, 3}, {ldx, zpy, 3},    invalid   , {clv, imp, 3}, {lda, aby, 3}, {tsx, imp, 2},    invalid   , {ldy, abx, 1}, {lda, abx, 3}, {ldx, abx, 3},    invalid   ,
/* C_ */ {cpy, imm, 3}, {cmp, xin, 3},    invalid   ,    invalid   , {cpy,  zp, 1}, {cmp,  zp, 3}, {dec,  zp, 3},    invalid   , {iny, imp, 3}, {cmp, imm, 3}, {dex, imp, 1},    invalid   , {cpy,  ab, 1}, {cmp,  ab, 3}, {dec,  ab, 3},    invalid   ,
/* D_ */ {bne, rel, 3}, {cmp, yin, 3},    invalid   ,    invalid   ,    invalid   , {cmp, zpx, 3}, {dec, zpx, 3},    invalid   , {cld, imp, 3}, {cmp, aby, 3},    invalid   ,    invalid   ,    invalid   , {cmp, abx, 3}, {dec, abx, 3},    invalid   ,
/* E_ */ {cpx, imm, 3}, {sbc, xin, 3},    invalid   ,    invalid   , {cpx,  zp, 1}, {sbc,  zp, 3}, {inc,  zp, 3},    invalid   , {inx, imp, 3}, {sbc, imm, 3}, {nop, imp, 1},    invalid   , {cpx,  ab, 1}, {sbc,  ab, 3}, {inc,  ab, 3},    invalid   ,
/* F_ */ {beq, rel, 3}, {sbc, yin, 3},    invalid   ,    invalid   ,    invalid   , {sbc, zpx, 3}, {inc, zpx, 3},    invalid   , {sed, imp, 3}, {sbc, aby, 3},    invalid   ,    invalid   ,    invalid   , {sbc, abx, 3}, {inc, abx, 3},    invalid   };

vrEmu6502Opcode  standard65c02[256] = {
/*      |      _0      |      _1      |      _2      |      _3      |      _4      |      _5      |      _6      |      _7      |      _8      |      _9      |      _A      |      _B      |      _C      |      _D      |      _E      |      _F      | */
/* 0_ */ {brk, imp, 3}, {ora, xin, 3},    invalid   ,    invalid   , {err, imp, 3}, {ora,  zp, 3}, {asl,  zp, 3},    invalid   , {php, imp, 3}, {ora, imm, 3}, {asl, acc, 3},    invalid   ,    invalid   , {ora,  ab, 3}, {asl,  ab, 3},    invalid   ,
/* 1_ */ {bpl, rel, 3}, {ora, yin, 3},    invalid   ,    invalid   , {err, imp, 3}, {ora, zpx, 3}, {asl, zpx, 3},    invalid   , {clc, imp, 3}, {ora, aby, 3}, {err, imp, 3},    invalid   ,    invalid   , {ora, abx, 3}, {asl, abx, 3},    invalid   ,
/* 2_ */ {jsr,  ab, 3}, {and, xin, 3},    invalid   ,    invalid   , {bit,  zp, 3}, {and,  zp, 3}, {rol,  zp, 3},    invalid   , {plp, imp, 3}, {and, imm, 3}, {rol, acc, 1},    invalid   , {bit,  ab, 1}, {and,  ab, 3}, {rol,  ab, 3},    invalid   ,
/* 3_ */ {bmi, rel, 3}, {and, yin, 3},    invalid   ,    invalid   ,    invalid   , {and, zpx, 3}, {rol, zpx, 3},    invalid   , {sec, imp, 3}, {and, aby, 3},    invalid   ,    invalid   ,    invalid   , {and, abx, 3}, {rol, abx, 3},    invalid   ,
/* 4_ */ {rti, imp, 3}, {eor, xin, 3},    invalid   ,    invalid   ,    invalid   , {eor,  zp, 3}, {lsr,  zp, 3},    invalid   , {pha, imp, 3}, {eor, imm, 3}, {lsr, acc, 1},    invalid   , {jmp,  ab, 1}, {eor,  ab, 3}, {lsr,  ab, 3},    invalid   ,
/* 5_ */ {bvc, rel, 3}, {eor, yin, 3},    invalid   ,    invalid   ,    invalid   , {eor, zpx, 3}, {lsr, zpx, 3},    invalid   , {cli, imp, 3}, {eor, aby, 3}, {phy, imp, 3},    invalid   ,    invalid   , {eor, abx, 3}, {lsr, abx, 3},    invalid   ,
/* 6_ */ {rts, imp, 3}, {adc, xin, 3},    invalid   ,    invalid   , {stz,  zp, 1}, {adc,  zp, 3}, {ror,  zp, 3},    invalid   , {pla, imp, 3}, {adc, imm, 3}, {ror, acc, 1},    invalid   , {jmp, ind, 1}, {adc,  ab, 3}, {ror,  ab, 3},    invalid   ,
/* 7_ */ {bvs, rel, 3}, {adc, yin, 3},    invalid   ,    invalid   , {stz, zpx, 1}, {adc, zpx, 3}, {ror, zpx, 3},    invalid   , {sei, imp, 3}, {adc, aby, 3}, {ply, imp, 3},    invalid   ,    invalid   , {adc, abx, 3}, {ror, abx, 3},    invalid   ,
/* 8_ */ {bra, rel, 3}, {sta, xin, 3},    invalid   ,    invalid   , {sty,  zp, 1}, {sta,  zp, 3}, {stx,  zp, 3},    invalid   , {dey, imp, 3}, {err, imp, 3}, {txa, imp, 2},    invalid   , {sty,  ab, 1}, {sta,  ab, 3}, {stx,  ab, 3},    invalid   ,
/* 9_ */ {bcc, rel, 3}, {sta, yin, 3},    invalid   ,    invalid   , {sty, zpx, 1}, {sta, zpx, 3}, {stx, zpy, 3},    invalid   , {tya, imp, 2}, {sta, aby, 3}, {txs, imp, 2},    invalid   , {stz,  ab, 1}, {sta, abx, 3}, {stz, abx, 1},    invalid   ,
/* A_ */ {ldy, imm, 3}, {lda, xin, 3}, {ldx, imm, 1},    invalid   , {ldy,  zp, 1}, {lda,  zp, 3}, {ldx,  zp, 3},    invalid   , {tay, imp, 2}, {lda, imm, 3}, {tax, imp, 2},    invalid   , {ldy,  ab, 1}, {lda,  ab, 3}, {ldx,  ab, 3},    invalid   ,
/* B_ */ {bcs, rel, 3}, {lda, yin, 3},    invalid   ,    invalid   , {ldy, zpx, 1}, {lda, zpx, 3}, {ldx, zpy, 3},    invalid   , {clv, imp, 3}, {lda, aby, 3}, {tsx, imp, 2},    invalid   , {ldy, abx, 1}, {lda, abx, 3}, {ldx, abx, 3},    invalid   ,
/* C_ */ {cpy, imm, 3}, {cmp, xin, 3},    invalid   ,    invalid   , {cpy,  zp, 1}, {cmp,  zp, 3}, {dec,  zp, 3},    invalid   , {iny, imp, 3}, {cmp, imm, 3}, {dex, imp, 1},    invalid   , {cpy,  ab, 1}, {cmp,  ab, 3}, {dec,  ab, 3},    invalid   ,
/* D_ */ {bne, rel, 3}, {cmp, yin, 3},    invalid   ,    invalid   ,    invalid   , {cmp, zpx, 3}, {dec, zpx, 3},    invalid   , {cld, imp, 3}, {cmp, aby, 3}, {phx, imp, 3},    invalid   ,    invalid   , {cmp, abx, 3}, {dec, abx, 3},    invalid   ,
/* E_ */ {cpx, imm, 3}, {sbc, xin, 3},    invalid   ,    invalid   , {cpx,  zp, 1}, {sbc,  zp, 3}, {inc,  zp, 3},    invalid   , {inx, imp, 3}, {sbc, imm, 3}, {nop, imp, 1},    invalid   , {cpx,  ab, 1}, {sbc,  ab, 3}, {inc,  ab, 3},    invalid   ,
/* F_ */ {beq, rel, 3}, {sbc, yin, 3},    invalid   ,    invalid   ,    invalid   , {sbc, zpx, 3}, {inc, zpx, 3},    invalid   , {sed, imp, 3}, {sbc, aby, 3}, {plx, imp, 3}, {err, imp, 1}, {err, imp, 1}, {sbc, abx, 3}, {inc, abx, 3}, {err, imp, 1}};