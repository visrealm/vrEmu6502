## Test runner

[vrEmu6502Test.c](vrEmu6502Test.c)
* The source file for the test runner.
* It can be built using the solution in the [msvc](../msvc) folder.
* The test runner binary (Windows) is included in the [bin](../bin) directory. 

[test.bat](test.bat)
* All included tests can be run using `test.bat`. 


### Options:

The test runner accepts Intel HEX files provided by the Klauss Dormann tests (located in the [programs](programs) folder)

```Usage:
vrEmu6502Test [OPTION...] <testfile.hex>

Options:
  -c                output instruction count
  -f <lines>        filter output to every #<lines> lines
  -h                output help and exit
  -m <from>[:<to>]  output given memory address or range
  -q                quiet mode - only print report
  -r <addr>         override run address
  -v [<count>]      verbose output from instruction #<count>
```

### Example output:

`..\bin\vrEmu6502Test -q programs\65C02_extended_opcodes_test.hex  -c -m 0x08:0x0f -v 21986970`

```
  -------------------------------------
          vrEmu6502 Test Runner
  -------------------------------------
    Copyright (c) 2022 Troy Schrapel
  https://github.com/visrealm/vrEmu6502
  -------------------------------------

Running test:                "programs\65C02_extended_opcodes_test.hex"

Options:
  Output instruction count:  Yes
  Output filtering:          Quiet until #21986950
  Output memory:             $0008 - $000f
  Start address:             $0400


Instr #     PC     Instruction    Acc    InX    InY    SP        Status      $0008 - $000f

#21986970   $2496: lda #$99      A: $99 X: $0e Y: $ff SP: $ff F: $f8 NVD...  $00 $00 $bd $ad $01 $00 $00 $00
#21986971   $2498: sta $0d       A: $99 X: $0e Y: $ff SP: $ff F: $f8 NVD...  $00 $00 $bd $ad $01 $99 $00 $00
#21986972   $249a: lda $0e       A: $00 X: $0e Y: $ff SP: $ff F: $7a .VD..Z  $00 $00 $bd $ad $01 $99 $00 $00
#21986973   $249c: beq $24d7     A: $00 X: $0e Y: $ff SP: $ff F: $7a .VD..Z  $00 $00 $bd $ad $01 $99 $00 $00
#21986974   $24d7: cpx #$0e      A: $00 X: $0e Y: $ff SP: $ff F: $7b .VD.CZ  $00 $00 $bd $ad $01 $99 $00 $00
#21986975   $24d9: bne $24d9     A: $00 X: $0e Y: $ff SP: $ff F: $7b .VD.CZ  $00 $00 $bd $ad $01 $99 $00 $00
#21986976   $24db: cpy #$ff      A: $00 X: $0e Y: $ff SP: $ff F: $7b .VD.CZ  $00 $00 $bd $ad $01 $99 $00 $00
#21986977   $24dd: bne $24dd     A: $00 X: $0e Y: $ff SP: $ff F: $7b .VD.CZ  $00 $00 $bd $ad $01 $99 $00 $00
#21986978   $24df: tsx           A: $00 X: $ff Y: $ff SP: $ff F: $f9 NVD.C.  $00 $00 $bd $ad $01 $99 $00 $00
#21986979   $24e0: cpx #$ff      A: $00 X: $ff Y: $ff SP: $ff F: $7b .VD.CZ  $00 $00 $bd $ad $01 $99 $00 $00
#21986980   $24e2: bne $24e2     A: $00 X: $ff Y: $ff SP: $ff F: $7b .VD.CZ  $00 $00 $bd $ad $01 $99 $00 $00
#21986981   $24e4: cld           A: $00 X: $ff Y: $ff SP: $ff F: $73 .V..CZ  $00 $00 $bd $ad $01 $99 $00 $00
#21986982   $24e5: lda $0202     A: $15 X: $ff Y: $ff SP: $ff F: $71 .V..C.  $00 $00 $bd $ad $01 $99 $00 $00
#21986983   $24e8: cmp #$15      A: $15 X: $ff Y: $ff SP: $ff F: $73 .V..CZ  $00 $00 $bd $ad $01 $99 $00 $00
#21986984   $24ea: bne $24ea     A: $15 X: $ff Y: $ff SP: $ff F: $73 .V..CZ  $00 $00 $bd $ad $01 $99 $00 $00
#21986985   $24ec: lda #$f0      A: $f0 X: $ff Y: $ff SP: $ff F: $f1 NV..C.  $00 $00 $bd $ad $01 $99 $00 $00
#21986986   $24ee: sta $0202     A: $f0 X: $ff Y: $ff SP: $ff F: $f1 NV..C.  $00 $00 $bd $ad $01 $99 $00 $00
#21986987   $24f1: stp           A: $f0 X: $ff Y: $ff SP: $ff F: $f1 NV..C.  $00 $00 $bd $ad $01 $99 $00 $00

  -------------------------------------
  "programs\65C02_extended_opcodes_test.hex"

  Total instructions executed: 21986987
  Total clock cycles:          66905005

  Test completed:              PASSED
```
  
