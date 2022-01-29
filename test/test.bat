@echo off

pushd "%~dp0"

..\bin\vrEmu6502Test -quiet -cpu 6502 programs\6502_functional_test.hex 
IF NOT ERRORLEVEL 0 GOTO endtests

..\bin\vrEmu6502Test -quiet -cpu 65c02 programs\6502_functional_test.hex 
IF NOT ERRORLEVEL 0 GOTO endtests

..\bin\vrEmu6502Test -quiet -cpu w65c02 programs\6502_functional_test.hex 
IF NOT ERRORLEVEL 0 GOTO endtests

..\bin\vrEmu6502Test -quiet -cpu r65c02 programs\6502_functional_test.hex 
IF NOT ERRORLEVEL 0 GOTO endtests

..\bin\vrEmu6502Test -filter 1000000 -cpu 6502 programs\6502_decimal_test.hex
IF NOT ERRORLEVEL 0 GOTO endtests

..\bin\vrEmu6502Test -filter 1000000 -cpu 65c02 programs\65c02_decimal_test.hex
IF NOT ERRORLEVEL 0 GOTO endtests

..\bin\vrEmu6502Test -filter 1000000 -cpu w65c02 programs\65c02_decimal_test.hex
IF NOT ERRORLEVEL 0 GOTO endtests

..\bin\vrEmu6502Test -filter 1000000 -cpu r65c02 programs\65c02_decimal_test.hex
IF NOT ERRORLEVEL 0 GOTO endtests

..\bin\vrEmu6502Test -quiet -cpu 65c02 programs\65C02_extended_opcodes_test.hex
IF NOT ERRORLEVEL 0 GOTO endtests

..\bin\vrEmu6502Test -quiet -cpu r65c02 programs\R65C02_extended_opcodes_test.hex
IF NOT ERRORLEVEL 0 GOTO endtests

..\bin\vrEmu6502Test -quiet -cpu w65c02 programs\W65C02_extended_opcodes_test.hex
IF NOT ERRORLEVEL 0 GOTO endtests

echo All vrEmu6502 tests:         PASSED

:endtests
popd

pause