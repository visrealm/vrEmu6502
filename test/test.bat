@echo off

pushd "%~dp0"

..\bin\vrEmu6502Test -q -f 1000000 -c programs\6502_functional_test.hex 
IF ERRORLEVEL 0 echo 6502 functional tests PASSED

echo:
..\bin\vrEmu6502Test -q -f 1000000 -c programs\6502_decimal_test.hex
IF ERRORLEVEL 0 echo 6502 decimal tests PASSED

echo:
..\bin\vrEmu6502Test -q -f 1000000 -c programs\65C02_extended_opcodes_test.hex
IF ERRORLEVEL 0 echo 6502 extended opcode tests PASSED

popd

pause