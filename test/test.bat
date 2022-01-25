@echo off

pushd "%~dp0"

echo 6502 functional tests:
..\bin\vrEmu6502Test programs\6502_functional_test.hex
IF ERRORLEVEL 0 echo 6502 functional tests PASSED

echo:
echo 6502 decimal tests:
..\bin\vrEmu6502Test programs\6502_decimal_test.hex
IF ERRORLEVEL 0 echo 6502 decimal tests PASSED

echo:
echo 6502 extended opcode tests:
..\bin\vrEmu6502Test programs\65C02_extended_opcodes_test.hex
IF ERRORLEVEL 0 echo 6502 extended opcode tests PASSED

popd

pause