@echo off

pushd "%~dp0"

..\bin\vrEmu6502Test -q -f 1000000 -c 6502 programs\6502_functional_test.hex 
IF ERRORLEVEL 0 (echo 6502_functional_test PASSED) ELSE (echo 6502_functional_test FAILED)

echo:
..\bin\vrEmu6502Test -q -f 1000000 -c 6502 programs\6502_decimal_test.hex
IF ERRORLEVEL 0 (echo 6502_decimal_test PASSED) ELSE (echo 6502_decimal_test FAILED)

echo:
..\bin\vrEmu6502Test -q -f 1000000 -c 65c02 programs\65C02_extended_opcodes_test.hex
IF ERRORLEVEL 0 (echo 65C02_extended_opcodes_test PASSED) ELSE (echo 65C02_extended_opcodes_test FAILED)

popd

pause