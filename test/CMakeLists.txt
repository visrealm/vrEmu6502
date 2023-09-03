enable_testing()

add_executable (vrEmu6502Test vrEmu6502Test.c)

target_link_libraries(vrEmu6502Test vrEmu6502)

set(TESTS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/programs/)

add_test(6502_functional_test vrEmu6502Test -quiet -cpu 6502 ${TESTS_DIR}6502_functional_test.hex)
add_test(65c02_functional_test vrEmu6502Test -quiet -cpu 65c02 ${TESTS_DIR}6502_functional_test.hex)
add_test(w65c02_functional_test vrEmu6502Test -quiet -cpu w65c02 ${TESTS_DIR}6502_functional_test.hex)
add_test(r65c02_functional_test vrEmu6502Test -quiet -cpu r65c02 ${TESTS_DIR}6502_functional_test.hex)

add_test(6502_decimal_test vrEmu6502Test -quiet -cpu 6502 ${TESTS_DIR}6502_decimal_test.hex)
add_test(65c02_decimal_test vrEmu6502Test -quiet -cpu 65c02 ${TESTS_DIR}6502_decimal_test.hex)
add_test(w65c02_decimal_test vrEmu6502Test -quiet -cpu w65c02 ${TESTS_DIR}6502_decimal_test.hex)
add_test(r65c02_decimal_test vrEmu6502Test -quiet -cpu r65c02 ${TESTS_DIR}6502_decimal_test.hex)

add_test(65c02_extended_test vrEmu6502Test -quiet -cpu 65c02 ${TESTS_DIR}65C02_extended_opcodes_test.hex)
add_test(w65c02_extended_test vrEmu6502Test -quiet -cpu w65c02 ${TESTS_DIR}W65C02_extended_opcodes_test.hex)
add_test(r65c02_extended_test vrEmu6502Test -quiet -cpu r65c02 ${TESTS_DIR}R65C02_extended_opcodes_test.hex)
