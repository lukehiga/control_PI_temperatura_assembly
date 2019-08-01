; Macros para el controlador PI

; Las macros de PRG_FLAG se usan para hacer el codigo mas legible.
; Notar que por las operaciones que hacen, pueden alterar el 
; registro SREG.


.macro IS_PRG_FLAG_SET
LDS R25, PROGRAM_FLAGS
ANDI R25, (1<<@0)
.endmacro

.macro CLEAR_PRG_FLAG
LDS R25, PROGRAM_FLAGS
ANDI R25, ~(1<<@0)
STS PROGRAM_FLAGS, R25
.endmacro

.macro SET_PRG_FLAG
LDS R25, PROGRAM_FLAGS
ORI R25, (1<<@0)
STS PROGRAM_FLAGS, R25
.endmacro


.macro READ_EEPROM_INTO_RAM
; Cuidado con los registros R20 y R21.
LDI R20, LOW(@1)
LDI R21, HIGH(@1)
CALL _EEEPROM_READ ; Devuelve por R22
STS @0, R22
.endmacro

.macro WRITE_R20_INTO_EEPROM
; Cuidado si se esta utilizando el puntero Z
LDI R30, LOW(@0)
LDI R31, HIGH(@0)
CALL _EEPROM_WRITE
.endmacro


.macro LOAD_INDEX_FROM_TABLE
; @0 es el nombre de la tabla.
; @1 es el registro que contiene el indice del elemento (indexado arranca desde 0)

; Devuelve en R19:R18 el valor obtenido.

LDI ZL, LOW(@0)
LDI ZH, HIGH(@0)
CLR R21

ADD ZL, @1
ADC ZH, R21

LSL ZL
ROL ZH

; Primero carga el LOW byte y despues el HIGH byte por ser little endian
LPM R18, Z+
LPM R19, Z
.endmacro


.MACRO CONVERT_ASCII_2_NUM
	;@0 es la cant de digitos
	;@1 es el puntero al primer ASCII num
	
	CPI @0, 5
	BRNE CASE_3_DIGITS
	CALL GET_5_DIGITS_2_NUM 
	STS @1,R20 ;Guardo el low 
	STS (@1+1),R21 ;Guardo el high 
	JMP EXIT_MACRO

CASE_3_DIGITS: ;y se que no superan 255
	CPI @0, 3
	BRNE CASE_2_DIGITS
	CALL GET_3_DIGITS_2_NUM 
	STS @1,R20 
	JMP EXIT_MACRO
	
CASE_2_DIGITS:
	CPI @0, 2
	BRNE EXIT_MACRO
	CALL GET_2_DIGITS_2_NUM 
	STS @1,R20 

EXIT_MACRO:
.ENDMACRO
