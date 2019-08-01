; Microcontrolador = ATMEGA328p
; Frecuencia de clock = 16MHz
; Placa del club de robotica FIUBA

.include "macros.inc"
.include "f_sampling_tables.inc"


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
				  ; Datos RAM
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
.dseg
.org SRAM_START
	
	; Variable auxiliar para flujo de programa
	PROGRAM_FLAGS: .byte 1 
	; Flags para PROGRAM_FLAGS
		.equ RX_STRING_READY = 1
		.equ ADC_DATA_READY = 2
		.equ TX_TRANSFER_READY = 3

	ADC_BUFFER: .byte 2
	CURRENT_TEMPERATURE: .byte 1
	TEMPERATURE_REF: .byte 1
	TEMPERATURE_ERROR: .byte 1

	; DIVISION_NUMERATOR_APPROX es el numerador usado para intentar 
	; aproximar una division que no es potencia de 2.
	; Por ejemplo N/11 se puede aproximar como N*186/2048 
	; donde DIVISION_NUMERATOR_APPROX = 186.
	DIVISION_NUMERATOR_APPROX: .byte 2
	ERROR_ACUM: .byte 3
	CONTROL_SIGNAL: .byte 2
	KP: .byte 2
	KI: .byte 1
	Frec_S: .byte 1

	.equ RX_BUFFER_SIZE = 10 +1 ; +1 por caracter ENDCHAR
	.equ STRING_ENDCHAR = 0xA ; es LF '/n'

	RX_BUFFER: .byte RX_BUFFER_SIZE
	RX_BUFFER_POINTER: .byte 2

	TX_BUFFER_ISR: .byte 4
	TX_ISR_TRANSMIT_BUFFERINDEX: .byte 1
	TX_ISR_TRANSMIT_COUNTER: .byte 1


	; TIMER_OC0A_COUNTER:
	; Contador de CTC del TIMER0 para frec. de muestreo del ADC
	;
	; Debido a que el TIMER1 esta siendo utilizado para la senal de control,
	; para poder muestrear a frecuencias del orden de 1 Hz se cuentan la cantidad
	; de interrupciones que el timer0 genera por CTC.
	;
	; Ejemplo: con un prescaler de 8, modo CTC, OCR0A = 250 se genera una interrupcion
	; a 125 us, por lo que con TIMER_OC0A_MAXVALCOUNTER = 8000 se podria actuar regularmente
	; a un periodo de 8000*125us = 1s.

	TIMER_OC0A_COUNTER: .byte 2
	TIMER_OC0A_MAXVALCOUNTER: .byte 2


	; TIMER0_COUNTER_TX
	; Contadores para frec. de transmision del TX
	; Es posible hacer una distincion entre la frecuencia de muestreo y la 
	; frecuencia a la que se envian datos por puerto serial. A medida que se genera
	; cada muestreo, se va llevando la cuenta para enviar algo.
	;
	; Ejemplo: para TIMER0_TX_COUNTER_MAX = 8, a una frecuencia de muestreo de 8 Hz
	; se estarian enviando datos a una frecuencia de 1 Hz.
	;
	; Por defecto vale 1, ya que podria traer confusion/inconvenientes si se
	; utiliza con un controlador externo (como podria ser Matlab/Simulink).

	TIMER0_COUNTER_TX: .byte 1
	TIMER0_TX_COUNTER_MAX: .byte 1


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
				  ; Datos EEPROM
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
.eseg
	; Valores default semi-arbitrarios. Se espera que sean cambiados en tiempo de ejecucion.
	.equ KP_init = 2940
	.equ KI_init = 60
	.equ TEMP_REF_init = 10
	.equ ADC_FREQ_HZ_init = 1
	
/*	; Definiciones incluidas en el archivo f_sampling_tables
	.equ TIMER0_PRESCALER = 8
	.equ F_CLOCK = 16000000
	.equ T0_CTC_FREQ_INTERRUPT = F_CLOCK / (TIMER0_PRESCALER*250)  ; OCR0A = 250, modo CTC*/
	; Ejemplo, T0_CTC_INTRPT_COUNTER_init = 4000 para 2 Hz si la interrupcion por CTC se da a 125us.*/
	.equ T0_CTC_INTRPT_COUNTER_init = T0_CTC_FREQ_INTERRUPT / ADC_FREQ_HZ_init 


	KP__EEPROM: .db LOW(KP_init), HIGH(KP_init)
	KI__EEPROM: .DB KI_init


	; TIMER_OC0A_MAXVALCOUNTER__EEPROM:
	; Valor de contador para frecuencia de muestreo inicial del ADC a 1 Hz (TIMER0 @ prescaler 8, CTC, OCR0A = 250)
	;
	; Valor inicial arbitrario a 1Hz (se cambia posteriormente en tiempo de ejecucion si fuera necesario).
	; Ver definicion de variable en .dseg para mas detalles. 

	TIMER_OC0A_MAXVALCOUNTER__EEPROM: .db LOW( T0_CTC_INTRPT_COUNTER_init ), HIGH( T0_CTC_INTRPT_COUNTER_init)

	; TIMER0_TX_COUNTER_MAX__EEPROM:
	; Permite mandar datos a frecuencia distinta a la de muestreo. 
	; Ej: Cada 8 muestreos del ADC enviar dato por TX necesitaria un valor de 8.
	;
	; Por defecto vale 1 para no tener inconvenientes cuando se utiliza un controlador
	; externo (por ejemplo Matlab) ya que se esperaria que los datos enviados tengan la misma
	; frecuencia que la muestreada. 
	TIMER0_TX_COUNTER_MAX__EEPROM: .DB 1

	TEMPERATURE_REF__EEPROM: .DB TEMP_REF_init

	DIVISION_NUMERATOR_APPROX__EEPROM: \
	.db LOW( DENOMINATOR_APPROX/(ADC_FREQ_HZ_init * KI_ADDITIONAL_DIVISION) ),\
	HIGH( DENOMINATOR_APPROX/(ADC_FREQ_HZ_init * KI_ADDITIONAL_DIVISION) )
									

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
			 ; Vectores de interrupcion
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

.cseg
.org 0x0000
	rjmp SETUP;

.org OC0Aaddr
	JMP ISR_TIMER_OC0A

.org ADCCaddr
	JMP ISR_ADC_CONV_COMPLETE

.org URXCaddr
	JMP ISR_RX_RECEIVE_COMPLETE

.org OVF1addr
	JMP ISR_TIMER1_OVF

.org OC1Aaddr
	JMP ISR_TIMER1_OCA

.org UDREaddr
	JMP ISR_TX_UDRE

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
					 ; Codigo
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

.org INT_VECTORS_SIZE
SETUP:

	LDI	R20,HIGH(RAMEND)
	OUT	SPH,R20
	LDI	R20,LOW(RAMEND)
	OUT	SPL,R20

	CALL BOARD_PINS_SETUP

	; Inicializar variables
	CLR R20
	STS PROGRAM_FLAGS, R20
	STS TIMER_OC0A_COUNTER, R20
	STS (TIMER_OC0A_COUNTER+1), R20
	STS CONTROL_SIGNAL, R20
	STS (CONTROL_SIGNAL+1), R20
	STS ERROR_ACUM, R20
	STS (ERROR_ACUM+1), R20
	STS (ERROR_ACUM+2), R20
	STS TX_BUFFER_ISR, R20

	LDI R20, LOW(RX_BUFFER)
	STS RX_BUFFER_POINTER, R20
	LDI R20, HIGH(RX_BUFFER)
	STS (RX_BUFFER_POINTER+1), R20

	
	READ_EEPROM_INTO_RAM  KP , KP__EEPROM
	READ_EEPROM_INTO_RAM  (KP+1) , (KP__EEPROM+1)
	READ_EEPROM_INTO_RAM  KI , KI__EEPROM
	READ_EEPROM_INTO_RAM  TIMER_OC0A_MAXVALCOUNTER, TIMER_OC0A_MAXVALCOUNTER__EEPROM
	READ_EEPROM_INTO_RAM  (TIMER_OC0A_MAXVALCOUNTER+1), TIMER_OC0A_MAXVALCOUNTER__EEPROM+1
	READ_EEPROM_INTO_RAM  TIMER0_TX_COUNTER_MAX, TIMER0_TX_COUNTER_MAX__EEPROM
	READ_EEPROM_INTO_RAM  TEMPERATURE_REF, TEMPERATURE_REF__EEPROM
	READ_EEPROM_INTO_RAM  DIVISION_NUMERATOR_APPROX, DIVISION_NUMERATOR_APPROX__EEPROM
	READ_EEPROM_INTO_RAM  DIVISION_NUMERATOR_APPROX+1, DIVISION_NUMERATOR_APPROX__EEPROM+1

	LDS R20, TIMER0_TX_COUNTER_MAX
	STS TIMER0_COUNTER_TX, R20

	CALL TIMERS_SETUP
	CALL ADC_SETUP
	CALL UART_SETUP

	SEI

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;; MAIN ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; Acciones del MAIN segun flags del programa
; - ADC_DATA_READY:
;	Se calcula la temperatura en base a la medicion del ADC y se 
;	busca la senal de control del PI. En caso de tener el 
;	interruptor UARTCONTROLLER activado no se calcula la senal de control
;	debido a que se obtiene mediante puerto serie.
;
; - RX_STRING_READY:
;	En caso de tener una cadena de caracteres lista en el
;	buffer RX_STRING, se procesa dicha cadena para cambiar 
;	parametros del controlador. Si el interruptor UARTCONTROLLER
;	se encuentra activado no se buscan parametros sino que se 
;	 obtiene la senal de control.
;
; - TX_STRING_READY
;	Cada 'TIMER0_TX_COUNTER_MAX' muestreos del ADC se 
;	levanta el flag de TX_TRANSFER_READY. En base al 
;	interruptor TX_CONTROL se manda la temperatura actual
;	o la senal de control. 
;	Formato por defecto: UINT16, little endian


MAIN:
		IS_PRG_FLAG_SET (ADC_DATA_READY)
		BREQ MAIN_KEEP1
			CALL CONVERT_ADC_2_TEMP

			SBIC PINB, PIN_UARTCONTROLLER_PORTB
				CALL PROCESS_TEMPERATURE_ERROR

			CLEAR_PRG_FLAG (ADC_DATA_READY)

	MAIN_KEEP1:
		IS_PRG_FLAG_SET (RX_STRING_READY)
		BREQ MAIN_KEEP2

			SBIS PINB, PIN_UARTCONTROLLER_PORTB
				RJMP LOAD_PI_UART

			SBI PORTC, PIN_REDLED_PORTC
			CALL PROCESS_RX_STRING
			RJMP RX_STRING_DONE

			LOAD_PI_UART:
			CBI PORTC, PIN_REDLED_PORTC
			CALL GET_CONTROL_SIGNAL_FROM_UART

			RX_STRING_DONE:
			; Reinicio puntero para asegurar escritura.
			LDI R20, LOW(RX_BUFFER)
			STS RX_BUFFER_POINTER, R20
			LDI R20, HIGH(RX_BUFFER)
			STS (RX_BUFFER_POINTER+1), R20

			; Reinicio RX
			LDS R20, UCSR0B
			ORI R20, ( (1<<RXEN0)|(1<<RXCIE0) )
			STS UCSR0B, R20

			CLEAR_PRG_FLAG (RX_STRING_READY)

	MAIN_KEEP2:
		IS_PRG_FLAG_SET (TX_TRANSFER_READY)
		BREQ MAIN_KEEP3

			LDS R20, UCSR0A
			ANDI R20, (1<<UDRE0)
			BREQ MAIN_KEEP3
			
			; Imprimo segun llave selectora
			SBIS PINB, PIN_TX_CONTROL_PORTB
				RJMP TX_TRANSMIT_OPT2

			TX_TRANSMIT_OPT1:
				LDS R11, CURRENT_TEMPERATURE

				; Guardo en el buffer el siguiente byte a enviar
				CLR R16
				STS TX_BUFFER_ISR, R16

				LDI R16, 1 ; Cantidad a enviar a traves del buffer = 1
				STS TX_ISR_TRANSMIT_COUNTER, R16
				RJMP SET_TX_TRANSMIT
			
			TX_TRANSMIT_OPT2:
				; La opcion 2 envia la temperatura y la senal de control juntas.
				; Cargo el segundo byte de CURRENT_TEMPERATURE (que esta vacio por ser un valor de 8 bits),
				; CONTROL_SIGNAL y CONTROL_SIGNAL+1 en el buffer.
				
				; El primer byte no se manda por interrupcion, sino al final de
				; esta seccion previo a habilitar la interrupcion de UDRE 
				; (registro de envio vacio)
				LDS R11, CURRENT_TEMPERATURE

				CLR R16
				STS TX_BUFFER_ISR, R16

				LDS R12, CONTROL_SIGNAL
				STS TX_BUFFER_ISR+1, R12

				LDS R12, CONTROL_SIGNAL+1
				STS TX_BUFFER_ISR+2, R12

				LDI R16, 3 ; Cantidad a enviar a traves del buffer = 3
				STS TX_ISR_TRANSMIT_COUNTER, R16

			SET_TX_TRANSMIT:
				CLR R20
				STS TX_ISR_TRANSMIT_BUFFERINDEX, R20
				; Mando primer dato (en R11) para arrancar el envio
				STS UDR0, R11
				LDS R20, UCSR0B
				ORI R20, (1<<UDRIE0)
				STS UCSR0B, R20
				
			CLEAR_PRG_FLAG (TX_TRANSFER_READY)


	MAIN_KEEP3:
		RJMP MAIN



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;  SUBRUTINAS  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


CONVERT_ADC_2_TEMP:
	; -Convierte el valor del ADC (en ADC_BUFFER) a valor de temperatura en grados.
	; -Guarda el valor en variable ACTUAL_TEMP en la RAM.
	; -Se usan numeros sin signo y devuelve temperatura en 8 bits.
	; - En caso de que el interruptor LPF_CONTROL este activo se usa
	;	un filtro de media exponencial como pasabajos.

	; -Como Vref = 1.235V y el LM35 devuelve 10mV/ºC
	; la expresión es 
	; Temp = ADC * (1.235V/1024) * 100 , lo cual equivale a
	; Temp = ADC * 247 / 2048

	LDS R22, ADC_BUFFER ; Low BYTE
	LDS R23, ADC_BUFFER + 1 ; High BYTE

	LDI R25, 247 ; Multiplicador
	LDI R21, log2(2048) ; Iteraciones para dividir por 2

	MUL R22, R25 ; ADCLow*247
	MOVW R2, R0
	CLR R4

	MUL R23, R25 ; ADCHigh*247
	ADD R3, R0 
	ADC R4, R1

	KEEP_DIV:
		LSR R4
		ROR R3
		ROR R2
		DEC R21
		BRNE KEEP_DIV
	
	; ADICIONAL: FILTRO PASABAJOS (Media exponencial)

	; Temp[k] = alpha * Temp[k] + beta * Temp[k-1]
	; tal que beta + alpha = 1

		SBIC PINB, PIN_LPF_CONTROL_PORTB
					RJMP EXIT_ADC2TEMP

			LDS R3, CURRENT_TEMPERATURE
			; R2 tiene el valor actual, R3 el anterior

			; Se pide (alpha' + beta')/div = 1
			.equ LPF_ALPHA = 1
			.equ LPF_BETA = 3
			.equ lpf_div = 4
			.equ LPF_DIV_LOG2 = log2(lpf_div)

			LDI R20, LPF_ALPHA
			LDI R21, LPF_BETA
	
			MUL R2, R20
			MOVW R8, R0 

			INC R8 ; 'compensacion' por truncamiento al dividir. 
			INC R8

			MUL R3, R21
			ADD R8, R0
			ADC R9, R1

			LDI R22, LPF_DIV_LOG2
		KEEP_DIV_LPF:
			LSR R9
			ROR R8
			DEC R22
			BRNE KEEP_DIV_LPF
			MOV R2, R8

EXIT_ADC2TEMP:
	STS CURRENT_TEMPERATURE, R2
RET

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

PROCESS_TEMPERATURE_ERROR:

	; - Maneja los casos segun valor de error de temperatura 
	;   (e.g. maneja el overflow del acumulador de error para el integrador)
	;
	; - En caso de tener el interruptor WINDUPCONTROL activo se vacia
	;	el error acumulado como metodo anti-windup debido a que no hay
	;	accion de control activa en caso de excederse de la referencia.
	;
	; - Invoca a GET_CONTROL_SIGNAL si los valores son adecuados.

			LDS R21, TEMPERATURE_REF
			LDS R20, CURRENT_TEMPERATURE
			SUB R21, R20
			BRSH TEMP_ERROR_POSITIVE


		;Caso de error de temperatura negativa:
		
			; Anti-Windup opcional mediante interruptor
			SBIC PINB, PIN_WINDUPCONTROL_PORTB
				RJMP NO_ANTIWINDUP

			CBI PORTD, PIN_YELLOWLED_PORTD 

			CLR R20
			STS TEMPERATURE_ERROR, R20
			STS ERROR_ACUM, R20
			STS (ERROR_ACUM+1), R20
			STS (ERROR_ACUM+2), R20
			CLI
			STS CONTROL_SIGNAL, R20
			STS (CONTROL_SIGNAL+1), R20
			SEI
			RJMP TEMP_ERROR_DONE


		NO_ANTIWINDUP:
			SBI PORTD, PIN_YELLOWLED_PORTD 
			CLR R20

			STS TEMPERATURE_ERROR, R20  ; trunco a cero para el error proporcional

			NEG R21 
			LDS R16, ERROR_ACUM
			LDS R17, (ERROR_ACUM+1)
			LDS R18, (ERROR_ACUM+2)
			SUB R16, R21
			SBCI R17, 0
			SBCI R18, 0
			; Si hay carry despues de SBCI es que hay borrow -> Se volveria negativo -> trunco a cero.
			BRCS TRUNCATE_LOW_ACUM	

			STS ERROR_ACUM, R16
			STS (ERROR_ACUM+1), R17
			STS (ERROR_ACUM+2), R18

			CLI
			STS CONTROL_SIGNAL, R20
			STS (CONTROL_SIGNAL+1), R20
			SEI
			RJMP TEMP_ERROR_DONE

		TRUNCATE_LOW_ACUM:
			STS ERROR_ACUM, R20
			STS (ERROR_ACUM+1), R20
			STS (ERROR_ACUM+2), R20
			CLI
			STS CONTROL_SIGNAL, R20
			STS (CONTROL_SIGNAL+1), R20
			SEI
			RJMP TEMP_ERROR_DONE

		; Caso de error de temperatura positivo
		TEMP_ERROR_POSITIVE:
			STS TEMPERATURE_ERROR, R21
			CLR R20
			LDS R16, ERROR_ACUM
			LDS R17, (ERROR_ACUM+1)
			LDS R18, (ERROR_ACUM+2)
			ADD R16, R21
			ADC R17, R20
			ADC R18, R20
			BRCC ACUM_NO_OVF

			; Saturar a 0xFFFFFF
			LDI R20, 0xFF 
			MOV R16,R20
			MOV R17,R20
			MOV R18, R20

		ACUM_NO_OVF:
			STS ERROR_ACUM, R16 
			STS (ERROR_ACUM+1), R17
			STS (ERROR_ACUM+2), R18

			; Si todo esta OK
			CALL GET_CONTROL_SIGNAL

TEMP_ERROR_DONE:
	RET

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

GET_CONTROL_SIGNAL:
		; Procesa CURRENT_TEMPERATURE y ERROR_ACUM para obtener los valores del PI.
		;
		; Como duty = (OCRAL1)/(0xFFFF)*100%, en caso de superar 0xFFFF se satura a este valor.
		;

		; Parte proporcional
		CLR R4 ; aux para sumar carry
		LDS R21, TEMPERATURE_ERROR
		LDS R22, KP
		LDS R23, (KP+1)

		MUL R22, R21 
		MOVW R2, R0
		MUL R23, R21 
		ADD R3, R0 
		ADC R1, R4 

		TST R1 ; Si hay tercer byte -> saturar
		BRNE SATURATE_ERROR


	
		; Obtencion de error integral
		CALL GET_INTEGRAL_CONTROL_SIGNAL

		
		; GET_INTEGRAL_CONTROL_SIGNAL devuelve
		; el resultado en R15:R14:R13:R12:R11:R10 pero el valor maximo admitido es
		; 0xFFFF por lo que se satura en caso de haber bytes adicionales.
		TST R12 
		BRNE SATURATE_ERROR
		TST R13
		BRNE SATURATE_ERROR
		TST R14
		BRNE SATURATE_ERROR
		TST R15
		BRNE SATURATE_ERROR

	SUM_ERRORS:
		ADD R10, R2
		ADC R11, R3
		BRCS SATURATE_ERROR

	SUM_ERROR_OK:
			CLI
			STS CONTROL_SIGNAL, R10
			STS (CONTROL_SIGNAL+1), R11
			SEI
		RET

		SATURATE_ERROR:
			; Saturar salida si me excedo de 16 bits.
			LDI R20, 0xFF
			CLI
			STS CONTROL_SIGNAL, R20
			STS (CONTROL_SIGNAL+1), R20
			SEI	
		RET


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

GET_INTEGRAL_CONTROL_SIGNAL:

; Obtiene el valor de la señal de control por el error integral.

; -> Recibe: el valor de la suma de error en R22:R21:R20
; -> Devuelve: en R15:R14:R13:R12:R11:R10 el valor de la señal de control por error integral
;	  aunque el valor util es cuando ocupa 2 bytes.
;
; Idea:
; -Normalizacion segun frecuencia de muestreo:
; Integral continua del error =~ sumatoria(error(m))*Ts = sumatoria(error(m))/Fs
;
; -Division como aproximacion de numerador/denominador:
; Numero/k  =~  Numero * (NUMERATOR_APPROX_TABLE(k) / DENOMINATOR_APPROX) 
; donde denominador es potencia de 2.
;
; Tambien dentro del numerador se tiene en cuenta una division extra por 
; una constante KI_ADDITIONAL_DIVISION para poder obtener un valor de KI 
; menor a 1. 
; Si KI_ADDITIONAL_DIVISION vale 10 el KI guardado en la RAM tendra el 
; efecto atenuado en factor de 10
;
; Datos:
; -sumatoria(error(m)) es el valor de ERROR_ACUM el cual ocupa 3 bytes
; -NUMERATOR_APPROX_TABLE es una tabla incluida en archivo aparte.
; -KI_ADDITIONAL_DIVISION, DENOMINATOR_APPROX son constantes de compilacion
; incluidas en dicho archivo. La tabla se genera en tiempo de
; compilacion en base a estas constantes.





LDS R16, KI 

LDS R20, ERROR_ACUM
LDS R21, (ERROR_ACUM+1)
LDS R22, (ERROR_ACUM+2)

LDS R18, DIVISION_NUMERATOR_APPROX
LDS R19, DIVISION_NUMERATOR_APPROX+1
LDS R16, KI

CLR R12
CLR R13
CLR R14
CLR R15
CLR R26
CLR R5

; R26:R25:R24 = numerador_aproximador_con_KI
; Se multiplica KI de la ram con el numerador aproximador de la tabla
; antes de hacer los corrimientos de bits para dividir. 
MUL R16, R18
MOVW R24,R0
MUL R16, R19
ADD R25,R0 
ADC R26,R1

CONTINUE_INTEGRAL_PRODUCT:
; Producto entre ERROR_ACUM (3 bytes) * numerador_aproximador_con_KI (3 bytes)
MUL R20, R24 
MOVW R10,R0

MUL R20, R25 
ADD R11,R0
ADC R12,R1

MUL R21, R24  
ADD R11, R0
ADC R12, R1
ADC R13, R5

MUL R21, R25 
ADD R12, R0
ADC R13, R1
ADC R14, R5
ADC R15, R5


MUL R22, R24 
ADD R12, R0
ADC R13, R1
ADC R14, R5
ADC R15, R5

MUL R22, R25 
ADD R13, R0
ADC R14, R1 
ADC R15, R5

; MODIFICACION 24-6-19 para multiplicar 3 bytes con 3 bytes
; Agrego tercer byte (en R26)
MUL R20, R26
ADD R12, R0
ADC R13, R1
ADC R14, R5
ADC R15, R5

MUL R21, R26
ADD R13, R0
ADC R14, R1
ADC R15, R5

MUL R22, R26
ADD R14, R0
ADC R15, R1



; Devuelve en R15:R14:R13:R12:R11:R12
; el valor de ERROR_ACUM * (Numerador_aprox_con_KI/divisor_aprox)
; el cual aproxima Ki * atenuadorKI * integral(error)
LDI R16, N_BITSHIFT_DIVISION
	KEEP_DIV_INTEGRALDIV:
	LSR R15
	ROR R14
	ROR R13
	ROR R12
	ROR R11
	ROR R10
	DEC R16
	BRNE KEEP_DIV_INTEGRALDIV

RET


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


END_RUT:
	RET


PROCESS_RX_STRING:

	LDI XL,LOW(RX_BUFFER) ;Carga del puntero 
	LDI XH,HIGH(RX_BUFFER)
	LD R20,X+ ;Cargo el primer caracter en R20 y post-incremento de puntero

	;Obtener K
	CPI R20,'K' ;Comparo si el primer caracter es K
	BRNE GET_R ;Si no es K, veo si es R

	LD R20,X+ ;Es K, guardo en R20 el sig caracter y post-incremento el puntero
	;Obtener (K)P
	CPI R20,'P' ;Comparo si es (K)P
	BRNE GET_I ; Si no es P, veo si es (K)I
	JMP GET_NUM_STRING_KP ;Es KP, guardo la cadena de num q sigue

GET_R:
	CPI R20,'R' ;Comparo si es R
	BRNE GET_F ;Si no es R, veo si es F
	JMP GET_NUM_STRING_R ;Es R, obtengo la cadena de num q sigue

GET_F:
	CPI R20,'F' ;Comparo si es F
	BRNE END_RUT ; Si no es F, vuelvo(no va a ser FS)
	LD R20,X+ ;Es F, avanzo y veo si es FS
	CPI R20,'S' ; Comparo si es (F)S
	BRNE END_RUT ;Si no es FS vuelvo
	JMP GET_NUM_STRING_FS ;Es FS, guardo la cadena de num que sigue

GET_I:
	;Obtener (K)I
	CPI R20,'I' ;Comparo si es (K)I
	BRNE END_RUT ;Si no es (K)I no fue K(P) ni R, asi que salgo
	JMP GET_NUM_STRING_KI ;Es (K)I, obtengo la cadena de num q sigue

GET_NUM_STRING_R:
	CLR R20	;Se devuelve por aca el num
	LDI R16,2 ;R es un num de dos digiteos
	CONVERT_ASCII_2_NUM R16, TEMPERATURE_REF
	
	WRITE_R20_INTO_EEPROM (TEMPERATURE_REF__EEPROM)

	JMP END_RUT
	
GET_NUM_STRING_KI:
	CLR R20	;Se devuelve por aca el num, KI menor 1byte
	LDI R16,3 ;KI es un num de 3 digitos
	CONVERT_ASCII_2_NUM R16,KI

	WRITE_R20_INTO_EEPROM (KI__EEPROM)
	
	JMP END_RUT

GET_NUM_STRING_KP:
	CLR R20	;Se devuelve por aca el num, KP de 2 bytes
	CLR R21
	LDI R16,5 ;KP puede ser un numero muy grande, pero menor a 2  bytes
	CONVERT_ASCII_2_NUM R16,KP

	WRITE_R20_INTO_EEPROM (KP__EEPROM)
	MOV R20, R21
	WRITE_R20_INTO_EEPROM ((KP__EEPROM+1))

	JMP END_RUT	 

GET_NUM_STRING_FS:
	CLR R20	;Se devuelve por aca el num, FS menor 1byte
	LDI R16,3 ;FS es un num de 3 digitos < 255
	CONVERT_ASCII_2_NUM R16,Frec_S

	MOV R16,R20 ;pongo la frecuencia en R16 para pasarlo por la rutina siguiente
	CALL SET_SAMPLING_FREQ
	
	JMP END_RUT

GET_5_DIGITS_2_NUM: ;Devuelvo por R20 el num (si es que necesito 2 bytes, sino solo R20)

	LD R23 ,X+ ;*10.000
	
	CALL GET_4_DIGITS_2_NUM

	SUBI R23,0x30 ;ASCII-NUM difieren en 30
	LDI R19,0x10 ;LOW(10.000)
	LDI R26,0x27 ;HIGH(10.000)
	MUL R23,R19
	ADD R20,R0 ;Suma de los low 
	ADC R21,R1 ;Suma con carry de los high 
	MUL R23,R26
	ADD R21,R0
	
	JMP END_RUT

GET_4_DIGITS_2_NUM: ;Devuelvo por R20:R21 el num (si es que necesito 2 bytes, sino solo R20)
	
	LD R22,X+ ;*1.000
	
	CALL GET_3_DIGITS_2_NUM
	
	SUBI R22,0x30 ;ASCII-NUM difieren en 30
	;Obtencion *1.000 no supera 1 byte
	LDI R19,0xE8 ;LOW(1.000)
	LDI R26,0x03 ;HIGH(1.000)
	MUL R22,R19 
	ADD R20,R0 ;Suma de los low 
	ADC R21,R1 ;Suma con carry de los high 
	MUL R22,R26
	ADD R21,R0 ;Suma de los low 
	
	JMP END_RUT

GET_3_DIGITS_2_NUM: ;Devuelvo por R20:R21 el num (si es que necesito 2 bytes, sino solo R20)
	
	LD R16,X+ ;Centena

	CALL GET_2_DIGITS_2_NUM
	
	SUBI R16,0x30 ;ASCII-NUM difieren en 30
	LDI R19,100 ;Obtencion de la centena
	MUL R16,R19 
	ADD R20,R0 ;Suma de los low 
	ADC R21,R1 ;Suma con carry de los high 
	
	JMP END_RUT

GET_2_DIGITS_2_NUM: ;Devuelvo por R20 el num
	
	LD R17,X+ ;Decena
	LD R18,X ;Unidad
	SUBI R17,0x30 ;ASCII-NUM difieren en 30
	SUBI R18,0x30
	LDI R19,10 ;Obtencion de la decena
	MUL R17,R19
	ADD R20,R0 ;Suma de los low de la decena y la unidad
	ADC R21,R1 ;Suma con carry de los high de la decena y la unidad
	CLR R30 ;Para el carry posible, lo cargo con 0 ahora para no tener flags que no corresponden luego
	ADD R20,R18 ;Suma del low con la unidad
	ADC R21,R30 ;Suma del carry posible,,,El numero convertido esta en R20:R21  
	
	JMP END_RUT
	
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

SET_SAMPLING_FREQ:
	; Recibe en R16 la frecuencia de muestreo, maneja los
	; registros necesarios y guarda en EEPROM el valor nuevo.

	; Carga de las tablas en el archivo "f_sampling_tables.inc"
	; los valores necesarios de:
	; - Contador de interrupciones para la frecuencia de muestreo
	;   (ej: para una interrupcion a 125us, contando 4000 se puede
	;	 actuar a 2 Hz)
	; - Numerador para aproximar la division del acumulador
	;   de error cuando se normaliza segun frecuencia de muestreo.
	;	(ej: para dividir por 11 se aproxima como el producto con 186/2048)

	; Detener TIMER1 y reiniciar el valor del contador
	IN R24, TCCR0B
	MOV R23, R24
	ANDI R23, ~( (1<<CS00)|(1<<CS01)|(1<<CS02))
	OUT TCCR0B, R23
	CLR R23
	STS TIMER_OC0A_COUNTER, R23
	STS (TIMER_OC0A_COUNTER+1), R23
	OUT TCNT0, R23

	; Verificar rango entre (1;MAX_SAMPLE_FREQ)
	TST R16
	BREQ EXIT_SET_SAMPL_FREQ
	CPI R16, MAX_SAMPLE_FREQ+1
	BRSH EXIT_SET_SAMPL_FREQ

	; Cargar datos en R19:R18
	LOAD_INDEX_FROM_TABLE (TABLA_CONTADOR_CTC_FREC_MUESTREO), R16
	STS TIMER_OC0A_MAXVALCOUNTER, R18
	STS TIMER_OC0A_MAXVALCOUNTER+1, R19
	MOV R20, R18
	WRITE_R20_INTO_EEPROM (TIMER_OC0A_MAXVALCOUNTER__EEPROM)
	MOV R20, R19
	WRITE_R20_INTO_EEPROM (TIMER_OC0A_MAXVALCOUNTER__EEPROM+1)

	; Cargo valor del numerador para aproximar
	LOAD_INDEX_FROM_TABLE (NUMERATOR_APPROX_TABLE), R16
	STS DIVISION_NUMERATOR_APPROX, R18
	STS DIVISION_NUMERATOR_APPROX+1, R19
	MOV R20, R18
	WRITE_R20_INTO_EEPROM (DIVISION_NUMERATOR_APPROX__EEPROM)
	MOV R20, R19
	WRITE_R20_INTO_EEPROM (DIVISION_NUMERATOR_APPROX__EEPROM+1)



EXIT_SET_SAMPL_FREQ:
	OUT TCCR0B, R24 
	RET

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

 GET_CONTROL_SIGNAL_FROM_UART:

	; Funcion para usar controlador externo por puerto serie.
	; - La entrada debe estar truncada previamente entre 0 y 65535
	; - El tipo de dato recibido es uint16	 (no ASCII)

	LDI XL, LOW(RX_BUFFER)
	LDI XH, HIGH(RX_BUFFER)

	LD R20, X+
	LD R21, X+

	LD R22, X
	CPI R22, STRING_ENDCHAR
	BRNE EXIT_RXSTRING_CONTROL_SIGNAL

	CLI
	STS CONTROL_SIGNAL, R20
	STS (CONTROL_SIGNAL+1), R21
	SEI

	EXIT_RXSTRING_CONTROL_SIGNAL:
	RET


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
_EEEPROM_READ:
	; Carga en R22 lo que haya en la eeprom en
	; la direccion dada por R20 (EEARL) y R21 (EEARH)

	SBIC EECR,EEPE      
	RJMP _EEEPROM_READ       

	OUT EEARL, R20 
	OUT EEARH, R21    

	SBI EECR,EERE      
	IN R22, EEDR           

 RET


 _EEPROM_WRITE:
	; Carga lo que hay en R20 en 
	; la direccion R30 (EEARL) y R31 (EEARH)
	CLI

	SBIC EECR,EEPE
	RJMP _EEPROM_WRITE

	OUT EEARH, R31
	OUT EEARL, R30

	OUT EEDR,R20
	SBI EECR,EEMPE
	SBI EECR,EEPE
	SEI

	RET
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; Inicializacion ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
						; TIMERS SETUP
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	
TIMERS_SETUP:

	; TIMER0: Uso para muestreo del ADC y transmision TX.

	; Configuracion de los registros de los timers
	; Prescaler 8 @16Mhz -> Ttclk = 500ns 
	; Modo CTC con OCRA = 250
	; ---> Periodo de interrupcion: 125us

	; Registros:
	; TCCR0A = 0000--10 CTC, (I/O mem)
	; TCCR0B = 00--0010 CTC, Prescaler 8 (I/O mem)
	; OCR0A = 250 dec (I/O mem)
	; TIMSKS0 = 0x02, OCIE0A (NO I/O mem)

	LDI R20, 250 
	OUT OCR0A, R20

	LDI R20, 0x02
	STS TIMSK0, R20

	LDI R20, 0x02  
	OUT TCCR0A, R20

	LDI R20, 0x02
	OUT TCCR0B, R20

	; TIMER1: Uso como senal de control.
	; -En la interrupcion de overflow se busca el valor de la senal de
	;	control CONTROL_SIGNAL, se la introduce en el registro OCR1B y se 
	;	enciende el actuador.
	; -En la interrupcion por comparacion con el OCR1B se apaga el actuador.
	
	; Configuracion:
	; TCCR1A = 0x00 ; Modo normal, sin compare Output-Mode
	; TCCR1B = (1<<CS12) ; prescaler 256 ( Overflow approx 1 seg )
	; TIMSK1 = (1<<TOIE1)|(1<<OCIE1A)
	; OCRAL1 es tal que Duty = (OCRAL1)/(0xFFFF)*100%
	

	LDI R20, 0x00
	STS TCCR1A, R20

	LDI R20, (1<<TOIE1)|(1<<OCIE1A)
	STS TIMSK1, R20

	CLR R20
	STS OCR1AH, R20
	STS OCR1AL, R20
	
	LDI R20, (1<<CS12)
	STS TCCR1B, R20

	; TIMER2
	;
	; El TIMER2 genera una onda cuadrada de aprox 1 kHz y se utiliza 
	; para poder asegurar el disparo ('trigger') del optoacoplador.
	; 
	; Pin OC2B: set on OCR2B y off en BOTTOM. 
	; Modo: FAST PWM
	; OCR2B = 128  tal de tener senal cuadrada con duty 50%
	
	; Registros
	; TCCR2A = (1<<COM2A1)| (1<<WGM21) | (1<<WGM20)   ; STS 
	; TCCR2B = Prescaler  ; STS 
	; OCR2A = 128  ; STS

	;LDI R20,  (1<<COM2A1)|(1<<WGM21)|(1<<WGM20)
	LDI R20, (1<<COM2B1)|(1<<WGM21)|(1<<WGM20)
	STS TCCR2A, R20

	LDI R20, (1<<CS22)
	STS TCCR2B, R20

	LDI R20, 128
	STS OCR2B, R20



RET

	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
						; ADC SETUP
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

ADC_SETUP:
	; Prescaler 128:  0000 0111  
	.equ ADC_PRESCALER_MASK = 0x07

	LDI R20, (1<<ADEN)|(1<<ADIE)|ADC_PRESCALER_MASK
	STS ADCSRA, R20

	; Ref externa 1.235V usando LM385 , MUX=ADC0
	LDI R20, 0 ; (1<<REFS0) ; usar REFS0 para ref interna
	STS ADMUX, R20

RET

	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
						; UART SETUP
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

UART_SETUP:
	; Tabla de UBRR
	; @ 16 Mhz
	;	Baud	UBRR	% of error
	;	300		3332	0.0
	;	600		1666	0.0
	;	1200	832		0.0
	;	2400	416		0.1
	;	4800	207		0.2
	;	9600	103		0.2
	;	14400	68		0.6
	;	19200	51		0.2
	;	28800	34		0.8
	;	38400	25		0.2
	;	57600	16		2.1
	;	76800	12		0.2
	;	115200	8		3.7

	; UART a 38400 baud
	.equ UART_UBRR = 25

	LDI r16, LOW(UART_UBRR) 
	STS UBRR0L, r16;  
	LDI R16, HIGH(UART_UBRR) 
	STS UBRR0H, r16;

	; Config de UCSRB: TX RX on, 8 bits
	LDI R16, (1<<RXEN0)|(1<<RXCIE0)|(1<<TXEN0)
	STS UCSR0B, R16

	; Config de UCSRC: asincronico, sin paridad, 1 stop bit, 8 bits
	LDI R16, (1<<UCSZ01)|(1<<UCSZ00)
	STS UCSR0C, R16
RET

	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
						; Pins INIT
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

BOARD_PINS_SETUP:

	; Definiciones para la placa del club de robotica
	.equ PIN_GREENLED_PORTD = 4
	.equ PIN_YELLOWLED_PORTD = 7
	.equ PIN_REDLED_PORTC = 2
	.equ PIN_ORANGELED_PORTC = 3

	; Salida hacia optoacoplador
	.equ PIN_OC2B_PORTD = 3

	; Interruptores en puerto B.
	.equ PIN_TX_CONTROL_PORTB = 3
	.equ PIN_LPF_CONTROL_PORTB = 1
	.equ PIN_WINDUPCONTROL_PORTB = 2
	.equ PIN_UARTCONTROLLER_PORTB = 4
	.equ SWITCHES_MASK_PORTB = (1<<PIN_TX_CONTROL_PORTB)|(1<<PIN_LPF_CONTROL_PORTB)|(1<<PIN_WINDUPCONTROL_PORTB)|(1<<PIN_UARTCONTROLLER_PORTB)

	; Buffer de salida deshabilitados
	IN R16, DDRB
	ANDI R16, ~SWITCHES_MASK_PORTB
	OUT DDRB, R16

	; Pullups
	IN R16, PINB
	ORI R16, SWITCHES_MASK_PORTB
	OUT PORTB, R16

	; LEDS
	LDI R16, (1<<PIN_GREENLED_PORTD)|(1<<PIN_YELLOWLED_PORTD)
	OUT DDRD, R16
	OUT PORTD, R16
	LDI R16, (1<<PIN_REDLED_PORTC)|(1<<PIN_ORANGELED_PORTC) 
	OUT DDRC, R16
	OUT PORTC, R16
	RET


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;  ISR   ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

ISR_TIMER_OC0A:
		; Timer para muestreo de ADC y transmision TX
		; Configuracion de TIMER0: (Prescaler 8, modo CTC, OCR0A = 250)
		;
		; Por un lado TIMER_OC0A_COUNTER cuenta la cantidad de veces que se da
		; esta interrupcion para poder realizar otra tarea a una frecuencia
		; relativamente baja.
		;
		; Para la configuracion actual, la interrupcion se da cada
		; 125us por lo que, a modo de ejemplo, si TIMER_OC0A_COUNTER = 2000:
		; 2000 * 125us = 0.25s -> 4 Hz
		; Es decir, se muestrea la temperatura a 4 Hz. 
		; 
		; Adicionalmente, se cuentan la cantidad de muestreos del ADC tal de 
		; poder tener frecuencia de transmision serial distinta:
		; -Cada 'TIMER0_TX_COUNTER_MAX' muestreos de ADC se setea flag de transmision TX para el main.
		; (e.g. @8hz de muestreo de ADC, si TIMER0_TX_COUNTER_MAX = 8 -> 1Hz de transmision TX)

		PUSH R24
		PUSH R25
		PUSH R22
		IN R22, SREG 
		PUSH R22

		LDS R24, TIMER_OC0A_COUNTER
		LDS R25, (TIMER_OC0A_COUNTER+1)
		ADIW R24, 1
		STS TIMER_OC0A_COUNTER, R24
		STS (TIMER_OC0A_COUNTER+1), R25

		LDS R22, TIMER_OC0A_MAXVALCOUNTER
		CP R24, R22
		BRNE EXIT_TIMER_OC0A_ISR

		LDS R22, (TIMER_OC0A_MAXVALCOUNTER+1)
		CP R25, R22
		BRNE EXIT_TIMER_OC0A_ISR


		CLR R22
		STS TIMER_OC0A_COUNTER, R22
		STS (TIMER_OC0A_COUNTER+1), R22


		; Contador extra para imprimir por puerto serie
			LDS R22, TIMER0_COUNTER_TX
			DEC R22
			BRNE SKIP_TX_FLAG_SET
				SET_PRG_FLAG (TX_TRANSFER_READY)
				LDS R22, TIMER0_TX_COUNTER_MAX
			SKIP_TX_FLAG_SET:
				STS TIMER0_COUNTER_TX, R22

		LDS R22, ADCSRA
		ORI R22, (1<<ADSC) ; Empezar conversion de ADC
		STS ADCSRA, R22


	EXIT_TIMER_OC0A_ISR:
		POP R22
		OUT SREG, R22
		POP R22
		POP R25
		POP R24
		RETI

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

ISR_ADC_CONV_COMPLETE:
	; -Guarda en ADC_BUFFER lo que se lee del ADC.
	; -Led verde para monitorear @ f_muestreo/2 
	PUSH R22
	PUSH R23
	IN R23, SREG 

	LDS R22, ADCL
	STS ADC_BUFFER, R22
	LDS R22, ADCH
	STS ADC_BUFFER + 1 , R22

	SBI PIND, PIN_GREENLED_PORTD 

	SET_PRG_FLAG (ADC_DATA_READY)

	OUT SREG, R23
	POP R23
	POP R22
	RETI

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

ISR_RX_RECEIVE_COMPLETE:

; Rutina de interrupcion para guardar una cadena de caracteres
; en RAM. 
;
; -Verifica fin de cadena mediante caracter STRING_ENDCHAR o trunca con 
; dicho caracter si se llega al limite del buffer.
;
; -Cuando se tiene una cadena lista levanta el flag RX_STRING_READY
; en PROGRAM_FLAGS

	PUSH R20
	PUSH R21
	IN R21, SREG 

		LDS R20, UDR0

		LDS XH, (RX_BUFFER_POINTER+1)
		LDS XL, RX_BUFFER_POINTER
		ST X+, R20
		STS (RX_BUFFER_POINTER+1), XH
		STS RX_BUFFER_POINTER, XL

		; Busco caracter delimitador para fin de cadena
		CPI R20, STRING_ENDCHAR
		BREQ RX_READY

		; Mientras no encuentre caracter '/0' verifico que no llegue al limite
		CPI XL, LOW(RX_BUFFER + RX_BUFFER_SIZE-1)
		BRNE EXIT_RX_ISR_RECEIVE
		CPI XH, HIGH(RX_BUFFER + RX_BUFFER_SIZE-1)
		BRNE EXIT_RX_ISR_RECEIVE

		; Si escribio en (RX_BUFFER + X_BUFFER_SIZE - 1) -> trunco y termino. 
		; (X apunta a (RX_BUFFER + X_BUFFER_SIZE) por el ST X+)
		LDI R20,  STRING_ENDCHAR
		ST X, R20 



	RX_READY:
		SET_PRG_FLAG (RX_STRING_READY)

		; Apago RX hasta procesar cadena. 
		LDS R20, UCSR0B
		ANDI R20, ~( (1<<RXEN0)|(1<<RXCIE0) )
		STS UCSR0B, R20

	EXIT_RX_ISR_RECEIVE:
	OUT SREG, R21
	POP R21
	POP R20
	RETI

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

ISR_TIMER1_OVF:
	; -Busca el valor de CONTROL_SIGNAL y activa el buffer de
	; salida de OC2A tal de poder activar el optoacoplador.
	; -Led naranja para monitoreo
	PUSH R20
	PUSH R21
	IN R21, SREG
	PUSH R21
	
	LDS R20, CONTROL_SIGNAL
	LDS R21, (CONTROL_SIGNAL+1)

	; PRUEBA PARA MODELAR LA PLANTA
	; DUTY 5%
	; 65536*0.05 =~ 3277
	;LDI R20, LOW(3277)
	;LDI R21, HIGH(3277)

	TST R20
	BRNE SET_CONTROL_ACTION
	TST R21 
	BREQ EXIT_T1_OVF_ISR

SET_CONTROL_ACTION:
	STS OCR1AH, R21
	STS OCR1AL, R20

	CBI PORTC, PIN_ORANGELED_PORTC 
	SBI DDRD, PIN_OC2B_PORTD 


EXIT_T1_OVF_ISR:
	POP R21
	OUT SREG,R21
	POP R21
	POP R20

	RETI

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

ISR_TIMER1_OCA:

	; Apaga el buffer de salida de OC2B y el led de monitoreo.
	SBI PORTC, PIN_ORANGELED_PORTC 
	CBI DDRD, PIN_OC2B_PORTD 

	RETI


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

ISR_TX_UDRE:
	; Interrupcion utilizada para enviar los datos en el buffer
	; TX_BUFFER_ISR a traves del puerto serie.
	;
	; Por cada interrupcion envia lo que hay en 
	;	TX_BUFFER_ISR + TX_ISR_TRANSMIT_BUFFERINDEX
	;
	; Notar que por el indexado empezando en cero se debe parar
	; antes de TX_ISR_TRANSMIT_COUNTER - 1

	PUSH R16
	PUSH R17
	PUSH XL
	PUSH XH
	IN R16, SREG
	PUSH R16
	
	LDS R17, TX_ISR_TRANSMIT_BUFFERINDEX
	
	; Por indexado en 0 para N = 3 transferencias paro antes de mandar el indice 3.
	LDS R16, TX_ISR_TRANSMIT_COUNTER
	CP R16, R17
	BREQ DISABLE_TX_UDRIE
	
	LDI XH, HIGH(TX_BUFFER_ISR)
	LDI XL, LOW(TX_BUFFER_ISR)
	
	CLR R16
	ADD XL, R17
	ADC XH, R16
	
	LD R16, X
	STS UDR0, R16

	INC R17
	STS TX_ISR_TRANSMIT_BUFFERINDEX, R17
	RJMP TX_ISR_UDRE_EXIT


	DISABLE_TX_UDRIE:
		LDS R16, UCSR0B
		ANDI R16, ~(1<<UDRIE0)
		STS UCSR0B, R16

	TX_ISR_UDRE_EXIT:
		POP R16
		OUT SREG, R16
		POP XH
		POP XL
		POP R17
		POP R16
RETI


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
