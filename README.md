# Proyecto: Controlador de Temperatura

Trabajo práctico para la materia Laboratorio de Microcontroladores.

Integrantes: Higa, Lucas --- Hsieh, Pablo 

El proyecto a realizar es un controlador PI de temperatura utilizando el microcontrolador ATMEGA328p. La motivación es tal de poder aplicar conceptos de la materia Control Automático I, la cuál está siendo cursada en paralelo a Laboratorio de microprocesadores.

Se calentará agua y se controlará que el fluído se mantenga en una temperatura fija. Se realizará de esta forma ya que se encontró una pava eléctrica que tiene el controlador sin funcionar, por lo que la aplicación de este proyecto podrá arreglar el producto y hasta mejorar su utilidad.

La temperatura se medirá con un LM35(con un sistema sumergible en líquido) y se procesará la información a través del ADC del microcontrolador.

Se utilizará un circuito optoacoplador junto a otro tiristor para activar y desactivar el circuito de la pava eléctrica. La utilización de dos tiristores es para aislar el circuito de potencia con el circuito del microcontrolador.

La forma de control es con un PI, fijando una temperatua de referencia, una vez que se alcance, se dejará de excitar el tiristor por lo que el circuito de la pava se apagará. Esto generará que la temperatura vuelva a disminuir, por lo que luego de alcanzar otro umbral, se encenderá nuevamente el actuador y asi sucesivamente para mantener el fluído a la temperatura de referencia.

