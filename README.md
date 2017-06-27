# Medidor-de-Temperatura
-- PROJETO PARA A DISCIPLINA DE SISTEMAS EMBARCADOS --
MATERIAL UTILIZADO:
- SAM R21;
- SENSOR LM35;
- DISPLAY OLED;

PROJETO:
Usar o sensor de temperatura para receber a temperatura atual medida do ambiente e calcular a temperatura média, 
temperatura maxima, temperatura mínima e mostrar esses valores no display OLED. 
Ainda, será feito um uso da memória EEPROM para guardar os valores calculados e prevenir possíveis resets da aplicação.
A média dos valores será feita com base em uma constante, que simboliza a quantidade de valores que serão considerados
nesse calculo.

IMPLEMENTADO:
- Uso do display;
- Uso da memória EEPROM;
- Cálculos básicos;
- Tabela de estados;
- Sensor integrado na placa;
- Conversão dos valores lidos pelo sensor de bits para graus Célcius;
- Inicio da documentação doxygen.

DATA DE ATUALIZAÇÃO: 27/06/2017
