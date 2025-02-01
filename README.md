# 🏗️ Projeto: Plataforma de Equilíbrio com Controle PID

Este projeto foi desenvolvido para a disciplina de **Introdução à Engenharia de Computação**, na **Universidade Federal de Goiás (UFG)**. O objetivo foi criar uma plataforma que pudesse se equilibrar automaticamente utilizando controle PID (Proporcional, Integral e Derivativo).

## 🎯 Objetivo
Implementar um sistema de controle PID para equilibrar uma bolinha sobre uma plataforma oscilante. O sistema ajusta o movimento do servomotor com base na posição da bolinha, buscando mantê-la no centro.

## 🛠️ Materiais Utilizados

- 1 x Microcontrolador Arduino
- 1 x Servomotor
- 1 x Sensor de posição (sensor ultrassônico)
- Platafora de papelão e cola quente.
- Componentes eletrônicos adicionais (fios, resistores, bateria)

## 📐 Esquema Eletrônico

Confira o projeto no Tinkercad clicando [aqui](https://www.tinkercad.com/things/3u3tynL4MZY-pid-ball?sharecode=vPQNCwW3k5mzyC0wkryjHGs4Zh8dpo4prxrz84mHo7Y).

## 🖼️ Fotos do Projeto

![Vista Frontal](assets/20241211_165748.jpg)
![Vista Lateral](assets/20241211_165926.jpg)
![Detalhes do Sistema](assets/20241211_165734.jpg)
![Servomotor](assets/20241211_165816.jpg)
![Sensor ultrassonico](assets/20241211_165812.jpg)

## 🎥 Vídeo Demonstrativo

[![Vídeo do Projeto](https://img.youtube.com/vi/-myj7t524_s/hqdefault.jpg)](https://youtube.com/shorts/-myj7t524_s?si=RU0jZc96KrgZ-HZB)

## 🧪 Resultados Observados

- O sistema foi capaz de manter a bolinha em posição de equilíbrio, ajustando automaticamente a inclinação da plataforma.
- A calibração dos parâmetros do controlador PID (“P”, “I” e “D”) foi essencial para obter uma resposta estável.
- Foram observados fenômenos de oscilação excessiva quando os valores do controlador estavam incorretos, demonstrando na prática a importância de ajuste fino.
- Ajustes nas dobradiças e no peso da plataforma para atender a força do servomotor.

## 📚 Aprendizado

- Compreensão prática dos conceitos de controle PID.
- Integração de sensores e atuadores em sistemas embarcados.
- Importância da calibração de parâmetros em sistemas de controle.

## 🔗 Inspiração e Créditos
Este projeto foi inspirado no projeto original [Self Balancing Ball and Beam using PID control](https://www.hackster.io/nihalsuri/self-balancing-ball-and-beam-using-pid-control-2bfb86), desenvolvido por **Nihal Suri**. O código foi modificado e adaptado conforme as necessidades deste projeto acadêmico.
