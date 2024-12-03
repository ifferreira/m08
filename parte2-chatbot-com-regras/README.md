# Chatbot de comando 

Este projeto implementa um chatbot interativo que pode receber comandos de texto do usuário e direcionar um robô para locais específicos (secretaria, laboratório, biblioteca) usando ROS 2 (Robot Operating System). O chatbot é alimentado por uma interface gráfica feita com Tkinter e utiliza um sistema de reconhecimento de intenção simples para entender os comandos dados pelo usuário. Ele também sugere comandos próximos se um comando não for reconhecido diretamente. 

A integração com o ROS 2 permite que o chatbot publique comandos e respostas via tópicos, e receba feedback do robô sobre a ação executada.

