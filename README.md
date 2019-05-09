# Smart-Office-Illumination-System
Base em Arduino

As pastas total_4 e total_5 contém o código para um arduino. Este código recebe valores do ldr e consoante esse valor baixa ou aumenta a intensidade do seu led. 

Contém o algoritmo consesus que é usado para que todos os arduinos que estejam na mesma rede de comunicação cheguem a um conseso no valor a colocar nos seus respectivos leds.

O server é corrido do raspberry. Este vai estar à escuta na rede de comunicação dos arduinos e vai guardando valores. Quando inquirido o servidor irá responder com dados mais atuais.

Este projeto serve para simular a instalação de uma rede de iluminação com controlo à distância.
