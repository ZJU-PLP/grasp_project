Repositório dedicado a pesquisa sobre "robotic grasping" de Caio Viturino.
------------
### Usage

First run kinect driver
`rosrun kinect2_bridge kinect2_bridge depth_method:=opengl reg_method:=cpu`

------------
### Reunião - 25/11/2019
Assuntos abordados:
1. Utilizar, preferencialmente, os dispositivos já presentes no laboratório, como o UR5, Intel Realsense e o Gripper 2-Fingers Robotiq
2. Verificar como utilizar redes neurais para prever a posição de objetos. Dessa forma, o método proposto seria robusto contra limitações da câmera em relação a proximidade do objeto, ou seja, mesmo que não haja informações de profundidade, a rede neural utilizaria dados passados para prever onde o objeto está no dado momento.
3. Fazer uma pesquisa bibliográfica em relação à aplicações de grasping.
4. Traduzir a tese para a língua inglesa
