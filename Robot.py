from hub import motion_sensor, port, button
import motor
import runloop, motor_pair, time, color_sensor, color
from app import linegraph
from motor import *
import math
from math import atan2, degrees, sqrt

# Definição de constantes que representam comandos de movimento
subir = 'subir'
descer = 'descer'
parFrente = motor_pair.PAIR_1 # Par de motores frontais

class Controle_PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp # Proporcional
        self.ki = ki # Integral
        self.kd = kd # Derivativo
        self.integral_sum = 0 # Soma do erro integral
        self.last_error = 0 # Erro anterior usado no cálculo do derivativo

    # Obtém o ângulo atual da guinada(yaw) do robô
    async def _get_current_yaw(self):
        return motion_sensor.tilt_angles()[0] * -0.1

    # Calcula o erro atual com base no ângulo alvo
    async def _compute_error(self, target):
        current_angle = await self._get_current_yaw()
        return target - current_angle

    # Calcula o termo derivativo (taxa de mudança do erro)
    async def _compute_integral(self, error):
        self.integral_sum += error
        return self.integral_sum

    # Calcula o termo derivativo (taxa de mudança do erro)
    async def _compute_derivative(self, error):
        derivative = error - self.last_error
        self.last_error = error
        return derivative

    # Função principal que calcula a correção PID para os motores
    async def calculate(self, target, power):
        error = await self._compute_error(target)
        integral = await self._compute_integral(error)
        derivative = await self._compute_derivative(error)

        # Aplica a fórmula PID: (kp * erro) + (ki * integral) + (kd * derivativo)
        correction = (self.kp * error) + (self.ki * integral) + (self.kd * derivative)

        # Ajusta as velocidades dos motores com base na correção
        left_speed = power - correction
        right_speed = power + correction

        return int(right_speed), int(left_speed) # Retorna o ângulo atual

class Missao:
    def __init__(self, coordenadas, angulo):
        self.coordenadas = coordenadas  # coordenadas como par ordenado [x, y]
        self.angulo = angulo



class PathPlanner:
    def __init__(self, largura, altura):
        self.largura = largura
        self.altura = altura
        self.missoes = []
        self.x_atual = 0  # Posição inicial no eixo X -
        self.y_atual = 0  # Posição inicial no eixo Y

    def adicionar_missao(self, missao):
        self.missoes.append(missao)

    def calcular_rota(self, coordenada_atual, missao_proxima):
        delta_x = missao_proxima.coordenadas[0] - coordenada_atual[0]
        delta_y = missao_proxima.coordenadas[1] - coordenada_atual[1]
        distancia = math.sqrt(delta_x**2 + delta_y**2)
        angulo = math.degrees(math.atan2(delta_y, delta_x))
        return round(distancia, 2), round(angulo, 2)

    def converter_distancia_para_graus(self, distancia_cm):
        return int((distancia_cm / 5) * 103)

    def calcular_distancia(self, x_atual, y_atual, x_destino, y_destino):
        return sqrt((x_destino - x_atual) ** 2 + (y_destino - y_atual) ** 2)

    def calcular_angulo(self, x_atual, y_atual, x_destino, y_destino):
        angulo_rad = atan2(y_destino - y_atual, x_destino - x_atual)
        return degrees(angulo_rad)
        
    async def mover_para_ponto(self, x_m, y_m):
        # Atualiza a posição atual do robô
        x_inicial, y_inicial = self.x_atual, self.y_atual

        # Calcula a distância e o ângulo até o ponto de destino
        distancia_cm = self.calcular_distancia(x_inicial, y_inicial, x_m, y_m) * 2  # Cada unidade = 2 cm
        angulo_graus = int(self.calcular_angulo(x_inicial, y_inicial, x_m, y_m))

        if angulo_graus == 0:  # Evitar divisão por zero
            motor_pair.move_for_degrees(motor_pair.PAIR_1, self.converter_distancia_para_graus(distancia_cm), 0, velocity=700)
        else:
            # Gira o robô para alinhar com o ângulo desejado
            motion_sensor.reset_yaw(0)
            print("Angulo em Graus {}".format(angulo_graus))

            if angulo_graus > 0:
                while round((motion_sensor.tilt_angles()[0] / 10)) != angulo_graus:
                    print("TILT {}".format(round(motion_sensor.tilt_angles()[0] / 10)))
                    motor_pair.move_tank(motor_pair.PAIR_1, int((-300 * angulo_graus) / angulo_graus), int((300 * angulo_graus) / angulo_graus))
                print("Giro Realizado com Sucesso!")
                motor_pair.stop(motor_pair.PAIR_1)

            else:
                while round((motion_sensor.tilt_angles()[0] / 10)) != angulo_graus:
                    print("TILT {}".format(round(motion_sensor.tilt_angles()[0] / 10)))
                    motor_pair.move_tank(motor_pair.PAIR_1, int((300 * angulo_graus) / angulo_graus), int((-300 * angulo_graus) / angulo_graus))
                print("Giro Realizado com Sucesso!")
                motor_pair.stop(motor_pair.PAIR_1)

        # Move o robô para frente pela distância calculada
        motor_pair.move_for_degrees(motor_pair.PAIR_1, self.converter_distancia_para_graus(distancia_cm), 0, velocity=spike.velocidade)

        # Atualiza a posição atual após o movimento
        # Calculando a nova posição
        # Atualiza a posição X do robô
        # A nova posição X é calculada somando:
        # - A posição inicial X Inicial (onde o robô estava no começo)
        # - A distância que o robô andou na direção do ângulo, mas só na parte horizontal (X).
        # Para isso, usamos:
        # - math.cos(): Ajuda a descobrir quanta da distância está na direção horizontal.
        # - math.radians(): Transforma o ângulo (em graus) para um formato que o programa entenda.
        self.x_atual = x_inicial + distancia_cm * math.cos(math.radians(angulo_graus))  # Atualiza posição X

        # Atualiza a posição Y do robô
        # A nova posição Y é calculada somando:
        # - A posição inicial Y (onde o robô estava no começo)
        # - A distância que o robô andou na direção do ângulo, mas só na parte vertical (Y).
        # Para isso, usamos:
        # - math.sin(): Ajuda a descobrir quanta da distância está na direção vertical.
        # - math.radians(): Transforma o ângulo (em graus) para um formato que o programa entenda.
        self.y_atual = y_inicial + distancia_cm * math.sin(math.radians(angulo_graus))  # Atualiza posição Y
        
        print("Posição atual: X = {}, Y = {}".format(self.x_atual, self.y_atual))

    async def imprimir_posicao(self):
        while True:
            print("Posição atual: X = {}, Y = {}".format(self.x_atual, self.y_atual))
            await runloop.sleep(0.7)  # Espera 0.7 segundos


# Classe principal do robô que controla movimentos e sensores
class Robot(PathPlanner, Controle_PID):

    def __init__(self,control_pid, largura, altura):
        self.velocidade = 0
        self.velocidadeDosAnexos = 0
        self.controle_pid = control_pid
        self.largura = largura
        self.altura = altura

    # Define os motores conectados nas portas especificadas
    async def Definir_Motores(self, pair, porta1, porta2):
        motor_pair.pair(pair, porta1, porta2)

    # Define a velocidade de movimento do robô
    async def Definir_Velocidade_Movimento(self, velocidade = 50):
        self.velocidade = velocidade * 10

    # Define a velocidade dos anexos (motores adicionais)
    async def Definir_Velocidade_Anexos(self, velocidade = 30):
        self.velocidadeDosAnexos = velocidade * 10

    # Reseta o ângulo de guinada para 0
    async def Manutencao_Guinada(self):
        motion_sensor.reset_yaw(0)

    # Função genérica para mover o robô com aceleração e desaceleração suave
    async def _movimento(self, pair, duration_s, power = 800):
        # Definição dos parâmetros iniciais para aceleração e desaceleração:

        power_inicial = 100 # Potência inicial dos motores
        power_min = 100 # Potência mínima para desaceleração
        power_incremento = 100 # Incremento de potência para acelerar
        intervalo = 0.15 # Intervalo de tempo entre os ajustes de potência (em segundos)

        # Reseta o ângulo de guinada para garantir um movimento estável
        await self.Manutencao_Guinada()

        # Marca o tempo de início e calcula o tempo final da movimentação
        start_time = time.ticks_ms() / 1000 # Tempo atual em segundos
        end_time = start_time + duration_s # Tempo total de movimentação (start + duração)

        # Define o ponto em que a desaceleração deve começar (0.7s{700ms} antes do final)
        deceleration_start = end_time - 1

        # Inicializa a potência atual com a potência inicial definida
        current_power = power_inicial

        # Loop que executa o movimento até o tempo final definido
        while time.ticks_ms() / 1000 < end_time:
            # Calcula as velocidades corrigidas usando o controle PID
            right_speed, left_speed = await self.calculate(0, current_power)

            # Move o robô usando as velocidades calculadas
            motor_pair.move_tank(motor_pair.PAIR_1, left_speed, right_speed)

            # Se o tempo atual ainda estiver antes do ponto de desaceleração
            if time.ticks_ms() / 1000 < deceleration_start:
                # Aumenta a potência suavemente, mas limita ao valor máximo (power)
                current_power = min(current_power + power_incremento, power)

            else:
                # Calcula o tempo restante até o final
                remaining_time = end_time - time.ticks_ms() / 1000

                # Fator de desaceleração proporcional ao tempo restante (0 a 1)
                deceleration_factor = remaining_time / 0.7

                # Ajusta a potência atual para desacelerar suavemente
                current_power = max(power_min, power_min + (power - power_min) * deceleration_factor)

            # Aguarda o intervalo definido antes de atualizar novamente
            time.sleep(intervalo)

        # Após o término do loop, para os motores
        motor_pair.move_tank(pair, 0, 0)

    # Movimenta o robô para frente
    async def mover_forward(self, duration_s, power = 500):
        await self._movimento(motor_pair.PAIR_1, duration_s, power)

    # Movimenta o robô para trás
    async def mover_backward(self, duration_s, power):
        await self._movimento(motor_pair.PAIR_1, duration_s, -power)

    # Gira o robô em um ângulo específico (Direita ou esquerda)
    async def Girar(self, angulo, velocidade = 300):
        await self.Manutencao_Guinada()

        if angulo > 0:
            while round((motion_sensor.tilt_angles()[0] / 10)) != angulo:
                print("TILT {}" .format(round(motion_sensor.tilt_angles()[0] / 10)))
                motor_pair.move_tank(motor_pair.PAIR_1, int((-velocidade * angulo) / angulo), int((velocidade * angulo) / angulo))
            print("Giro Realizado com Sucesso!")
            motor_pair.stop(motor_pair.PAIR_1)
        else:
            while round((motion_sensor.tilt_angles()[0] / 10)) != angulo:
                print("TILT {}" .format(round(motion_sensor.tilt_angles()[0] / 10)))
                motor_pair.move_tank(motor_pair.PAIR_1, int((velocidade * angulo)), int((-velocidade * angulo)))
            print("Giro Realizado com Sucesso!")
            motor_pair.stop(motor_pair.PAIR_1) 

    # Função para mover um motor específico em uma direção por um tempo determinado - Prioridade a Anexos
    async def MoverMotor(self, direcao: str, port1: int, duracao:int):

        # Define um multiplicador que ajusta o sentido do movimento do motor
        # Se a direção for 'subir', o multiplicador será 1 (movimento normal).
        # Se a direção for diferente de 'subir', o multiplicador será -1 (movimento inverso).
        multiplicador = lambda direcao: 1 if direcao == 'subir' else -1

        # Executa o motor na porta especificada (port1) por uma duração (duracao) e com uma velocidade ajustada.
        # A velocidade é definida pela variável `self.velocidadeDosAnexos` multiplicada pelo multiplicador,
        # que garante a direção correta.
        await motor.run_for_time(port1, duracao, self.velocidadeDosAnexos * multiplicador(direcao))
        motor.stop(port.A, stop=HOLD)


    # Função para o Robo parar de Mover
    async def Parar_de_Mover(self):
        motor_pair.stop(motor_pair.PAIR_1)

    # Função que retorna a cor atual lida pelo Sensor de Cor
    async def VerificarCorSensor(self, port):
        return color_sensor.color(port.C)

    # Função que Retorna a intensidade da luz refletida pelo Sensor de Cor
    async def VerificarLuzRefletida(self, port):
        return color_sensor.reflection(port.C)

    # Função para Mostrar um grafico de Linhas com valor x e y alteraveis (Sem tela cheia)
    async def PlotarLineGraphic(self,valor_x, valor_y):
        linegraph.show(False)
        linegraph.plot(color.RED, valor_x, valor_y)

    # Função para limpar todos os Graficos de Linha
    async def LimparLineGraphic(self):
        linegraph.clear_all()

# Classe para Organizar Lancamentos (Em desenvolvimento)
class Lancamentos:
    async def L01(self):
        await spike.mover_forward(1)

# Instancia o Controle PID com valores iniciais
pid = Controle_PID(1.5, 0.01, 1)

# Instancia do robô passando o controle PID e as medidas da arena para o PathPlanner - em cm.
spike = Robot(pid, 220, 140)

lancamento = Lancamentos()

# Função principal
async def main():
    await spike.Definir_Motores(parFrente, port.C, port.B)

    await spike.Definir_Velocidade_Movimento(60)
    await spike.Definir_Velocidade_Anexos(velocidade = 35)

    await spike.Manutencao_Guinada()

    await spike.mover_forward(3)
    await spike.Girar(90)

    await spike.mover_para_ponto(10, 0)


#Função Secundária para armazenar toda a Logica de Sensor de Cor (Metodo dos Aneis)
async def ButtonColor():
    while True:
        # Variável que sempre armazena a cor atual lida pelo sensor de cor
        cor = await spike.VerificarCorSensor(port.E)
        
        if button.pressed(button.RIGHT):
            # Se a cor for vermelha
            if cor == color.RED:
                await lancamento.L01()

            # Se a cor for branca
            elif cor == color.WHITE:
                pass

            # Se a cor for verde
            elif cor == color.GREEN:
                pass

            # Se a cor for azul
            elif cor == color.BLUE:
                pass

            # Se a cor for amarela
            elif cor == color.YELLOW:
                pass

        # Senão, se o botão esquerdo for pressionado durante o programa
        elif button.pressed(button.LEFT):
            # Se a cor for vermelha
            if cor == color.RED:
                await lancamento.L01()

            # Se a cor for branca
            elif cor == color.WHITE:
                pass

            # Se a cor for verde
            elif cor == color.GREEN:
                pass

            # Se a cor for azul
            elif cor == color.BLUE:
                pass

            # Se a cor for amarela
            elif cor == color.YELLOW:
                pass


runloop.run(main(),ButtonColor())