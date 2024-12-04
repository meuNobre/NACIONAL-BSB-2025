from hub import motion_sensor, port, button
import motor
import runloop, motor_pair, time, color_sensor, color
from app import linegraph

# Definição de constantes que representam comandos de movimento
subir = 'subir'
descer = 'descer'
parFrente = motor_pair.PAIR_1 # Par de motores frontais
parBack = motor_pair.PAIR_2 # Par de motores traseiros

class Controle_PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp # Proporcional
        self.ki = ki # Integral
        self.kd = kd # Derivativo
        self.integral_sum = 0 # Soma do erro integral
        self.last_error = 0 # Erro anterior usado no cálculo do derivativo

    # Obtém o ângulo atual da guinada(yaw) do robô
    async def _get_current_yaw(self):
        return motion_sensor.tilt_angles()[0]

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
        correction = self.kp * error + self.ki * integral + self.kd * derivative

        # Ajusta as velocidades dos motores com base na correção
        left_speed = power + correction
        right_speed = power - correction
        
        return int(left_speed), int(right_speed)

# Classe principal do robô que controla movimentos e sensores
class Robot:
    def __init__(self,control_pid):
        self.velocidade = self.velocidade
        self.velocidadeDosAnexos = self.velocidadeDosAnexos
        self.controle_pid = control_pid

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
    async def _movimento(self, pair, duration_s, power = 400):
        # Definição dos parâmetros iniciais para aceleração e desaceleração:

        power_inicial = 100 # Potência inicial dos motores
        power_min = 100 # Potência mínima para desaceleração
        power_incremento = 25 # Incremento de potência para acelerar
        intervalo = 0.05 # Intervalo de tempo entre os ajustes de potência (em segundos)

        # Reseta o ângulo de guinada para garantir um movimento estável
        await self.Manutencao_Guinada()

        # Marca o tempo de início e calcula o tempo final da movimentação
        start_time = time.ticks_ms() / 1000 # Tempo atual em segundos
        end_time = start_time + duration_s # Tempo total de movimentação (start + duração)

        # Define o ponto em que a desaceleração deve começar (0.7s{700ms} antes do final)
        deceleration_start = end_time - 0.7
        
        # Inicializa a potência atual com a potência inicial definida
        current_power = power_inicial

        # Loop que executa o movimento até o tempo final definido
        while time.ticks_ms() / 1000 < end_time:
            # Calcula as velocidades corrigidas usando o controle PID
            left_speed, right_speed = await self.controle_pid.calculate(0, current_power)

            # Move o robô usando as velocidades calculadas
            motor_pair.move_tank(pair, left_speed, right_speed)

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
    async def Girar(self, angulo, velocidade = 20):
        await self.Manutencao_Guinada()

        while round((motion_sensor.tilt_angles()[0] / 10)) != angulo:

            print("Angulo Atual {}" .format(round(motion_sensor.tilt_angles()[0] / 10))) #Printar Angulo(Guinada) Atual.
 
            motor_pair.move_tank(motor_pair.PAIR_1, (((velocidade * 10) * (angulo)) / (angulo)), (((-velocidade * 10) * (angulo)) / (angulo) ))

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

    # Função para o Robot parar de Mover
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

# Instancia do robô e execução de funções principais
spike = Robot(pid)

lancamento = Lancamentos()

# Função principal
async def main():
    await spike.Definir_Motores(parFrente, port.A, port.B)
    await spike.Definir_Motores(parBack, port.B, port.A)

    await spike.Definir_Velocidade_Movimento(velocidade = 600)
    await spike.Definir_Velocidade_Anexos(velocidade = 350)

    await spike.Manutencao_Guinada()

    await spike.mover_forward(duration_s = 3, power = 600)
    await spike.mover_backward(duration_s = 3, power = 600)
    await spike.Girar(angulo =  90)

#Função Secundária para armazenar toda a Logica de Sensor de Cor (Metodo dos Aneis)
async def ButtonColor():
    while True:
        #Variavel que sempre armazena a cor atual lida pelo sensor de cor
        cor = await spike.VerificarCorSensor(port.C)

        #Se o Botao direito por pressionado durante o programa
        if button.pressed(button.RIGHT):
            
            match cor:
                #Caso a cor armazenada na função for Vermelha
                case color.RED:
                    await lancamento.L01()

                #Caso a cor armazenada na função for Branca
                case color.WHITE:
                    pass                

                #Caso a cor armazenada na função for Verde
                case color.GREEN:
                    pass

                #Caso a cor armazenada na função for Azul
                case color.BLUE:
                    pass

                #Caso a cor armazenada na função for Verde
                case color.GREEN:
                    pass

                #Caso a cor armazenada na função for Amarela
                case color.YELLOW:
                    pass

        #  Senão, se o Botao Esquerdo por pressionado durante o programa
        elif button.pressed(button.LEFT):
            match cor:
                #Caso a cor armazenada na função for Vermelha
                case color.RED:
                    await lancamento.L01()

                #Caso a cor armazenada na função for Branca
                case color.WHITE:
                    pass

                #Caso a cor armazenada na função for Verde
                case color.GREEN:
                    pass

                #Caso a cor armazenada na função for Azul
                case color.BLUE:
                    pass

                #Caso a cor armazenada na função for Verde
                case color.GREEN:
                    pass

                #Caso a cor armazenada na função for Amarela
                case color.YELLOW:
                    pass

runloop.run(main(),ButtonColor())
