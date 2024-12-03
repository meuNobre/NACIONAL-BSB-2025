from hub import motion_sensor, port
import motor
import runloop, motor_pair, time

subir = 'subir'
descer = 'descer'

class Controle_PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_sum = 0
        self.last_error = 0

    async def _guinada_atual(self):
        return motion_sensor.tilt_angles()[0]

    async def _catch_erro(self, target):
        angulo_atual = await self._guinada_atual()
        error = target - angulo_atual
        return error

    async def _calculo_integral(self, error):
        self.integral_sum += error
        return self.integral_sum

    async def _calculo_derivada(self, error):
        derivative = error - self.last_error
        self.last_error = error
        return derivative

    async def Calcular(self, target, power):
        error = await self._catch_erro(target)
        integral_sum = await self._calculo_integral(error)
        derivative = await self._calculo_derivada(error)

        correction = self.kp * error
        correction_integral = self.ki * integral_sum
        correction_derivative = self.kd * derivative

        ajuste_final = correction + correction_integral + correction_derivative

        velocidade_direita = power - ajuste_final
        velocidade_esquerda = power + ajuste_final

        return int(velocidade_direita), int(velocidade_esquerda)

PID = Controle_PID(1, 0.01, 1)

class Robo:
    def __init__(self):
        self.velocidade = self.velocidade
        self.velocidadeDosAnexos = self.velocidadeDosAnexos
        self.parFrente = self.parFrente
        self.parTras = self.parTras

    async def Definir_Motores_Frente(self, porta1, porta2):
        motor_pair.pair(motor_pair.PAIR_1, porta1, porta2)
    
    async def Definir_Motores_Tras(self, porta1, porta2):
        motor_pair.pair(motor_pair.PAIR_2, porta1, porta2)

    async def Definir_Velocidade_Movimento(self, velocidade = 500):
        self.velocidade = velocidade

    async def Definir_Velocidade_Anexos(self, velocidade = 300):
        self.velocidadeDosAnexos = velocidade

    async def Manutencao_Guinada(self):
        motion_sensor.reset_yaw(0)        


    async def Movimentar_Robo_Frente(self, duration_s, power=400):
        velocidade_inicial = 400 # Potência inicial
        potencia_minima = 100 # Potência mínima durante a desaceleração
        incremento = 25
        passo_tempo = 0.05# Intervalo de incremento

        await self.Manutencao_Guinada()

        start_time = time.ticks_ms() / 1000
        end_time = start_time + duration_s

        desaceleracao_inicio = end_time - 0.7# Início da desaceleração 
        potencia_atual = velocidade_inicial

        while time.ticks_ms() / 1000 < end_time:

            vel_direita, vel_esquerda = await PID.Calcular(target=0, power=potencia_atual)

            motor_pair.move_tank(motor_pair.PAIR_1, vel_direita, vel_esquerda)

            # Verifica se está no período de desaceleração
            if time.ticks_ms() / 1000 < desaceleracao_inicio:
                potencia_atual = min(potencia_atual + incremento, power)
            else:
                tempo_restante = end_time - time.ticks_ms() / 1000
                fator_desaceleracao = tempo_restante / 0.7  # Escala linear nos últimos 700 ms
                potencia_atual = max(potencia_minima, potencia_minima + (power - potencia_minima) * fator_desaceleracao)

            # Aguarda o próximo passo
            time.sleep(passo_tempo)
        motor_pair.move_tank(motor_pair.PAIR_1, 0, 0)

    async def Movimentar_Robo_Tras(self, duration_s, power=400):
        velocidade_inicial = 400 # Potência inicial
        potencia_minima = 100 # Potência mínima durante a desaceleração
        incremento = 25
        passo_tempo = 0.05# Intervalo de incremento

        motion_sensor.reset_yaw(0)

        start_time = time.ticks_ms() / 1000
        end_time = start_time + duration_s

        desaceleracao_inicio = end_time - 0.7# Início da desaceleração
        potencia_atual = velocidade_inicial

        while time.ticks_ms() / 1000 < end_time:

            vel_direita, vel_esquerda = await PID.Calcular(target=0, power=potencia_atual)

            motor_pair.move_tank(motor_pair.PAIR_2, vel_direita, vel_esquerda)

            # Verifica se está no período de desaceleração
            if time.ticks_ms() / 1000 < desaceleracao_inicio:
                potencia_atual = min(potencia_atual + incremento, power)
            else:
                tempo_restante = end_time - time.ticks_ms() / 1000
                fator_desaceleracao = tempo_restante / 0.7# Escala linear nos últimos 700 ms
                potencia_atual = max(potencia_minima, potencia_minima + (power - potencia_minima) * fator_desaceleracao)

            # Aguarda o próximo passo
            time.sleep(passo_tempo)
        motor_pair.move_tank(motor_pair.PAIR_2, 0, 0)

    async def Turn_Direita(self, angulo, velocidade):
        await self.Manutencao_Guinada()
        while round((motion_sensor.tilt_angles()[0] / 10)) != angulo:
            print("Angulo Atual {}" .format(round(motion_sensor.tilt_angles()[0] / 10)))
            motor_pair.move_tank(motor_pair.PAIR_1, velocidade, -velocidade)
        motor_pair.stop(motor_pair.PAIR_1)

    async def Turn_Esquerda(self,angulo, velocidade):
        await self.Manutencao_Guinada()
        while round((motion_sensor.tilt_angles()[0] / 10)) != angulo:
            print("Angulo Atual {}" .format(round(motion_sensor.tilt_angles()[0] / 10)))
            motor_pair.move_tank(motor_pair.PAIR_1, -velocidade, velocidade)
        motor_pair.stop(motor_pair.PAIR_1)

    async def Girar(self, angulo, velocidade = 200):
        await self.Manutencao_Guinada()

        if angulo > 0:
            await self.Turn_Esquerda(angulo, velocidade)

        elif angulo < 0:
            await self.Turn_Direita(angulo, velocidade)

    async def MoverMotor(self, direcao: str, port1: int, duracao:int):
        #Subir Anexo/Direita
        if direcao == 'subir':
            await motor.run_for_time(port1, duracao, self.velocidadeDosAnexos * 1)

        #Descer Anexo/Esquerda
        if direcao == 'descer':
            await motor.run_for_time(port1, duracao, self.velocidadeDosAnexos * -1)

    async def Parar_de_Mover(self):
        motor_pair.stop(motor_pair.PAIR_1)
        motor_pair.stop(motor_pair.PAIR_2)

spike = Robo()

# Função principal
async def main():
    await spike.Definir_Motores_Frente(port.A, port.B)
    await spike.Definir_Motores_Tras(port.B, port.A)
    
    await spike.Definir_Velocidade_Movimento(600)
    await spike.Definir_Velocidade_Anexos(350)
    
    await spike.Manutencao_Guinada()
    await spike.Movimentar_Robo_Tras(3, 600)
    await spike.Movimentar_Robo_Tras(3, 600)
    await spike.Girar(90)

runloop.run(main())
